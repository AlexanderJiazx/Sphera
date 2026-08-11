import Accelerate
import CoreML
import Foundation
import UIKit

struct LoFTRCorrespondence: Sendable {
  var x0: Float
  var y0: Float
  var x1: Float
  var y1: Float
  var confidence: Float
}

struct LoFTRPairMatches: Sendable {
  var sourceIndex: Int
  var targetIndex: Int
  var correspondences: [LoFTRCorrespondence]
}

/// On-device outdoor LoFTR (coarse): CoreML backbone + coarse transformer,
/// dual-softmax mutual nearest-neighbor matching via Accelerate BLAS.
final class LoFTRMatcher: @unchecked Sendable {
  struct Config: Sendable {
    var inputHeight: Int
    var inputWidth: Int
    var coarseHeight: Int
    var coarseWidth: Int
    var coarseStride: Int
    var temperature: Float
    var threshold: Float
    var borderRm: Int
  }

  private let backbone: MLModel
  private let coarse: MLModel
  let config: Config
  private let backboneImageName: String
  private let coarseFeat0Name: String
  private let coarseFeat1Name: String
  private let coarseOut0Name: String
  private let coarseOut1Name: String

  /// - Parameter onStage: optional progress callbacks during model load.
  init(onStage: ((String) -> Void)? = nil) throws {
    guard
      let backboneURL = Bundle.main.url(
        forResource: "LoFTRBackbone", withExtension: "mlmodelc")
        ?? Bundle.main.url(forResource: "LoFTRBackbone", withExtension: "mlpackage"),
      let coarseURL = Bundle.main.url(
        forResource: "LoFTRCoarse", withExtension: "mlmodelc")
        ?? Bundle.main.url(forResource: "LoFTRCoarse", withExtension: "mlpackage")
    else {
      throw LoFTRMatcherError.modelMissing
    }

    // `.all` (Neural Engine) hangs on first load for these models on device.
    // CPU+GPU specializes reliably and still runs fast enough.
    let mlConfig = MLModelConfiguration()
    mlConfig.computeUnits = .cpuAndGPU

    onStage?("Loading LoFTR backbone")
    NSLog("LoFTR loading backbone from %@", backboneURL.lastPathComponent)
    let loadStart = CFAbsoluteTimeGetCurrent()
    if backboneURL.pathExtension == "mlpackage" {
      let compiledBackbone = try MLModel.compileModel(at: backboneURL)
      backbone = try MLModel(contentsOf: compiledBackbone, configuration: mlConfig)
    } else {
      backbone = try MLModel(contentsOf: backboneURL, configuration: mlConfig)
    }
    NSLog("LoFTR backbone ready in %.2fs", CFAbsoluteTimeGetCurrent() - loadStart)

    onStage?("Loading LoFTR coarse matcher")
    let coarseStart = CFAbsoluteTimeGetCurrent()
    if coarseURL.pathExtension == "mlpackage" {
      let compiledCoarse = try MLModel.compileModel(at: coarseURL)
      coarse = try MLModel(contentsOf: compiledCoarse, configuration: mlConfig)
    } else {
      coarse = try MLModel(contentsOf: coarseURL, configuration: mlConfig)
    }
    NSLog("LoFTR coarse ready in %.2fs", CFAbsoluteTimeGetCurrent() - coarseStart)

    if let configURL = Bundle.main.url(forResource: "config", withExtension: "json"),
      let data = try? Data(contentsOf: configURL),
      let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any]
    {
      config = Config(
        inputHeight: json["input_height"] as? Int ?? 640,
        inputWidth: json["input_width"] as? Int ?? 480,
        coarseHeight: json["coarse_height"] as? Int ?? 80,
        coarseWidth: json["coarse_width"] as? Int ?? 60,
        coarseStride: json["coarse_stride"] as? Int ?? 8,
        temperature: Float(json["temperature"] as? Double ?? 0.1),
        threshold: Float(json["threshold"] as? Double ?? 0.2),
        borderRm: json["border_rm"] as? Int ?? 2
      )
    } else {
      config = Config(
        inputHeight: 640,
        inputWidth: 480,
        coarseHeight: 80,
        coarseWidth: 60,
        coarseStride: 8,
        temperature: 0.1,
        threshold: 0.2,
        borderRm: 2
      )
    }

    backboneImageName =
      backbone.modelDescription.inputDescriptionsByName.keys.first ?? "image"
    let coarseInputs = Array(coarse.modelDescription.inputDescriptionsByName.keys).sorted()
    coarseFeat0Name = coarseInputs.count > 0 ? coarseInputs[0] : "feat_c0"
    coarseFeat1Name = coarseInputs.count > 1 ? coarseInputs[1] : "feat_c1"
    let coarseOutputs = Array(coarse.modelDescription.outputDescriptionsByName.keys).sorted()
    coarseOut0Name = coarseOutputs.count > 0 ? coarseOutputs[0] : "input_255"
    coarseOut1Name = coarseOutputs.count > 1 ? coarseOutputs[1] : "var_1129"
  }

  func extractCoarseFeature(from image: UIImage) throws -> MLMultiArray {
    let tensor = try grayTensor(from: image)
    let provider = try MLDictionaryFeatureProvider(dictionary: [
      backboneImageName: MLFeatureValue(multiArray: tensor)
    ])
    let out = try backbone.prediction(from: provider)
    var chosen: MLMultiArray?
    for name in out.featureNames {
      guard let array = out.featureValue(for: name)?.multiArrayValue,
        array.shape.count == 4
      else { continue }
      if array.shape[1].intValue == 256 {
        return array
      }
      if chosen == nil { chosen = array }
    }
    guard let chosen else { throw LoFTRMatcherError.inferenceFailed("backbone") }
    return chosen
  }

  func matchPair(
    feat0: MLMultiArray,
    feat1: MLMultiArray,
    sourceIndex: Int,
    targetIndex: Int
  ) throws -> LoFTRPairMatches {
    let pairStart = CFAbsoluteTimeGetCurrent()
    NSLog("LoFTR matchPair begin %d -> %d", sourceIndex, targetIndex)
    let tokens = try coarseTokens(feat0: feat0, feat1: feat1)
    NSLog(
      "LoFTR matchPair tokens %d -> %d  n0=%d n1=%d dim=%d  %.2fs",
      sourceIndex,
      targetIndex,
      tokens.2,
      tokens.3,
      tokens.4,
      CFAbsoluteTimeGetCurrent() - pairStart
    )
    let correspondences = dualSoftmaxMatches(
      tokens0: tokens.0,
      tokens1: tokens.1,
      n0: tokens.2,
      n1: tokens.3,
      dim: tokens.4
    )
    NSLog(
      "LoFTR matchPair end %d -> %d  matches=%d  %.2fs",
      sourceIndex,
      targetIndex,
      correspondences.count,
      CFAbsoluteTimeGetCurrent() - pairStart
    )
    return LoFTRPairMatches(
      sourceIndex: sourceIndex,
      targetIndex: targetIndex,
      correspondences: correspondences
    )
  }

  func writeCache(
    pairs: [LoFTRPairMatches],
    imageNames: [String],
    to directory: URL
  ) throws {
    let fm = FileManager.default
    try fm.createDirectory(at: directory, withIntermediateDirectories: true)
    var pairRecords: [[String: Any]] = []
    for pair in pairs {
      let fileName = String(format: "pair_%02d_%02d.bin", pair.sourceIndex, pair.targetIndex)
      let fileURL = directory.appendingPathComponent(fileName)
      var floats: [Float] = []
      floats.reserveCapacity(pair.correspondences.count * 5)
      for c in pair.correspondences {
        floats.append(contentsOf: [c.x0, c.y0, c.x1, c.y1, c.confidence])
      }
      let data = floats.withUnsafeBufferPointer { Data(buffer: $0) }
      try data.write(to: fileURL, options: .atomic)
      pairRecords.append([
        "source": pair.sourceIndex,
        "target": pair.targetIndex,
        "file": fileName,
        "completed": true,
        "matches": pair.correspondences.count,
      ])
    }
    let manifest: [String: Any] = [
      "format_version": 1,
      "model": "loftr_outdoor_coarse_coreml",
      "image_names": imageNames,
      "input_height": config.inputHeight,
      "input_width": config.inputWidth,
      "pair_count": pairs.count,
      "pairs": pairRecords,
    ]
    let manifestData = try JSONSerialization.data(
      withJSONObject: manifest, options: [.prettyPrinted, .sortedKeys]
    )
    try manifestData.write(
      to: directory.appendingPathComponent("manifest.json"), options: .atomic
    )
  }

  private func grayTensor(from image: UIImage) throws -> MLMultiArray {
    guard let source = image.cgImage else {
      throw LoFTRMatcherError.imageConversionFailed
    }
    let width = config.inputWidth
    let height = config.inputHeight
    var pixels = [UInt8](repeating: 0, count: width * height * 4)
    let colorSpace = CGColorSpaceCreateDeviceRGB()
    guard
      let context = CGContext(
        data: &pixels,
        width: width,
        height: height,
        bitsPerComponent: 8,
        bytesPerRow: width * 4,
        space: colorSpace,
        bitmapInfo: CGImageAlphaInfo.noneSkipLast.rawValue
          | CGBitmapInfo.byteOrder32Big.rawValue
      )
    else {
      throw LoFTRMatcherError.imageConversionFailed
    }
    context.interpolationQuality = .medium
    drawDisplayOriented(
      source,
      orientation: image.imageOrientation,
      into: context,
      width: width,
      height: height
    )

    // Models are fp16 — feed Float16 tensors (not Float32).
    let array = try MLMultiArray(
      shape: [1, 1, NSNumber(value: height), NSNumber(value: width)],
      dataType: .float16
    )
    let count = width * height
    let ptr = array.dataPointer.bindMemory(to: Float16.self, capacity: count)
    for i in 0..<count {
      let o = i * 4
      let r = Float(pixels[o])
      let g = Float(pixels[o + 1])
      let b = Float(pixels[o + 2])
      ptr[i] = Float16((0.299 * r + 0.587 * g + 0.114 * b) / 255.0)
    }
    return array
  }

  /// `UIImage.cgImage` is the encoded sensor bitmap: EXIF orientation rides on the
  /// UIImage and is never baked into those pixels. The native engine registers the
  /// display-oriented image, so LoFTR has to see the same axes or every cached
  /// coordinate lands rotated relative to the OpenCV features.
  private func drawDisplayOriented(
    _ source: CGImage,
    orientation: UIImage.Orientation,
    into context: CGContext,
    width: Int,
    height: Int
  ) {
    let w = CGFloat(width)
    let h = CGFloat(height)
    var transform = CGAffineTransform.identity
    switch orientation {
    case .down, .downMirrored:
      transform = transform.translatedBy(x: w, y: h).rotated(by: .pi)
    case .left, .leftMirrored:
      transform = transform.translatedBy(x: w, y: 0).rotated(by: .pi / 2)
    case .right, .rightMirrored:
      transform = transform.translatedBy(x: 0, y: h).rotated(by: -.pi / 2)
    default:
      break
    }
    switch orientation {
    case .upMirrored, .downMirrored:
      transform = transform.translatedBy(x: w, y: 0).scaledBy(x: -1, y: 1)
    case .leftMirrored, .rightMirrored:
      transform = transform.translatedBy(x: h, y: 0).scaledBy(x: -1, y: 1)
    default:
      break
    }
    context.concatenate(transform)
    switch orientation {
    case .left, .leftMirrored, .right, .rightMirrored:
      // Axes swap, so the pre-rotation footprint is height x width.
      context.draw(source, in: CGRect(x: 0, y: 0, width: h, height: w))
    default:
      context.draw(source, in: CGRect(x: 0, y: 0, width: w, height: h))
    }
  }

  private func coarseTokens(feat0: MLMultiArray, feat1: MLMultiArray) throws -> (
    [Float], [Float], Int, Int, Int
  ) {
    let provider = try MLDictionaryFeatureProvider(dictionary: [
      coarseFeat0Name: MLFeatureValue(multiArray: feat0),
      coarseFeat1Name: MLFeatureValue(multiArray: feat1),
    ])
    let out = try coarse.prediction(from: provider)
    var arrays: [MLMultiArray] = []
    for name in [coarseOut0Name, coarseOut1Name] {
      if let value = out.featureValue(for: name)?.multiArrayValue {
        arrays.append(value)
      }
    }
    if arrays.count < 2 {
      for name in out.featureNames.sorted() {
        if let value = out.featureValue(for: name)?.multiArrayValue {
          arrays.append(value)
        }
      }
    }
    guard arrays.count >= 2 else {
      throw LoFTRMatcherError.inferenceFailed("coarse")
    }
    let (t0, n0, c0) = try packedTokens(arrays[0])
    let (t1, n1, c1) = try packedTokens(arrays[1])
    guard c0 == c1, n0 > 0, n1 > 0 else {
      throw LoFTRMatcherError.inferenceFailed("coarse-dim")
    }
    return (t0, t1, n0, n1, c0)
  }

  /// Safely copy CoreML tensors. Outputs are Float16 — treating them as Float32
  /// caused EXC_BAD_ACCESS in Array.init/memmove (crash at ~28% / first pair).
  private func packedTokens(_ array: MLMultiArray) throws -> ([Float], Int, Int) {
    let shape = array.shape.map(\.intValue)
    let n: Int
    let c: Int
    switch shape.count {
    case 3:
      // [1, N, C]
      n = shape[1]
      c = shape[2]
    case 2:
      n = shape[0]
      c = shape[1]
    case 4:
      // [1, C, H, W] → N=H*W tokens
      c = shape[1]
      n = shape[2] * shape[3]
    default:
      throw LoFTRMatcherError.inferenceFailed("token-shape-\(shape)")
    }

    let count = n * c
    guard count > 0, array.count >= count else {
      throw LoFTRMatcherError.inferenceFailed("token-count")
    }

    var out = [Float](repeating: 0, count: count)
    switch array.dataType {
    case .float16:
      let src = array.dataPointer.bindMemory(to: Float16.self, capacity: array.count)
      if Self.isContiguous(array) {
        for i in 0..<count {
          out[i] = Float(src[i])
        }
      } else {
        for i in 0..<count {
          out[i] = array[i].floatValue
        }
      }
    case .float32:
      let src = array.dataPointer.bindMemory(to: Float.self, capacity: array.count)
      if Self.isContiguous(array) {
        out.withUnsafeMutableBufferPointer { dst in
          dst.baseAddress!.update(from: src, count: count)
        }
      } else {
        for i in 0..<count {
          out[i] = array[i].floatValue
        }
      }
    default:
      for i in 0..<count {
        out[i] = Float(truncating: array[i])
      }
    }
    return (out, n, c)
  }

  private static func isContiguous(_ array: MLMultiArray) -> Bool {
    let shape = array.shape.map(\.intValue)
    let strides = array.strides.map(\.intValue)
    guard shape.count == strides.count, !shape.isEmpty else { return false }
    var expected = 1
    for i in stride(from: shape.count - 1, through: 0, by: -1) {
      if strides[i] != expected { return false }
      expected *= shape[i]
    }
    return true
  }

  private func dualSoftmaxMatches(
    tokens0: [Float],
    tokens1: [Float],
    n0: Int,
    n1: Int,
    dim: Int
  ) -> [LoFTRCorrespondence] {
    let hc = config.coarseHeight
    let wc = config.coarseWidth
    let thr = config.threshold
    let border = config.borderRm
    let temp = max(config.temperature, 1e-6)

    // One gemm buffer, then two softmax buffers (exact dual-softmax).
    // Avoid the old 4× full-matrix peak that could jetsam after the fp16 crash was fixed.
    var sim = [Float](repeating: 0, count: n0 * n1)
    tokens0.withUnsafeBufferPointer { aBuf in
      tokens1.withUnsafeBufferPointer { bBuf in
        sim.withUnsafeMutableBufferPointer { sBuf in
          cblas_sgemm(
            CblasRowMajor,
            CblasNoTrans,
            CblasTrans,
            Int32(n0),
            Int32(n1),
            Int32(dim),
            1.0 / temp,
            aBuf.baseAddress,
            Int32(dim),
            bBuf.baseAddress,
            Int32(dim),
            0,
            sBuf.baseAddress,
            Int32(n1)
          )
        }
      }
    }

    var rowProb = sim
    for i in 0..<n0 {
      rowProb.withUnsafeMutableBufferPointer { buf in
        let row = buf.baseAddress!.advanced(by: i * n1)
        var maxV: Float = 0
        vDSP_maxv(row, 1, &maxV, vDSP_Length(n1))
        var negMax = -maxV
        vDSP_vsadd(row, 1, &negMax, row, 1, vDSP_Length(n1))
        var count = Int32(n1)
        vvexpf(row, row, &count)
        var sum: Float = 0
        vDSP_sve(row, 1, &sum, vDSP_Length(n1))
        var inv = 1 / max(sum, 1e-12)
        vDSP_vsmul(row, 1, &inv, row, 1, vDSP_Length(n1))
      }
    }

    // Column softmax on a transposed copy (vDSP_mtrans — not Swift nested loops).
    var colProb = [Float](repeating: 0, count: n0 * n1)
    sim.withUnsafeBufferPointer { src in
      colProb.withUnsafeMutableBufferPointer { dst in
        // sim is n0 rows × n1 cols → transpose to n1 rows × n0 cols
        vDSP_mtrans(
          src.baseAddress!, 1,
          dst.baseAddress!, 1,
          vDSP_Length(n1),
          vDSP_Length(n0)
        )
      }
    }
    sim.removeAll(keepingCapacity: false)
    NSLog("LoFTR dualSoftmax transposed n=%d", n0)

    for j in 0..<n1 {
      colProb.withUnsafeMutableBufferPointer { buf in
        let col = buf.baseAddress!.advanced(by: j * n0)
        var maxV: Float = 0
        vDSP_maxv(col, 1, &maxV, vDSP_Length(n0))
        var negMax = -maxV
        vDSP_vsadd(col, 1, &negMax, col, 1, vDSP_Length(n0))
        var count = Int32(n0)
        vvexpf(col, col, &count)
        var sum: Float = 0
        vDSP_sve(col, 1, &sum, vDSP_Length(n0))
        var inv = 1 / max(sum, 1e-12)
        vDSP_vsmul(col, 1, &inv, col, 1, vDSP_Length(n0))
      }
    }

    // conf[i,j] = rowProb[i,j] * colProb_T[j,i]; find row/col argmax of conf
    var bestJ = [Int](repeating: 0, count: n0)
    var bestP = [Float](repeating: -1, count: n0)
    for i in 0..<n0 {
      var peak: Float = -1
      var peakJ = 0
      let rowBase = i * n1
      for j in 0..<n1 {
        let p = rowProb[rowBase + j] * colProb[j * n0 + i]
        if p > peak {
          peak = p
          peakJ = j
        }
      }
      bestJ[i] = peakJ
      bestP[i] = peak
    }

    var bestI = [Int](repeating: 0, count: n1)
    for j in 0..<n1 {
      var peak: Float = -1
      var peakI = 0
      let colBase = j * n0
      for i in 0..<n0 {
        let p = rowProb[i * n1 + j] * colProb[colBase + i]
        if p > peak {
          peak = p
          peakI = i
        }
      }
      bestI[j] = peakI
    }
    rowProb.removeAll(keepingCapacity: false)
    colProb.removeAll(keepingCapacity: false)

    var matches: [LoFTRCorrespondence] = []
    matches.reserveCapacity(256)
    let stride = Float(config.coarseStride)
    for i in 0..<n0 {
      let yi = i / wc
      let xi = i % wc
      if xi < border || yi < border || xi >= wc - border || yi >= hc - border {
        continue
      }
      let j = bestJ[i]
      let p = bestP[i]
      if p < thr || bestI[j] != i { continue }
      let yj = j / wc
      let xj = j % wc
      if xj < border || yj < border || xj >= wc - border || yj >= hc - border {
        continue
      }
      matches.append(
        LoFTRCorrespondence(
          x0: (Float(xi) + 0.5) * stride,
          y0: (Float(yi) + 0.5) * stride,
          x1: (Float(xj) + 0.5) * stride,
          y1: (Float(yj) + 0.5) * stride,
          confidence: p
        )
      )
    }

    if matches.count > 600 {
      matches.sort { $0.confidence > $1.confidence }
      matches = Array(matches.prefix(600))
    }
    return matches
  }
}

enum LoFTRMatcherError: LocalizedError {
  case modelMissing
  case imageConversionFailed
  case inferenceFailed(String)

  var errorDescription: String? {
    switch self {
    case .modelMissing:
      "LoFTR CoreML models are missing from the app bundle."
    case .imageConversionFailed:
      "Could not convert an image for LoFTR."
    case .inferenceFailed(let stage):
      "LoFTR \(stage) inference failed."
    }
  }
}
