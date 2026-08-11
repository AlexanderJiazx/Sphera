import CoreML
import Foundation
import UIKit

/// Builds LoFTR match caches for topology-approved pairs, then runs the native
/// OpenCV stitcher with those matches injected (augment mode).
final class OpenCVSpheraEngine: NativeSpheraEngine, @unchecked Sendable {
  func stitch(
    _ request: SpheraEngineRequest,
    progress: StitchProgressHandler?
  ) async throws -> StitchingResult {
    let matchCacheURL = request.outputDirectoryURL
      .appendingPathComponent("loftr-cache", isDirectory: true)

    report(progress, fraction: 0.02, message: "Preparing LoFTR matching")
    await Task.yield()

    var usedLoFTR = false
    do {
      try await buildLoFTRCache(
        request: request,
        cacheDirectory: matchCacheURL,
        progress: progress
      )
      usedLoFTR = FileManager.default.fileExists(
        atPath: matchCacheURL.appendingPathComponent("manifest.json").path
      )
    } catch {
      NSLog("LoFTR cache build failed: %@", error.localizedDescription)
      report(progress, fraction: 0.9, message: "LoFTR failed — OpenCV only")
      await Task.yield()
    }

    report(
      progress,
      fraction: 0.92,
      message: usedLoFTR ? "Stitching with LoFTR matches" : "Stitching panorama"
    )
    await Task.yield()

    let result: StitchingResult = try await withCheckedThrowingContinuation { continuation in
      SpheraNativeEngineBridge.stitch(
        manifestURL: request.manifestURL,
        outputDirectoryURL: request.outputDirectoryURL,
        matchCacheDirectoryURL: usedLoFTR ? matchCacheURL : nil
      ) { artifacts, error in
        if let error {
          continuation.resume(throwing: error)
        } else if let artifacts {
          continuation.resume(
            returning: StitchingResult(
              panoramaURL: artifacts.panoramaURL,
              reportURL: artifacts.reportURL
            )
          )
        } else {
          continuation.resume(throwing: OpenCVSpheraEngineError.missingResult)
        }
      }
    }
    report(progress, fraction: 1, message: "Panorama complete")
    return result
  }

  private struct OrderedFrame {
    let filename: String
    let ring: String
    let yaw: Double
    let image: UIImage
    let layoutRing: Int
    let localIndex: Int
    let ringSize: Int
    var phase: Double { Double(localIndex) / Double(max(ringSize, 1)) }
  }

  private func buildLoFTRCache(
    request: SpheraEngineRequest,
    cacheDirectory: URL,
    progress: StitchProgressHandler?
  ) async throws {
    let data = try Data(contentsOf: request.manifestURL)
    let object = try JSONSerialization.jsonObject(with: data)
    guard let manifest = object as? [String: Any],
      let frames = manifest["frames"] as? [[String: Any]],
      let imageDirectory = manifest["imageDirectory"] as? String
    else {
      throw OpenCVSpheraEngineError.invalidManifest
    }

    let imageRoot = request.packageDirectoryURL
      .appendingPathComponent(imageDirectory, isDirectory: true)

    struct RawFrame {
      let filename: String
      let ring: String
      let yaw: Double
      let image: UIImage
    }

    report(progress, fraction: 0.04, message: "Loading frames")
    await Task.yield()

    var raw: [RawFrame] = []
    raw.reserveCapacity(frames.count)
    for (frameIndex, frame) in frames.enumerated() {
      guard let filename = frame["imageFilename"] as? String,
        let target = frame["target"] as? [String: Any],
        let ring = target["ring"] as? String,
        let yaw = (target["yawDegrees"] as? NSNumber)?.doubleValue
          ?? target["yawDegrees"] as? Double
      else {
        throw OpenCVSpheraEngineError.invalidManifest
      }
      let url = imageRoot.appendingPathComponent(filename)
      guard let image = UIImage(contentsOfFile: url.path) else {
        throw OpenCVSpheraEngineError.imageLoadFailed(filename)
      }
      raw.append(RawFrame(filename: filename, ring: ring, yaw: yaw, image: image))
      if frameIndex % 3 == 0 || frameIndex + 1 == frames.count {
        let local = Double(frameIndex + 1) / Double(max(frames.count, 1))
        report(
          progress,
          fraction: 0.04 + 0.04 * local,
          message: "Loading frames \(frameIndex + 1)/\(frames.count)"
        )
        await Task.yield()
      }
    }

    let ringOrder = ["horizontal", "downward", "upward"]
    var ordered: [OrderedFrame] = []
    for (layoutRing, ringName) in ringOrder.enumerated() {
      let members = raw.filter { $0.ring == ringName }
        .sorted { lhs, rhs in
          if lhs.yaw != rhs.yaw { return lhs.yaw < rhs.yaw }
          return lhs.filename < rhs.filename
        }
      let size = members.count
      for (local, member) in members.enumerated() {
        ordered.append(
          OrderedFrame(
            filename: member.filename,
            ring: member.ring,
            yaw: member.yaw,
            image: member.image,
            layoutRing: layoutRing,
            localIndex: local,
            ringSize: size
          )
        )
      }
    }
    guard ordered.count == raw.count, !ordered.isEmpty else {
      throw OpenCVSpheraEngineError.invalidManifest
    }

    var pairs: [(Int, Int)] = []
    for left in 0..<ordered.count {
      for right in (left + 1)..<ordered.count {
        let a = ordered[left]
        let b = ordered[right]
        let allowed: Bool
        if a.layoutRing == b.layoutRing {
          let size = a.ringSize
          let forward = (a.localIndex - b.localIndex + size) % size
          let backward = (b.localIndex - a.localIndex + size) % size
          allowed = min(forward, backward) <= 1
        } else {
          var distance = abs(a.phase - b.phase)
          distance = min(distance, 1 - distance)
          allowed = distance <= 0.14
        }
        if allowed {
          pairs.append((left, right))
        }
      }
    }

    let backboneStart = 0.12
    let backboneEnd = 0.28
    let pairStartFraction = 0.28
    let pairEndFraction = 0.90

    report(progress, fraction: 0.09, message: "Loading LoFTR backbone")
    await Task.yield()
    let matcher = try LoFTRMatcher { [self] stage in
      self.report(progress, fraction: stage.contains("coarse") ? 0.11 : 0.09, message: stage)
    }
    report(progress, fraction: backboneStart, message: "Extracting LoFTR features")
    await Task.yield()

    var features: [MLMultiArray] = []
    features.reserveCapacity(ordered.count)
    let featureStart = CFAbsoluteTimeGetCurrent()
    for (index, frame) in ordered.enumerated() {
      NSLog("LoFTR backbone %d/%d %@", index + 1, ordered.count, frame.filename)
      features.append(try matcher.extractCoarseFeature(from: frame.image))
      let local = Double(index + 1) / Double(ordered.count)
      report(
        progress,
        fraction: backboneStart + (backboneEnd - backboneStart) * local,
        message: "Extracting features \(index + 1)/\(ordered.count)"
      )
      await Task.yield()
    }
    NSLog(
      "LoFTR backbone done in %.1fs for %d images",
      CFAbsoluteTimeGetCurrent() - featureStart,
      ordered.count
    )

    var results: [LoFTRPairMatches] = []
    results.reserveCapacity(pairs.count)
    let pairWallStart = CFAbsoluteTimeGetCurrent()
    var pairDurations: [Double] = []
    pairDurations.reserveCapacity(pairs.count)

    for (pairIndex, pair) in pairs.enumerated() {
      let matched: LoFTRPairMatches = try autoreleasepool {
        let t0 = CFAbsoluteTimeGetCurrent()
        let result = try matcher.matchPair(
          feat0: features[pair.0],
          feat1: features[pair.1],
          sourceIndex: pair.0,
          targetIndex: pair.1
        )
        let elapsed = CFAbsoluteTimeGetCurrent() - t0
        pairDurations.append(elapsed)
        NSLog(
          "LoFTR pair %d/%d: %d -> %d  matches=%d  %.2fs",
          pairIndex + 1,
          pairs.count,
          pair.0,
          pair.1,
          result.correspondences.count,
          elapsed
        )
        return result
      }
      if matched.correspondences.count >= 8 {
        results.append(matched)
      }

      let completed = pairIndex + 1
      let local = Double(completed) / Double(max(pairs.count, 1))
      let eta = Self.estimateETA(
        completed: completed,
        total: pairs.count,
        recentDurations: pairDurations
      )
      var message = "Matching pair \(completed)/\(pairs.count)"
      if let eta {
        message += " · \(Self.formatDuration(eta)) left"
      }
      report(
        progress,
        fraction: pairStartFraction + (pairEndFraction - pairStartFraction) * local,
        message: message
      )
      await Task.yield()
    }
    NSLog(
      "LoFTR matching done in %.1fs (%d/%d pairs kept)",
      CFAbsoluteTimeGetCurrent() - pairWallStart,
      results.count,
      pairs.count
    )
    try matcher.writeCache(
      pairs: results,
      imageNames: ordered.map(\.filename),
      to: cacheDirectory
    )
    report(progress, fraction: pairEndFraction, message: "LoFTR match cache ready")
  }

  private func report(
    _ progress: StitchProgressHandler?,
    fraction: Double,
    message: String
  ) {
    progress?(
      StitchProgress(
        fraction: min(1, max(0, fraction)),
        message: message
      )
    )
  }

  private static func estimateETA(
    completed: Int,
    total: Int,
    recentDurations: [Double]
  ) -> Double? {
    guard completed > 0, completed < total, !recentDurations.isEmpty else { return nil }
    let window = Array(recentDurations.suffix(min(5, recentDurations.count)))
    let average = window.reduce(0, +) / Double(window.count)
    return average * Double(total - completed)
  }

  private static func formatDuration(_ seconds: Double) -> String {
    let clamped = max(0, seconds)
    if clamped < 10 { return "<10s" }
    if clamped < 60 { return "~\(Int(clamped.rounded()))s" }
    let minutes = Int((clamped / 60.0).rounded())
    if minutes < 60 { return "~\(max(1, minutes))m" }
    let hours = minutes / 60
    let rem = minutes % 60
    return rem == 0 ? "~\(hours)h" : "~\(hours)h \(rem)m"
  }
}

enum OpenCVSpheraEngineError: LocalizedError {
  case missingResult
  case invalidManifest
  case imageLoadFailed(String)

  var errorDescription: String? {
    switch self {
    case .missingResult:
      "The native Sphera engine finished without returning a panorama."
    case .invalidManifest:
      "The capture manifest could not be parsed for LoFTR matching."
    case .imageLoadFailed(let name):
      "Could not load \(name) for LoFTR matching."
    }
  }
}
