import CoreML
import Foundation
import UIKit

/// Device-side regression for the Float16 packedTokens crash and dual-softmax path.
enum LoFTRSmokeTest {
  static func run() async {
    let started = CFAbsoluteTimeGetCurrent()
    do {
      NSLog("LoFTR smoke: begin")
      let matcher = try LoFTRMatcher { stage in
        NSLog("LoFTR smoke stage: %@", stage)
      }

      let img0 = try makePatternImage(seed: 1)
      let img1 = try makePatternImage(seed: 2)
      NSLog("LoFTR smoke: extracting features")
      let f0 = try matcher.extractCoarseFeature(from: img0)
      let f1 = try matcher.extractCoarseFeature(from: img1)
      NSLog(
        "LoFTR smoke: features shape0=%@ dtype0=%d shape1=%@ dtype1=%d",
        f0.shape.map(\.intValue).description,
        f0.dataType.rawValue,
        f1.shape.map(\.intValue).description,
        f1.dataType.rawValue
      )

      // Exercise the exact crash site (coarse → packedTokens) repeatedly.
      for i in 1...8 {
        let t0 = CFAbsoluteTimeGetCurrent()
        let matched = try matcher.matchPair(
          feat0: f0,
          feat1: f1,
          sourceIndex: 0,
          targetIndex: 1
        )
        NSLog(
          "LoFTR smoke: pair %d/%d matches=%d %.2fs",
          i,
          8,
          matched.correspondences.count,
          CFAbsoluteTimeGetCurrent() - t0
        )
        await Task.yield()
      }

      NSLog(
        "LoFTR smoke: PASSED in %.1fs",
        CFAbsoluteTimeGetCurrent() - started
      )
    } catch {
      NSLog("LoFTR smoke: FAILED %@", error.localizedDescription)
    }
  }

  private static func makePatternImage(seed: UInt64) throws -> UIImage {
    let width = 480
    let height = 640
    var pixels = [UInt8](repeating: 0, count: width * height * 4)
    var state = seed &* 6364136223846793005 &+ 1
    for i in 0..<(width * height) {
      state = state &* 6364136223846793005 &+ 1
      let v = UInt8((state >> 33) & 0xFF)
      let o = i * 4
      // Structured noise so coarse features aren't degenerate.
      let x = i % width
      let y = i / width
      let band = UInt8((x / 16 + y / 16 + Int(seed)) % 255)
      pixels[o] = v &+ band
      pixels[o + 1] = v &- band
      pixels[o + 2] = band
      pixels[o + 3] = 255
    }
    let colorSpace = CGColorSpaceCreateDeviceRGB()
    guard
      let context = CGContext(
        data: &pixels,
        width: width,
        height: height,
        bitsPerComponent: 8,
        bytesPerRow: width * 4,
        space: colorSpace,
        bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
      ),
      let cgImage = context.makeImage()
    else {
      throw LoFTRMatcherError.imageConversionFailed
    }
    return UIImage(cgImage: cgImage)
  }
}
