import Combine
import CoreImage
import CoreVideo
import UIKit

/// Live slit-scan strip: the first frame is drawn in full, then the leading
/// vertical edge of each ARKit frame is stamped by yaw, not by time.
final class ExperimentalLivePanoPreview: ObservableObject, @unchecked Sendable {
  @Published private(set) var image: UIImage?
  @Published private(set) var horizontalFOVDegrees: Double = 60

  fileprivate let engine = ExperimentalLivePanoEngine()

  func beginLine(
    startYawDegrees: Double,
    scanRangeDegrees: Double,
    horizontalFOVDegrees: Double,
    direction: ExperimentalCaptureDirection?
  ) {
    self.horizontalFOVDegrees = horizontalFOVDegrees
    engine.beginLine(
      startYawDegrees: startYawDegrees,
      scanRangeDegrees: scanRangeDegrees,
      horizontalFOVDegrees: horizontalFOVDegrees,
      direction: direction
    )
    publish(nil)
  }

  func setDirection(_ direction: ExperimentalCaptureDirection?) {
    engine.setDirection(direction)
  }

  func endLine() {
    engine.endLine()
    publish(nil)
  }

  func ingest(pixelBuffer: CVPixelBuffer, yawDegrees: Double) {
    engine.ingest(pixelBuffer: pixelBuffer, yawDegrees: yawDegrees) { [weak self] image in
      self?.publish(image)
    }
  }

  private func publish(_ image: UIImage?) {
    if Thread.isMainThread {
      self.image = image
    } else {
      DispatchQueue.main.async { self.image = image }
    }
  }
}

/// Serial slit-scan worker. Safe to call from the ARKit delegate queue.
final class ExperimentalLivePanoEngine: @unchecked Sendable {
  private let queue = DispatchQueue(label: "com.sphera.livepano", qos: .userInteractive)
  private let ciContext = CIContext(options: [.cacheIntermediates: false])

  private var isActive = false
  private var pendingSeed = false
  private var startYaw = 0.0
  private var lastRawYaw: Double?
  private var unwrappedYaw: Double?
  private var scanRange = 360.0
  private var fov = 60.0
  private var direction: ExperimentalCaptureDirection?
  private var lastWrittenColumn = -1
  private var canvas: CGContext?
  private let canvasWidth = 1024
  private let canvasHeight = 256
  private let sourceSlitWidth: CGFloat = 6

  func beginLine(
    startYawDegrees: Double,
    scanRangeDegrees: Double,
    horizontalFOVDegrees: Double,
    direction: ExperimentalCaptureDirection?
  ) {
    queue.sync {
      startYaw = startYawDegrees
      lastRawYaw = startYawDegrees
      unwrappedYaw = startYawDegrees
      scanRange = max(scanRangeDegrees, 1)
      fov = max(horizontalFOVDegrees, 1)
      self.direction = direction
      isActive = true
      pendingSeed = true
      lastWrittenColumn = -1
      canvas = makeCanvas()
    }
  }

  func setDirection(_ direction: ExperimentalCaptureDirection?) {
    queue.async { self.direction = direction }
  }

  func endLine() {
    queue.sync {
      isActive = false
      pendingSeed = false
      lastRawYaw = nil
      unwrappedYaw = nil
      lastWrittenColumn = -1
      canvas = nil
    }
  }

  func ingest(
    pixelBuffer: CVPixelBuffer,
    yawDegrees: Double,
    onImage: @escaping @Sendable (UIImage) -> Void
  ) {
    queue.async { [self] in
      guard isActive, let canvas else { return }
      let portrait = CIImage(cvPixelBuffer: pixelBuffer).oriented(.right)
      if pendingSeed {
        drawFullStartFrame(portrait, in: canvas)
        lastWrittenColumn = max(startWidth - 1, 0)
        pendingSeed = false
        if let image = snapshot(from: canvas) {
          onImage(image)
        }
      }

      let directed = ExperimentalPoseMath.directedYawOffsetDegrees(
        signedOffset: unwrapOffset(yawDegrees),
        direction: direction
      )
      let target = Int(
        ExperimentalPoseMath.panoramaSlitColumn(
          directedYawDegrees: directed,
          horizontalFOVDegrees: fov,
          scanRangeDegrees: scanRange,
          canvasWidth: Double(canvasWidth)
        ).rounded(.down)
      )
      let clampedTarget = min(max(target, 0), canvasWidth - 1)
      guard clampedTarget > lastWrittenColumn else { return }

      let fromRight = direction != .counterclockwise
      guard let slit = leadingSlit(from: portrait, fromRight: fromRight) else { return }
      let from = lastWrittenColumn + 1
      drawSlit(slit, in: canvas, fromColumn: from, toColumn: clampedTarget)
      lastWrittenColumn = clampedTarget
      if let image = snapshot(from: canvas) {
        onImage(image)
      }
    }
  }

  private func unwrapOffset(_ rawYaw: Double) -> Double {
    if let lastRawYaw, let unwrappedYaw {
      let delta = ExperimentalPoseMath.shortestDeltaDegrees(from: lastRawYaw, to: rawYaw)
      let updated = unwrappedYaw + delta
      self.unwrappedYaw = updated
      self.lastRawYaw = rawYaw
      return updated - startYaw
    }
    lastRawYaw = rawYaw
    unwrappedYaw = rawYaw
    return rawYaw - startYaw
  }

  private var startWidth: Int {
    Int(
      (fov / scanRange * Double(canvasWidth)).rounded(.down)
    )
  }

  private func makeCanvas() -> CGContext? {
    let colorSpace = CGColorSpaceCreateDeviceRGB()
    guard
      let context = CGContext(
        data: nil,
        width: canvasWidth,
        height: canvasHeight,
        bitsPerComponent: 8,
        bytesPerRow: 0,
        space: colorSpace,
        bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue
          | CGBitmapInfo.byteOrder32Little.rawValue
      )
    else { return nil }
    context.translateBy(x: 0, y: CGFloat(canvasHeight))
    context.scaleBy(x: 1, y: -1)
    context.interpolationQuality = .medium
    context.setFillColor(UIColor.clear.cgColor)
    context.fill(CGRect(x: 0, y: 0, width: canvasWidth, height: canvasHeight))
    return context
  }

  private func drawFullStartFrame(_ portrait: CIImage, in canvas: CGContext) {
    let width = max(startWidth, 1)
    let dest = CGRect(x: 0, y: 0, width: width, height: canvasHeight)
    guard let cgImage = ciContext.createCGImage(portrait, from: portrait.extent) else { return }
    let extent = portrait.extent
    guard extent.width > 1 else { return }
    let scale = dest.width / extent.width
    let drawnHeight = extent.height * scale
    let drawRect = CGRect(
      x: dest.minX,
      y: dest.midY - drawnHeight / 2,
      width: dest.width,
      height: drawnHeight
    )
    canvas.saveGState()
    canvas.clip(to: dest)
    canvas.draw(cgImage, in: drawRect)
    canvas.restoreGState()
  }

  private func leadingSlit(from portrait: CIImage, fromRight: Bool) -> CGImage? {
    let extent = portrait.extent
    guard extent.width > 1, extent.height > 1 else { return nil }
    let slitWidth = min(sourceSlitWidth, extent.width)
    let rect = CGRect(
      x: fromRight ? extent.maxX - slitWidth : extent.minX,
      y: extent.minY,
      width: slitWidth,
      height: extent.height
    )
    return ciContext.createCGImage(portrait.cropped(to: rect), from: rect)
  }

  private func drawSlit(_ slit: CGImage, in canvas: CGContext, fromColumn: Int, toColumn: Int) {
    let width = max(toColumn - fromColumn + 1, 1)
    let dest = CGRect(x: fromColumn, y: 0, width: width, height: canvasHeight)
    canvas.draw(slit, in: dest)
  }

  private func snapshot(from canvas: CGContext) -> UIImage? {
    guard let cgImage = canvas.makeImage() else { return nil }
    return UIImage(cgImage: cgImage, scale: 1, orientation: .up)
  }
}
