@preconcurrency import ARKit
import AVFoundation
import Combine
import CoreImage
import CoreVideo
import ImageIO
import simd
import UIKit

enum ARKitTrackingError: LocalizedError {
  case unsupported
  case permissionDenied
  case sessionFailed(String)
  case frameUnavailable
  case imageEncodingFailed
  case notRunning

  var errorDescription: String? {
    switch self {
    case .unsupported:
      "ARKit world tracking is not available on this device."
    case .permissionDenied:
      "Camera access is required. Enable it in Settings and reopen Sphera."
    case .sessionFailed(let message):
      "ARKit session failed: \(message)"
    case .frameUnavailable:
      "No ARKit camera frame is available."
    case .imageEncodingFailed:
      "The ARKit camera frame could not be saved as a JPEG."
    case .notRunning:
      "The ARKit session is not running."
    }
  }
}

@MainActor
final class ARKitTrackingService: NSObject, ObservableObject {
  nonisolated let session = ARSession()

  @Published private(set) var isRunning = false
  @Published private(set) var isFeedActive = false
  @Published private(set) var livePose: ExperimentalLivePose?
  @Published private(set) var trackingState: ARKitTrackingStateRecord = .notAvailable

  var onSessionFailed: ((Error) -> Void)?
  var onInterruptionBegan: (() -> Void)?
  var onInterruptionEnded: (() -> Void)?

  nonisolated private let sessionDelegateQueue = DispatchQueue(
    label: "com.sphera.arkit.delegate",
    qos: .userInitiated
  )
  nonisolated private let jpegEncoder = ARKitImageEncoder()
  nonisolated private let poseBox = LatestPoseBox()
  nonisolated private let previewBufferBox = LatestPixelBufferBox()
  private var configuration: ARWorldTrackingConfiguration?
  private var lastLoggedTrackingState: ARKitTrackingStateRecord?
  private var runGeneration = 0
  weak var previewView: ARKitPreviewContainerView?

  override init() {
    super.init()
    session.delegate = self
    session.delegateQueue = sessionDelegateQueue
  }

  static var isWorldTrackingSupported: Bool {
    ARWorldTrackingConfiguration.isSupported
  }

  func attachPreview(_ view: ARKitPreviewContainerView) {
    previewView = view
    view.trackingService = self
    session.delegate = self
    session.delegateQueue = sessionDelegateQueue
  }

  func detachPreview(_ view: ARKitPreviewContainerView) {
    if previewView === view {
      previewView = nil
    }
  }

  func start(resetWorld: Bool) async throws {
    guard Self.isWorldTrackingSupported else {
      throw ARKitTrackingError.unsupported
    }
    guard await requestCameraAuthorization() else {
      throw ARKitTrackingError.permissionDenied
    }

    runGeneration += 1
    let generation = runGeneration
    let config = makeConfiguration()
    configuration = config
    livePose = nil
    isFeedActive = false
    trackingState = .notAvailable
    lastLoggedTrackingState = nil
    poseBox.reset()
    previewBufferBox.reset()

    guard generation == runGeneration else { throw CancellationError() }
    var options: ARSession.RunOptions = []
    if resetWorld {
      options.insert(.resetTracking)
      options.insert(.removeExistingAnchors)
    }
    session.run(config, options: options)
    guard generation == runGeneration else {
      session.pause()
      throw CancellationError()
    }
    isRunning = true
  }

  func pause() {
    guard isRunning else { return }
    session.pause()
    isFeedActive = false
  }

  func resume() throws {
    guard let configuration else {
      throw ARKitTrackingError.notRunning
    }
    session.run(configuration, options: [])
    isRunning = true
  }

  func stop() {
    runGeneration += 1
    session.pause()
    isRunning = false
    isFeedActive = false
    livePose = nil
    trackingState = .notAvailable
  }

  func currentCameraTransform() -> simd_float4x4? {
    livePose?.transform ?? session.currentFrame?.camera.transform
  }

  func captureKeyframe() async throws -> ARKitCapturedKeyframe {
    guard isRunning else { throw ARKitTrackingError.notRunning }
    do {
      return try await captureHighResolutionKeyframe()
    } catch {
      ExperimentalCaptureLog.event(
        "high-res capture unavailable (\(error.localizedDescription)); using current ARFrame"
      )
      return try await captureCurrentKeyframe()
    }
  }

  private func captureHighResolutionKeyframe() async throws -> ARKitCapturedKeyframe {
    try await withCheckedThrowingContinuation { continuation in
      let resumeOnce = OnceFlag()
      let encoder = jpegEncoder
      session.captureHighResolutionFrame { frame, error in
        guard resumeOnce.mark() else { return }
        guard let frame else {
          continuation.resume(throwing: error ?? ARKitTrackingError.frameUnavailable)
          return
        }
        Self.encodeKeyframe(frame: frame, encoder: encoder, continuation: continuation)
      }
    }
  }

  private func captureCurrentKeyframe() async throws -> ARKitCapturedKeyframe {
    guard let frame = session.currentFrame else {
      throw ARKitTrackingError.frameUnavailable
    }
    let encoder = jpegEncoder
    return try await withCheckedThrowingContinuation { continuation in
      sessionDelegateQueue.async {
        Self.encodeKeyframe(frame: frame, encoder: encoder, continuation: continuation)
      }
    }
  }

  nonisolated private static func encodeKeyframe(
    frame: ARFrame,
    encoder: ARKitImageEncoder,
    continuation: CheckedContinuation<ARKitCapturedKeyframe, Error>
  ) {
    let pose = makeLivePose(from: frame)
    do {
      let jpeg = try encoder.jpegData(from: frame.capturedImage)
      continuation.resume(
        returning: ARKitCapturedKeyframe(
          jpegData: jpeg,
          pose: pose,
          transform: frame.camera.transform,
          intrinsics: frame.camera.intrinsics,
          imageResolution: frame.camera.imageResolution,
          timestamp: frame.timestamp,
          trackingState: pose.trackingState,
          exifOrientation: ExperimentalCaptureImage.jpegEXIFOrientation
        )
      )
    } catch {
      continuation.resume(throwing: error)
    }
  }

  private func makeConfiguration() -> ARWorldTrackingConfiguration {
    let config = ARWorldTrackingConfiguration()
    config.worldAlignment = .gravity
    config.planeDetection = []
    config.environmentTexturing = .none
    config.isAutoFocusEnabled = true
    return config
  }

  private func requestCameraAuthorization() async -> Bool {
    switch AVCaptureDevice.authorizationStatus(for: .video) {
    case .authorized:
      true
    case .notDetermined:
      await withCheckedContinuation { continuation in
        AVCaptureDevice.requestAccess(for: .video) { granted in
          continuation.resume(returning: granted)
        }
      }
    case .denied, .restricted:
      false
    @unknown default:
      false
    }
  }

  private func flushFrame() {
    if let buffer = previewBufferBox.take() {
      previewView?.display(buffer)
      isFeedActive = true
    }
    guard let pose = poseBox.take() else { return }
    livePose = pose
    trackingState = pose.trackingState
    if lastLoggedTrackingState != pose.trackingState {
      lastLoggedTrackingState = pose.trackingState
      ExperimentalCaptureLog.event("trackingState=\(pose.trackingState.rawValue)")
    }
  }

  nonisolated static func makeLivePose(from frame: ARFrame) -> ExperimentalLivePose {
    let transform = frame.camera.transform
    let euler = ExperimentalPoseMath.eulerDegrees(cameraToWorld: transform)
    return ExperimentalLivePose(
      timestamp: frame.timestamp,
      trackingState: mapTrackingState(frame.camera.trackingState),
      transform: transform,
      intrinsics: frame.camera.intrinsics,
      imageResolution: SIMD2(
        Int(frame.camera.imageResolution.width.rounded()),
        Int(frame.camera.imageResolution.height.rounded())
      ),
      yawDegrees: euler.x,
      pitchDegrees: euler.y,
      rollDegrees: ExperimentalPoseMath.screenPlaneRollDegrees(
        cameraToWorld: transform,
        portraitRotationClockwise: true
      ),
      position: ExperimentalPoseMath.position(cameraToWorld: transform),
      rotationRateMagnitude: 0
    )
  }

  nonisolated static func mapTrackingState(
    _ state: ARCamera.TrackingState
  ) -> ARKitTrackingStateRecord {
    switch state {
    case .normal:
      .normal
    case .notAvailable:
      .notAvailable
    case .limited(let reason):
      switch reason {
      case .initializing: .limitedInitializing
      case .excessiveMotion: .limitedExcessiveMotion
      case .insufficientFeatures: .limitedInsufficientFeatures
      case .relocalizing: .limitedRelocalizing
      @unknown default: .limitedOther
      }
    }
  }
}

extension ARKitTrackingService: ARSessionDelegate {
  nonisolated func session(_ session: ARSession, didUpdate frame: ARFrame) {
    previewBufferBox.store(frame.capturedImage)
    if poseBox.store(Self.makeLivePose(from: frame)) {
      Task { @MainActor [weak self] in
        self?.flushFrame()
      }
    }
  }

  nonisolated func session(_ session: ARSession, didFailWithError error: Error) {
    Task { @MainActor [weak self] in
      guard let self else { return }
      self.isRunning = false
      self.isFeedActive = false
      self.onSessionFailed?(error)
    }
  }

  nonisolated func sessionWasInterrupted(_ session: ARSession) {
    Task { @MainActor [weak self] in
      self?.isFeedActive = false
      self?.onInterruptionBegan?()
    }
  }

  nonisolated func sessionInterruptionEnded(_ session: ARSession) {
    Task { @MainActor [weak self] in
      self?.onInterruptionEnded?()
    }
  }
}

struct ARKitCapturedKeyframe: Sendable {
  let jpegData: Data
  let pose: ExperimentalLivePose
  let transform: simd_float4x4
  let intrinsics: simd_float3x3
  let imageResolution: CGSize
  let timestamp: TimeInterval
  let trackingState: ARKitTrackingStateRecord
  let exifOrientation: Int
}

private final class LatestPoseBox: @unchecked Sendable {
  private let lock = NSLock()
  private var pose: ExperimentalLivePose?
  private var hopScheduled = false

  func store(_ pose: ExperimentalLivePose) -> Bool {
    lock.lock()
    defer { lock.unlock() }
    self.pose = pose
    if hopScheduled { return false }
    hopScheduled = true
    return true
  }

  func take() -> ExperimentalLivePose? {
    lock.lock()
    defer { lock.unlock() }
    hopScheduled = false
    return pose
  }

  func reset() {
    lock.lock()
    pose = nil
    hopScheduled = false
    lock.unlock()
  }
}

private final class LatestPixelBufferBox: @unchecked Sendable {
  private let lock = NSLock()
  private var buffer: CVPixelBuffer?

  func store(_ buffer: CVPixelBuffer) {
    lock.lock()
    self.buffer = buffer
    lock.unlock()
  }

  func take() -> CVPixelBuffer? {
    lock.lock()
    defer { lock.unlock() }
    let buffer = self.buffer
    self.buffer = nil
    return buffer
  }

  func reset() {
    lock.lock()
    buffer = nil
    lock.unlock()
  }
}

private final class OnceFlag: @unchecked Sendable {
  private let lock = NSLock()
  private var didRun = false

  func mark() -> Bool {
    lock.lock()
    defer { lock.unlock() }
    if didRun { return false }
    didRun = true
    return true
  }
}

final class ARKitImageEncoder: @unchecked Sendable {
  private let context = CIContext(options: [.cacheIntermediates: false])

  func jpegData(from pixelBuffer: CVPixelBuffer, quality: CGFloat = 0.9) throws -> Data {
    let image = CIImage(cvPixelBuffer: pixelBuffer)
    let colorSpace = image.colorSpace ?? CGColorSpaceCreateDeviceRGB()
    let options: [CIImageRepresentationOption: Any] = [
      kCGImageDestinationLossyCompressionQuality as CIImageRepresentationOption: quality
    ]
    guard
      let data = context.jpegRepresentation(
        of: image,
        colorSpace: colorSpace,
        options: options
      )
    else {
      throw ARKitTrackingError.imageEncodingFailed
    }
    return try Self.tagJPEG(
      data,
      orientation: ExperimentalCaptureImage.jpegEXIFOrientation
    )
  }

  private static func tagJPEG(_ jpeg: Data, orientation: Int) throws -> Data {
    guard
      let source = CGImageSourceCreateWithData(jpeg as CFData, nil),
      let uti = CGImageSourceGetType(source)
    else {
      throw ARKitTrackingError.imageEncodingFailed
    }
    let output = NSMutableData()
    guard let destination = CGImageDestinationCreateWithData(output, uti, 1, nil) else {
      throw ARKitTrackingError.imageEncodingFailed
    }
    CGImageDestinationAddImageFromSource(
      destination,
      source,
      0,
      [kCGImagePropertyOrientation: orientation] as CFDictionary
    )
    guard CGImageDestinationFinalize(destination) else {
      throw ARKitTrackingError.imageEncodingFailed
    }
    return output as Data
  }
}

enum ExperimentalCaptureLog {
  static func event(_ message: String) {
    print("SPHERA_ARKIT \(message)")
  }
}
