import Combine
import Foundation
import simd
import UIKit

@MainActor
final class ExperimentalCaptureController: ObservableObject {
  let configuration: ExperimentalPanoramaConfiguration
  let arkit = ARKitTrackingService()

  @Published private(set) var guidance = ExperimentalGuidanceSnapshot.idle
  @Published private(set) var capturedFrames: [ExperimentalCapturedFrame] = []
  @Published private(set) var isARKitReady = false
  @Published private(set) var hasActivePackage = false
  @Published private(set) var isCapturingPhoto = false
  @Published private(set) var isSweeping = false
  @Published private(set) var statusMessage = "Experimental ARKit capture"
  @Published private(set) var errorMessage: String?
  @Published var savedPackage: ExperimentalCapturePackage?

  var onSaved: ((ExperimentalCapturePackage) -> Void)?
  var onFatalError: ((String) -> Void)?

  private let store: ExperimentalCapturePackageStore
  private let motion: MotionTrackingService
  private var progressor: ExperimentalScanProgressor
  private var sessionStartTransform: simd_float4x4?
  private var lineOrder: [PanoramaScanLine] = []
  private var lineIndex = 0
  private var isTransitioning = false
  private var pitchAlignedSince: TimeInterval?
  private var isTabActive = false
  private var isStopping = false
  private var packageGeneration = 0
  private var subscriptions = Set<AnyCancellable>()

  init(
    motion: MotionTrackingService,
    configuration: ExperimentalPanoramaConfiguration = .default,
    store: ExperimentalCapturePackageStore = ExperimentalCapturePackageStore()
  ) {
    self.motion = motion
    self.configuration = configuration
    self.store = store
    self.progressor = ExperimentalScanProgressor(configuration: configuration)
    arkit.$livePose
      .compactMap { $0 }
      .sink { [weak self] pose in
        self?.handleLivePose(pose)
      }
      .store(in: &subscriptions)

    arkit.onSessionFailed = { [weak self] error in
      self?.handleSessionFailure(error)
    }
    arkit.onInterruptionBegan = { [weak self] in
      self?.handleInterruption()
    }
    arkit.onInterruptionEnded = { [weak self] in
      self?.handleInterruptionEnded()
    }
  }

  func setTabActive(_ active: Bool) {
    let wasActive = isTabActive
    isTabActive = active
    guard wasActive != active else { return }
    if active {
      if arkit.isRunning {
        try? arkit.resume()
      }
    } else {
      arkit.pause()
    }
  }

  func prepareSession() async throws {
    guard !isStopping else { throw CancellationError() }
    errorMessage = nil
    savedPackage = nil
    capturedFrames = []
    isSweeping = false
    isCapturingPhoto = false
    isTransitioning = false
    isARKitReady = false
    hasActivePackage = false
    sessionStartTransform = nil
    progressor.reset()
    guidance = ExperimentalGuidanceSnapshot.idle
    statusMessage = "Starting ARKit world tracking"
    UIApplication.shared.isIdleTimerDisabled = true

    try? motion.start()
    try await arkit.start(resetWorld: true)
    _ = try await waitForFirstPose()
    try await openCapturePackage()
    isARKitReady = true
    statusMessage = "Tap the shutter, then rotate"
    ExperimentalCaptureLog.event(
      "session ready horizontal=\(configuration.horizontalImageCount) upward=\(configuration.upwardImageCount) downward=\(configuration.downwardImageCount)"
    )
  }

  func beginSweep() {
    guard isARKitReady, hasActivePackage, !isSweeping, !isCapturingPhoto, let pose = arkit.livePose else { return }
    errorMessage = nil
    lineOrder = configuration.scanLineOrder
    lineIndex = 0
    isSweeping = true
    isTransitioning = true
    pitchAlignedSince = nil
    sessionStartTransform = pose.transform
    Task {
      do {
        try await store.recordSessionStart(
          transform: Matrix4x4Value(pose.transform),
          timestamp: pose.timestamp
        )
      } catch {
        errorMessage = error.localizedDescription
      }
    }
    ExperimentalCaptureLog.event(
      "sweep start timestamp=\(pose.timestamp.formatted(.number.precision(.fractionLength(3)))) tracking=\(pose.trackingState.rawValue)"
    )
    statusMessage = "Hold the phone, then rotate"
    updateGuidance(
      pose: pose,
      scanUpdate: nil,
      instructionOverride: nil,
      transitioningLine: .horizontal
    )
  }

  func cancelSweep() {
    guard isSweeping || capturedFrames.isEmpty == false else { return }
    ExperimentalCaptureLog.event("sweep cancelled after \(capturedFrames.count) frames")
    finishIncomplete(reason: "User cancelled the experimental capture.")
  }

  func stopAndAbandon() async {
    isStopping = true
    packageGeneration += 1
    isSweeping = false
    isCapturingPhoto = false
    isARKitReady = false
    hasActivePackage = false
    progressor.reset()
    capturedFrames = []
    sessionStartTransform = nil
    arkit.stop()
    motion.stop()
    await store.abandon()
    UIApplication.shared.isIdleTimerDisabled = false
    guidance = ExperimentalGuidanceSnapshot.idle
    statusMessage = "Experimental ARKit capture"
    isStopping = false
  }

  func handleAppBackground() {
    if isSweeping {
      ExperimentalCaptureLog.event("app backgrounded during experimental capture")
      handleSessionFailure(ARKitTrackingError.sessionFailed("Sphera left the foreground."))
    } else if isARKitReady {
      arkit.pause()
    }
  }

  func handleAppForeground() {
    guard isARKitReady, isTabActive, !isStopping else { return }
    try? arkit.resume()
  }

  func deletePackage(_ package: ExperimentalCapturePackage) async throws {
    try await store.deletePackage(package)
  }

  func listCompletedPackages() async throws -> [ExperimentalCapturePackage] {
    try await store.listCompletedPackages()
  }

  func makeShareArchive(for package: ExperimentalCapturePackage) async throws -> URL {
    try await store.makeShareArchive(for: package)
  }

  private func waitForFirstPose(timeoutSeconds: Double = 8) async throws -> ExperimentalLivePose {
    let deadline = ProcessInfo.processInfo.systemUptime + timeoutSeconds
    while ProcessInfo.processInfo.systemUptime < deadline {
      if let pose = arkit.livePose {
        return pose
      }
      if !arkit.isRunning {
        throw ARKitTrackingError.sessionFailed("ARKit session ended before the first frame.")
      }
      try await Task.sleep(for: .milliseconds(50))
    }
    throw ARKitTrackingError.frameUnavailable
  }

  private func handleLivePose(_ pose: ExperimentalLivePose) {
    guard isARKitReady, isTabActive, !isStopping else { return }
    if !isSweeping {
      updateGuidance(pose: pose, scanUpdate: nil, instructionOverride: nil)
      return
    }
    if isTransitioning {
      handleTransition(pose: pose)
      return
    }
    guard progressor.activeLine != nil else { return }

    let sample = makeScanSample(from: pose)
    let update = progressor.update(sample)
    updateGuidance(pose: pose, scanUpdate: update, instructionOverride: nil)

    if let index = update.captureIndex, !isCapturingPhoto {
      Task { await captureKeyframe(index: index, pose: pose, update: update) }
    } else if update.isLineComplete, !isCapturingPhoto {
      completeCurrentLine(pose: pose)
    }
  }

  private func handleTransition(pose: ExperimentalLivePose) {
    guard lineIndex < lineOrder.count else {
      Task { await finalizeSweep() }
      return
    }
    let line = lineOrder[lineIndex]
    let attitude = makeAttitude(pose: pose, line: line)
    let pitchAligned = abs(attitude.pitchErrorDegrees) <= configuration.pitchToleranceDegrees
    let rollAligned = abs(attitude.rollDegrees) <= configuration.maxRollForCaptureDegrees
    let aligned = pitchAligned && rollAligned
    if aligned {
      if pitchAlignedSince == nil {
        pitchAlignedSince = pose.timestamp
      }
      let held = pose.timestamp - (pitchAlignedSince ?? pose.timestamp)
      if held >= 0.2 {
        startLine(line, pose: pose)
        return
      }
    } else {
      pitchAlignedSince = nil
    }

    updateGuidance(
      pose: pose,
      scanUpdate: nil,
      instructionOverride: nil,
      transitioningLine: line
    )
  }

  private func startLine(_ line: PanoramaScanLine, pose: ExperimentalLivePose) {
    isTransitioning = false
    pitchAlignedSince = nil
    progressor.beginLine(line, currentYawDegrees: pose.yawDegrees)
    Task { try? await store.markLineStarted(line) }
    let count = configuration.imageCount(for: line)
    let step = configuration.yawStepDegrees(for: line)
    ExperimentalCaptureLog.event(
      "line start \(line.rawValue) count=\(count) step=\(step.formatted(.number.precision(.fractionLength(1))))° startYaw=\(pose.yawDegrees.formatted(.number.precision(.fractionLength(1))))°"
    )
    statusMessage = "\(line.displayName) · keep the arrow on the line"
    let sample = makeScanSample(from: pose)
    let update = progressor.update(sample)
    updateGuidance(pose: pose, scanUpdate: update, instructionOverride: nil)
    if let index = update.captureIndex {
      Task { await captureKeyframe(index: index, pose: pose, update: update) }
    }
  }

  private func completeCurrentLine(pose: ExperimentalLivePose) {
    guard let line = progressor.activeLine else { return }
    let count = capturedFrames.filter { $0.scanLine == line }.count
    ExperimentalCaptureLog.event(
      "line complete \(line.rawValue) captured=\(count)/\(configuration.imageCount(for: line))"
    )
    progressor.endLine()
    Task { try? await store.markLineCompleted(line) }
    lineIndex += 1
    if lineIndex >= lineOrder.count {
      Task { await finalizeSweep() }
      return
    }
    isTransitioning = true
    pitchAlignedSince = nil
    let next = lineOrder[lineIndex]
    statusMessage = "\(line.displayName) complete · \(next.expectedOrientationLabel.lowercased())"
    updateGuidance(
      pose: pose,
      scanUpdate: nil,
      instructionOverride: next.expectedOrientationLabel,
      transitioningLine: next,
      lineJustCompleted: true
    )
  }

  private func captureKeyframe(
    index: Int,
    pose: ExperimentalLivePose,
    update: ExperimentalScanUpdate
  ) async {
    guard isSweeping, let line = progressor.activeLine ?? lineOrder[safe: lineIndex] else {
      progressor.noteCaptureFinished(success: false)
      return
    }
    let precheck = makeAttitude(pose: pose, line: line)
    if precheck.shouldBlockCapture {
      progressor.noteCaptureFinished(success: false)
      updateGuidance(pose: pose, scanUpdate: update, instructionOverride: precheck.blockReason)
      return
    }
    isCapturingPhoto = true
    statusMessage = "Capturing \(line.displayName.lowercased()) \(index + 1)"
    do {
      let keyframe = try await arkit.captureKeyframe()
      let capturedAttitude = makeAttitude(pose: keyframe.pose, line: line)
      if capturedAttitude.shouldBlockCapture {
        progressor.noteCaptureFinished(success: false)
        isCapturingPhoto = false
        statusMessage = capturedAttitude.blockReason ?? "Hold the phone upright on the line"
        updateGuidance(
          pose: keyframe.pose,
          scanUpdate: update,
          instructionOverride: capturedAttitude.blockReason
        )
        ExperimentalCaptureLog.event(
          "capture aborted line=\(line.rawValue) index=\(index) pitch=\(capturedAttitude.pitchDegrees.formatted(.number.precision(.fractionLength(1))))° roll=\(capturedAttitude.rollDegrees.formatted(.number.precision(.fractionLength(1))))° reason=\(capturedAttitude.blockReason ?? "attitude")"
        )
        return
      }
      let start = sessionStartTransform ?? keyframe.transform
      let metadata = makeARKitMetadata(keyframe: keyframe, sessionStart: start)
      let frameID = UUID()
      let targetYaw = configuration.targetYawOffsetDegrees(for: line, index: index)
      let actualYaw = update.yawOffsetDegrees
      let translation = hypot(
        hypot(metadata.translationFromSessionStart.x, metadata.translationFromSessionStart.y),
        metadata.translationFromSessionStart.z
      )
      let record = try await store.append(
        imageData: keyframe.jpegData,
        frameID: frameID,
        scanLine: line,
        indexInLine: index,
        targetYawOffsetDegrees: targetYaw,
        actualYawOffsetDegrees: actualYaw,
        actualPitchDegrees: capturedAttitude.pitchDegrees,
        arkit: metadata,
        motion: motion.sampleStore.latest,
        photo: PhotoMetadata(
          codec: "jpeg",
          width: Int(keyframe.imageResolution.width.rounded()),
          height: Int(keyframe.imageResolution.height.rounded()),
          exifOrientation: keyframe.exifOrientation,
          exposureDurationSeconds: nil,
          iso: nil,
          aperture: nil,
          focalLengthMillimeters: nil,
          focalLength35mmEquivalent: nil,
          brightnessValue: nil,
          exposureBiasValue: nil,
          lensMake: nil,
          lensModel: nil
        ),
        qualityNotes: update.qualityNotes
      )
      capturedFrames.append(record)
      progressor.noteCaptureFinished(success: true)
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      ExperimentalCaptureLog.event(
        "capture line=\(line.rawValue) index=\(index)/\(configuration.imageCount(for: line)) targetYaw=\(targetYaw.formatted(.number.precision(.fractionLength(1))))° actualYaw=\(actualYaw.formatted(.number.precision(.fractionLength(1))))° pitch=\(capturedAttitude.pitchDegrees.formatted(.number.precision(.fractionLength(1))))° roll=\(capturedAttitude.rollDegrees.formatted(.number.precision(.fractionLength(1))))° tracking=\(keyframe.trackingState.rawValue) t=\(keyframe.timestamp.formatted(.number.precision(.fractionLength(3)))) translation=\(translation.formatted(.number.precision(.fractionLength(3))))m file=\(record.imageFilename) id=\(frameID.uuidString)"
      )
      isCapturingPhoto = false
      if capturedFrames.filter({ $0.scanLine == line }).count
        >= configuration.imageCount(for: line)
      {
        completeCurrentLine(pose: keyframe.pose)
      }
    } catch {
      progressor.noteCaptureFinished(success: false)
      isCapturingPhoto = false
      errorMessage = error.localizedDescription
      statusMessage = "Capture failed; keep rotating to retry"
      ExperimentalCaptureLog.event(
        "capture failed line=\(line.rawValue) index=\(index) error=\(error.localizedDescription)"
      )
    }
  }

  private func finalizeSweep() async {
    guard isSweeping else { return }
    isSweeping = false
    isCapturingPhoto = false
    statusMessage = "Saving experimental capture"
    arkit.stop()
    motion.stop()
    UIApplication.shared.isIdleTimerDisabled = false
    isARKitReady = false
    hasActivePackage = false
    do {
      let package = try await store.finalize()
      savedPackage = package
      ExperimentalCaptureLog.event(
        "session complete frames=\(package.manifest.frames.count) id=\(package.manifest.sessionID.uuidString)"
      )
      onSaved?(package)
    } catch {
      errorMessage = error.localizedDescription
      statusMessage = "Could not save experimental capture"
      await store.abandon()
      ExperimentalCaptureLog.event("finalize failed \(error.localizedDescription)")
      onFatalError?(error.localizedDescription)
    }
  }

  private func finishIncomplete(reason: String) {
    isSweeping = false
    isCapturingPhoto = false
    progressor.reset()
    capturedFrames = []
    sessionStartTransform = nil
    isTransitioning = false
    hasActivePackage = false
    statusMessage = reason
    errorMessage = nil
    if isARKitReady, let pose = arkit.livePose {
      updateGuidance(
        pose: pose,
        scanUpdate: nil,
        instructionOverride: "Tap the shutter, then rotate"
      )
    } else {
      guidance = ExperimentalGuidanceSnapshot.idle
    }
    let generation = packageGeneration
    Task {
      await store.abandon()
      guard generation == packageGeneration, !isStopping, isARKitReady else { return }
      do {
        try await openCapturePackage()
        guard generation == packageGeneration, !isStopping, isARKitReady else {
          hasActivePackage = false
          await store.abandon()
          return
        }
        statusMessage = "Tap the shutter, then rotate"
        if let pose = arkit.livePose {
          updateGuidance(
            pose: pose,
            scanUpdate: nil,
            instructionOverride: "Tap the shutter, then rotate"
          )
        }
      } catch {
        guard generation == packageGeneration, !isStopping else { return }
        failClosedAfterPackageError(error)
      }
    }
  }

  private func openCapturePackage() async throws {
    _ = try await store.begin(
      configuration: configuration,
      coreMotionReferenceFrame: ExperimentalCaptureManifest.worldTrackingReferenceFrame
    )
    hasActivePackage = true
  }

  private func failClosedAfterPackageError(_ error: Error) {
    packageGeneration += 1
    ExperimentalCaptureLog.event("package reopen failed \(error.localizedDescription)")
    hasActivePackage = false
    isSweeping = false
    isCapturingPhoto = false
    isARKitReady = false
    progressor.reset()
    capturedFrames = []
    sessionStartTransform = nil
    arkit.stop()
    motion.stop()
    UIApplication.shared.isIdleTimerDisabled = false
    guidance = ExperimentalGuidanceSnapshot.idle
    errorMessage = error.localizedDescription
    statusMessage = "Experimental capture unavailable"
    Task { await store.abandon() }
    onFatalError?(error.localizedDescription)
  }

  private func handleSessionFailure(_ error: Error) {
    ExperimentalCaptureLog.event("session failure \(error.localizedDescription)")
    let shouldNotify = isARKitReady || isSweeping
    packageGeneration += 1
    isSweeping = false
    isCapturingPhoto = false
    isARKitReady = false
    hasActivePackage = false
    progressor.reset()
    capturedFrames = []
    sessionStartTransform = nil
    arkit.stop()
    motion.stop()
    Task { await store.abandon() }
    UIApplication.shared.isIdleTimerDisabled = false
    errorMessage = error.localizedDescription
    statusMessage = "Experimental capture unavailable"
    if shouldNotify {
      onFatalError?(error.localizedDescription)
    }
  }

  private func handleInterruption() {
    if isSweeping {
      handleSessionFailure(ARKitTrackingError.sessionFailed("ARKit was interrupted."))
      return
    }
    guard isARKitReady else { return }
    statusMessage = "Tracking paused"
  }

  private func handleInterruptionEnded() {
    guard isARKitReady, isTabActive, !isStopping else { return }
    guard UIApplication.shared.applicationState == .active else { return }
    do {
      try arkit.resume()
      if !isSweeping {
        statusMessage = "Tap the shutter, then rotate"
      }
      ExperimentalCaptureLog.event("session interruption ended; tracking resumed")
    } catch {
      handleSessionFailure(error)
    }
  }

  private func makeAttitude(
    pose: ExperimentalLivePose,
    line: PanoramaScanLine
  ) -> ExperimentalAttitudeReading {
    ExperimentalAttitudeReading.make(
      gravity: motion.isRunning ? motion.sampleStore.latest?.gravity : nil,
      arkitPitchDegrees: pose.pitchDegrees,
      arkitRollDegrees: pose.rollDegrees,
      targetPitchDegrees: configuration.targetPitchDegrees(for: line),
      configuration: configuration
    )
  }

  private func makeScanSample(from pose: ExperimentalLivePose) -> ExperimentalScanSample {
    let motionSample = motion.isRunning ? motion.sampleStore.latest : nil
    let rate: Double
    if let motionSample {
      let r = motionSample.rotationRateRadiansPerSecond
      rate = sqrt(r.x * r.x + r.y * r.y + r.z * r.z)
    } else {
      rate = 0
    }
    let start = sessionStartTransform ?? pose.transform
    let delta = pose.position - ExperimentalPoseMath.position(cameraToWorld: start)
    let quality: ExperimentalTrackingQuality
    if pose.trackingState.isSeverelyLimited {
      quality = .severelyLimited
    } else if pose.trackingState == .normal {
      quality = .normal
    } else {
      quality = .limited
    }
    let line = progressor.activeLine ?? lineOrder[safe: lineIndex] ?? .horizontal
    let attitude = makeAttitude(pose: pose, line: line)
    return ExperimentalScanSample(
      yawDegrees: pose.yawDegrees,
      pitchDegrees: attitude.pitchDegrees,
      rollDegrees: attitude.rollDegrees,
      timestamp: pose.timestamp,
      trackingQuality: quality,
      rotationRateMagnitude: rate,
      translationMeters: Double(simd_length(delta))
    )
  }

  private func makeARKitMetadata(
    keyframe: ARKitCapturedKeyframe,
    sessionStart: simd_float4x4
  ) -> ARKitCameraMetadata {
    let relative = ExperimentalPoseMath.relativeTransform(
      from: sessionStart,
      to: keyframe.transform
    )
    let startOrientation = ExperimentalPoseMath.orientation(cameraToWorld: sessionStart)
    let currentOrientation = ExperimentalPoseMath.orientation(cameraToWorld: keyframe.transform)
    let relativeRotation = simd_mul(startOrientation.inverse, currentOrientation)
    let translation = keyframe.pose.position
      - ExperimentalPoseMath.position(cameraToWorld: sessionStart)
    return ARKitCameraMetadata(
      timestamp: keyframe.timestamp,
      trackingState: keyframe.trackingState,
      transform: Matrix4x4Value(keyframe.transform),
      intrinsics: Matrix3x3Value(keyframe.intrinsics),
      imageResolutionWidth: Int(keyframe.imageResolution.width.rounded()),
      imageResolutionHeight: Int(keyframe.imageResolution.height.rounded()),
      position: Vector3Value(keyframe.pose.position),
      orientation: QuaternionValue(currentOrientation),
      translationFromSessionStart: Vector3Value(translation),
      rotationFromSessionStart: QuaternionValue(relativeRotation),
      relativeTransform: Matrix4x4Value(relative),
      eulerDegrees: ExperimentalPoseMath.eulerDegrees(cameraToWorld: keyframe.transform)
    )
  }

  private func updateGuidance(
    pose: ExperimentalLivePose,
    scanUpdate: ExperimentalScanUpdate?,
    instructionOverride: String?,
    transitioningLine: PanoramaScanLine? = nil,
    lineJustCompleted: Bool = false
  ) {
    let activeLine = transitioningLine ?? progressor.activeLine
    let nextLine: PanoramaScanLine?
    if isTransitioning, let transitioningLine {
      nextLine = transitioningLine
    } else if let activeLine, let current = lineOrder.firstIndex(of: activeLine) {
      nextLine = lineOrder[safe: current + 1]
    } else {
      nextLine = lineOrder.first
    }

    let line = activeLine ?? .horizontal
    let targetInLine = configuration.imageCount(for: line)
    let capturedInLine = capturedFrames.filter { $0.scanLine == line }.count
    let attitude = makeAttitude(pose: pose, line: line)
    let pitchError = scanUpdate?.pitchErrorDegrees ?? attitude.pitchErrorDegrees
    let roll = scanUpdate?.rollDegrees ?? attitude.rollDegrees
    let isRolled = scanUpdate?.isRolled ?? attitude.isRolled
    let isPitchBlocking =
      scanUpdate?.qualityNotes.contains("off-path-pitch") == true
      || attitude.isPitchBlockingCapture
    let isRollBlocking =
      scanUpdate?.qualityNotes.contains("off-upright-roll") == true
      || attitude.isRollBlockingCapture

    var warning: String?
    if isRollBlocking {
      warning = "Level the phone"
    } else if isPitchBlocking {
      warning = attitude.pitchErrorDegrees > 0 ? "Lower the phone" : "Raise the phone"
    } else if pose.trackingState.isSeverelyLimited {
      warning = "Hold still"
    } else if scanUpdate?.isWrongDirection == true {
      warning = "Turn the other way"
    } else if scanUpdate?.isTranslatingTooMuch == true {
      warning = "Rotate in place"
    } else if scanUpdate?.qualityNotes.contains("excessive-rotation-rate") == true {
      warning = "Slow down"
    }

    let instruction: String
    if isRollBlocking {
      instruction = "Level the phone"
    } else if isPitchBlocking {
      instruction = attitude.pitchErrorDegrees > 0 ? "Lower the phone" : "Raise the phone"
    } else if pose.trackingState.isSeverelyLimited {
      instruction = "Hold still"
    } else if scanUpdate?.isWrongDirection == true {
      instruction = "Turn the other way"
    } else if scanUpdate?.isTranslatingTooMuch == true {
      instruction = "Rotate in place"
    } else if scanUpdate?.qualityNotes.contains("excessive-rotation-rate") == true {
      instruction = "Slow down"
    } else if isTransitioning {
      instruction =
        configuration.isPitchErrorBlockingCapture(attitude.pitchErrorDegrees)
        ? (attitude.pitchErrorDegrees > 0 ? "Lower the phone" : "Raise the phone")
        : line.expectedOrientationLabel
    } else if let instructionOverride {
      instruction = instructionOverride
    } else if lineJustCompleted || scanUpdate?.isLineComplete == true {
      instruction = "\(line.displayName) complete"
    } else if isSweeping {
      instruction = "Photo \(capturedInLine) of \(targetInLine)"
    } else {
      instruction = "Tap the shutter, then rotate"
    }

    guidance = ExperimentalGuidanceSnapshot(
      activeLine: isSweeping ? line : nil,
      nextLine: nextLine,
      isTransitioning: isTransitioning && isSweeping,
      isReadyToStart: !isSweeping && isARKitReady && hasActivePackage,
      lineProgress: scanUpdate?.progress ?? 0,
      capturedInLine: capturedInLine,
      targetInLine: targetInLine,
      capturedTotal: capturedFrames.count,
      targetTotal: configuration.totalImageCount,
      pitchErrorDegrees: pitchError,
      rollDegrees: roll,
      currentYawOffsetDegrees: scanUpdate?.yawOffsetDegrees ?? 0,
      isAbovePath: scanUpdate?.isAbovePath ?? attitude.isAbovePath,
      isBelowPath: scanUpdate?.isBelowPath ?? attitude.isBelowPath,
      isRolled: isRolled,
      isPitchBlockingCapture: isPitchBlocking,
      isRollBlockingCapture: isRollBlocking,
      isWrongDirection: scanUpdate?.isWrongDirection ?? false,
      isTranslatingTooMuch: scanUpdate?.isTranslatingTooMuch ?? false,
      isLineComplete: lineJustCompleted || (scanUpdate?.isLineComplete ?? false),
      trackingState: pose.trackingState,
      expectedOrientationLabel: line.expectedOrientationLabel,
      instruction: instruction,
      warningMessage: warning,
      rotationDirection: scanUpdate?.lockedDirection,
      rollCorrectionInstruction: attitude.rollCorrectionInstruction,
      pitchGuideScaleDegrees: configuration.pitchGuideScaleDegrees
    )
  }
}

private extension Array {
  subscript(safe index: Int) -> Element? {
    guard indices.contains(index) else { return nil }
    return self[index]
  }
}
