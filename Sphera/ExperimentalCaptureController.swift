import Combine
import Foundation
import simd
import UIKit

/// A stop the user can recover from without losing the sweep.
enum ExperimentalPauseReason: Equatable, Sendable {
  case appBackgrounded
  case interrupted
  case trackingLost

  var title: String {
    switch self {
    case .appBackgrounded, .interrupted: "Sweep paused"
    case .trackingLost: "Tracking lost"
    }
  }

  var message: String {
    switch self {
    case .appBackgrounded:
      "The sweep stopped when you left the camera. The photos you already took are safe."
    case .interrupted:
      "Something interrupted the camera. The photos you already took are safe."
    case .trackingLost:
      "iPhone lost track of your surroundings. Stand where you were and point at the same spot to continue."
    }
  }
}

/// A stop the user cannot recover from in place.
struct ExperimentalCaptureFailure: Equatable, Sendable {
  var message: String
  var needsCameraPermission: Bool

  init(message: String, needsCameraPermission: Bool = false) {
    self.message = message
    self.needsCameraPermission = needsCameraPermission
  }

  init(error: Error) {
    if let tracking = error as? ARKitTrackingError, case .permissionDenied = tracking {
      self.init(message: tracking.localizedDescription, needsCameraPermission: true)
    } else {
      self.init(message: error.localizedDescription)
    }
  }
}

@MainActor
final class ExperimentalCaptureController: ObservableObject {
  /// How long the phone must sit on a row's guide line before the row starts.
  private static let alignmentHoldSeconds: TimeInterval = 0.5
  /// Confirmation beat between rows, so a finished row is visible.
  private static let rowCompleteHoldSeconds: TimeInterval = 0.9
  /// No ARKit frame for this long means the feed is gone, not just slow.
  private static let poseTimeoutSeconds: TimeInterval = 2.5
  /// Turning with no new photo for this long earns an explicit nudge.
  private static let stallNudgeSeconds: TimeInterval = 6

  let configuration: ExperimentalPanoramaConfiguration
  let arkit = ARKitTrackingService()

  @Published private(set) var guidance = ExperimentalGuidanceSnapshot.idle
  @Published private(set) var stage: ExperimentalCaptureStage = .starting
  @Published private(set) var isCapturingPhoto = false
  @Published private(set) var pauseReason: ExperimentalPauseReason?
  /// Non-fatal notice; the sweep keeps running.
  @Published private(set) var noticeMessage: String?
  @Published var savedPackage: ExperimentalCapturePackage?

  var onSaved: ((ExperimentalCapturePackage) -> Void)?
  var onFatalError: ((ExperimentalCaptureFailure) -> Void)?

  /// True when the shutter should accept a tap.
  var canStartSweep: Bool { stage == .ready && hasActivePackage }
  var isSweepInProgress: Bool { stage.isSweepInProgress }

  private enum Phase: Equatable {
    case idle
    case starting
    case ready
    case aligning(PanoramaScanLine)
    case scanning(PanoramaScanLine)
    case rowComplete(PanoramaScanLine)
    case finishing
    case paused(ExperimentalPauseReason)
    case unavailable

    var stage: ExperimentalCaptureStage {
      switch self {
      case .idle, .starting: .starting
      case .ready: .ready
      case .aligning: .aligning
      case .scanning: .scanning
      case .rowComplete: .rowComplete
      case .finishing: .finishing
      case .paused: .paused
      case .unavailable: .unavailable
      }
    }

    /// The row the sweep is on, if any.
    var line: PanoramaScanLine? {
      switch self {
      case .aligning(let line), .scanning(let line), .rowComplete(let line): line
      case .idle, .starting, .ready, .finishing, .paused, .unavailable: nil
      }
    }
  }

  private let store: ExperimentalCapturePackageStore
  private let motion: MotionTrackingService
  private var progressor: ExperimentalScanProgressor
  private let captureFeedback = UIImpactFeedbackGenerator(style: .light)

  private var phase: Phase = .idle {
    didSet {
      guard phase != oldValue else { return }
      stage = phase.stage
      ExperimentalCaptureLog.event("phase \(oldValue) -> \(phase)")
    }
  }
  /// Where the sweep was when it paused, so it can be resumed in place.
  private var phaseBeforePause: Phase?

  private var capturedFrames: [ExperimentalCapturedFrame] = []
  private var skippedTargets: [ExperimentalSkippedTarget] = []
  private var sessionStartTransform: simd_float4x4?
  private var lineOrder: [PanoramaScanLine] = []
  private var lineIndex = 0
  private var alignedSince: TimeInterval?
  private var alignmentHoldFraction: Double = 0
  private var latestUpdate: ExperimentalScanUpdate = .empty

  private var hasActivePackage = false
  private var isTabActive = false
  private var isStopping = false
  private var packageGeneration = 0
  private var lastPoseWallClock: Date?
  private var lastProgressWallClock: Date?
  private var lastDirectedOffset: Double = 0
  private var isNudgingStalledSweep = false

  private var watchdog: Task<Void, Never>?
  private var rowAdvance: Task<Void, Never>?
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

  // MARK: - Lifecycle

  func setTabActive(_ active: Bool) {
    let wasActive = isTabActive
    isTabActive = active
    guard wasActive != active else { return }
    if active {
      if arkit.isRunning {
        resumeTracking()
      }
    } else {
      if phase.stage.isSweepInProgress {
        enterPause(.appBackgrounded)
      } else {
        arkit.pause()
      }
    }
  }

  func prepareSession() async throws {
    guard !isStopping else { throw CancellationError() }
    resetSweepState()
    noticeMessage = nil
    pauseReason = nil
    savedPackage = nil
    phase = .starting
    publishGuidance(pose: nil)
    UIApplication.shared.isIdleTimerDisabled = true

    do {
      try motion.start()
    } catch {
      // Capture still works from ARKit alone; the level guide is just noisier.
      ExperimentalCaptureLog.event("core motion unavailable: \(error.localizedDescription)")
    }
    try await arkit.start(resetWorld: true)
    let pose = try await waitForFirstPose()
    try await openCapturePackage()
    phase = .ready
    publishGuidance(pose: pose)
    ExperimentalCaptureLog.event(
      "session ready horizontal=\(configuration.horizontalImageCount) upward=\(configuration.upwardImageCount) downward=\(configuration.downwardImageCount)"
    )
  }

  func stopAndAbandon() async {
    isStopping = true
    packageGeneration += 1
    cancelBackgroundWork()
    resetSweepState()
    hasActivePackage = false
    phase = .idle
    arkit.stop()
    motion.stop()
    await store.abandon()
    UIApplication.shared.isIdleTimerDisabled = false
    arkit.livePanoPreview.endLine()
    noticeMessage = nil
    pauseReason = nil
    guidance = .idle
    isStopping = false
  }

  // MARK: - User actions

  func beginSweep() {
    guard canStartSweep else { return }
    guard let pose = arkit.livePose else {
      noticeMessage = "Waiting for the camera. Try again in a moment."
      return
    }
    noticeMessage = nil
    lineOrder = configuration.scanLineOrder
    lineIndex = 0
    capturedFrames = []
    skippedTargets = []
    sessionStartTransform = pose.transform
    lastProgressWallClock = Date()
    isNudgingStalledSweep = false

    Task { [store] in
      do {
        try await store.recordSessionStart(
          transform: Matrix4x4Value(pose.transform),
          timestamp: pose.timestamp
        )
      } catch {
        // The per-frame poses are absolute, so a missing session anchor only
        // costs convenience in the exported dataset.
        ExperimentalCaptureLog.event("session start not recorded: \(error.localizedDescription)")
      }
    }
    ExperimentalCaptureLog.event(
      "sweep start timestamp=\(pose.timestamp.formatted(.number.precision(.fractionLength(3)))) tracking=\(pose.trackingState.rawValue)"
    )
    enterAlignment(for: lineOrder[0], pose: pose)
    startWatchdog()
  }

  func cancelSweep() {
    guard phase.stage.isSweepInProgress || phase.stage == .paused else { return }
    ExperimentalCaptureLog.event("sweep cancelled after \(capturedFrames.count) frames")
    discardSweepAndRearm()
  }

  /// The row a resume would land on: the interrupted one, or the next one if
  /// the interruption arrived after a row finished.
  private var resumeLine: PanoramaScanLine? {
    guard let target = phaseBeforePause else { return nil }
    if case .rowComplete = target { return lineOrder[safe: lineIndex] }
    return target.line
  }

  /// True when resuming throws away a partly shot row. Pausing stops world
  /// tracking, so the only way to keep the angles evenly spaced is to shoot the
  /// interrupted row again from the top.
  var resumeRestartsRow: Bool {
    guard case .paused = phase, case .scanning(let line)? = phaseBeforePause else { return false }
    return capturedFrames.contains { $0.scanLine == line }
  }

  /// Called from the paused overlay.
  func resumeSweep() {
    guard case .paused = phase, phaseBeforePause != nil else { return }
    guard isTabActive, UIApplication.shared.applicationState == .active else { return }
    let restartsRow = resumeRestartsRow
    let target = resumeLine
    pauseReason = nil
    phaseBeforePause = nil

    do {
      try arkit.resume()
    } catch {
      handleSessionFailure(error)
      return
    }
    lastPoseWallClock = Date()

    guard let target else {
      // Paused on the last row's confirmation: there is nothing left to shoot.
      Task { await finalizeSweep() }
      return
    }

    guard restartsRow else {
      enterAlignment(for: target, pose: arkit.livePose)
      startWatchdog()
      return
    }

    phase = .starting
    publishGuidance(pose: arkit.livePose)
    let generation = packageGeneration
    Task { [weak self] in
      guard let self else { return }
      do {
        try await self.store.discardFrames(for: target)
      } catch {
        ExperimentalCaptureLog.event("row rollback failed: \(error.localizedDescription)")
      }
      guard generation == self.packageGeneration, !self.isStopping else { return }
      self.capturedFrames.removeAll { $0.scanLine == target }
      self.skippedTargets.removeAll { $0.scanLine == target }
      self.progressor.reset()
      self.latestUpdate = .empty
      self.enterAlignment(for: target, pose: self.arkit.livePose)
      self.startWatchdog()
    }
  }

  // MARK: - App lifecycle

  func handleAppBackground() {
    if phase.stage.isSweepInProgress {
      enterPause(.appBackgrounded)
    } else {
      arkit.pause()
    }
  }

  func handleAppForeground() {
    guard isTabActive, !isStopping else { return }
    switch phase {
    case .ready:
      resumeTracking()
    case .paused:
      // The overlay drives the resume so the user knows the sweep stopped.
      break
    default:
      break
    }
  }

  // MARK: - Gallery passthrough

  func deletePackage(_ package: ExperimentalCapturePackage) async throws {
    try await store.deletePackage(package)
  }

  func listCompletedPackages() async throws -> [ExperimentalCapturePackage] {
    try await store.listCompletedPackages()
  }

  func makeShareArchive(for package: ExperimentalCapturePackage) async throws -> URL {
    try await store.makeShareArchive(for: package)
  }

  // MARK: - Pose pipeline

  private func waitForFirstPose(timeoutSeconds: Double = 8) async throws -> ExperimentalLivePose {
    let deadline = ProcessInfo.processInfo.systemUptime + timeoutSeconds
    while ProcessInfo.processInfo.systemUptime < deadline {
      if let pose = arkit.livePose {
        return pose
      }
      if !arkit.isRunning {
        throw ARKitTrackingError.sessionFailed("The camera stopped before the first frame.")
      }
      try await Task.sleep(for: .milliseconds(50))
    }
    throw ARKitTrackingError.frameUnavailable
  }

  private func handleLivePose(_ pose: ExperimentalLivePose) {
    guard isTabActive, !isStopping else { return }
    lastPoseWallClock = Date()

    switch phase {
    case .idle, .starting, .ready, .finishing, .paused, .unavailable:
      publishGuidance(pose: pose)
    case .aligning(let line):
      advanceAlignment(line: line, pose: pose)
    case .scanning(let line):
      advanceScan(line: line, pose: pose)
    case .rowComplete:
      publishGuidance(pose: pose)
    }
  }

  private func enterAlignment(for line: PanoramaScanLine, pose: ExperimentalLivePose?) {
    alignedSince = nil
    alignmentHoldFraction = 0
    latestUpdate = .empty
    progressor.endLine()
    arkit.livePanoPreview.endLine()
    phase = .aligning(line)
    publishGuidance(pose: pose)
  }

  private func advanceAlignment(line: PanoramaScanLine, pose: ExperimentalLivePose) {
    let attitude = makeAttitude(pose: pose, line: line)
    let isAligned =
      abs(attitude.pitchErrorDegrees) <= configuration.pitchToleranceDegrees
      && abs(attitude.rollDegrees) <= configuration.maxRollForCaptureDegrees
      && !pose.trackingState.isSeverelyLimited

    if isAligned {
      let since = alignedSince ?? pose.timestamp
      alignedSince = since
      let held = max(0, pose.timestamp - since)
      alignmentHoldFraction = min(1, held / Self.alignmentHoldSeconds)
      if held >= Self.alignmentHoldSeconds {
        startLine(line, pose: pose)
        return
      }
    } else {
      alignedSince = nil
      alignmentHoldFraction = 0
    }
    publishGuidance(pose: pose)
  }

  private func startLine(_ line: PanoramaScanLine, pose: ExperimentalLivePose) {
    alignedSince = nil
    alignmentHoldFraction = 1
    progressor.beginLine(line, currentYawDegrees: pose.yawDegrees)
    phase = .scanning(line)
    lastProgressWallClock = Date()
    lastDirectedOffset = 0
    isNudgingStalledSweep = false
    beginLivePanoLine(pose: pose)
    Task { [store] in
      do {
        try await store.markLineStarted(line)
      } catch {
        ExperimentalCaptureLog.event("line start not recorded: \(error.localizedDescription)")
      }
    }
    ExperimentalCaptureLog.event(
      "line start \(line.rawValue) count=\(configuration.imageCount(for: line)) step=\(configuration.yawStepDegrees(for: line).formatted(.number.precision(.fractionLength(1))))° startYaw=\(pose.yawDegrees.formatted(.number.precision(.fractionLength(1))))°"
    )
    advanceScan(line: line, pose: pose)
  }

  private func advanceScan(line: PanoramaScanLine, pose: ExperimentalLivePose) {
    let sample = makeScanSample(from: pose, line: line)
    let update = progressor.update(sample, canCapture: !isCapturingPhoto)
    latestUpdate = update
    arkit.livePanoPreview.setDirection(update.lockedDirection)

    if update.directedYawOffsetDegrees > lastDirectedOffset + 1 {
      lastDirectedOffset = update.directedYawOffsetDegrees
      if update.blockReason == nil {
        lastProgressWallClock = Date()
        isNudgingStalledSweep = false
      }
    }

    publishGuidance(pose: pose)

    if let index = update.captureIndex {
      Task { await captureKeyframe(index: index, line: line, pose: pose, update: update) }
    } else if update.isLineComplete, !isCapturingPhoto {
      completeCurrentLine(line, pose: pose)
    }
  }

  private func captureKeyframe(
    index: Int,
    line: PanoramaScanLine,
    pose: ExperimentalLivePose,
    update: ExperimentalScanUpdate
  ) async {
    guard case .scanning(let activeLine) = phase, activeLine == line else {
      progressor.noteCaptureFinished(success: false)
      return
    }
    isCapturingPhoto = true
    defer { isCapturingPhoto = false }

    do {
      let keyframe = try await arkit.captureKeyframe()
      guard case .scanning(let stillScanning) = phase, stillScanning == line else {
        progressor.noteCaptureFinished(success: false)
        return
      }
      let attitude = makeAttitude(pose: keyframe.pose, line: line)
      if attitude.shouldBlockCapture {
        // The phone drifted off the row between the request and the exposure.
        // The angle stays unresolved and will be offered again immediately.
        progressor.noteCaptureFinished(success: false)
        publishGuidance(pose: keyframe.pose)
        ExperimentalCaptureLog.event(
          "capture aborted line=\(line.rawValue) index=\(index) pitch=\(attitude.pitchDegrees.formatted(.number.precision(.fractionLength(1))))° roll=\(attitude.rollDegrees.formatted(.number.precision(.fractionLength(1))))°"
        )
        return
      }

      let start = sessionStartTransform ?? keyframe.transform
      let metadata = makeARKitMetadata(keyframe: keyframe, sessionStart: start)
      let frameID = UUID()
      let targetYaw = configuration.targetYawOffsetDegrees(for: line, index: index)
      let record = try await store.append(
        imageData: keyframe.jpegData,
        frameID: frameID,
        scanLine: line,
        indexInLine: index,
        targetYawOffsetDegrees: targetYaw,
        actualYawOffsetDegrees: update.yawOffsetDegrees,
        actualPitchDegrees: attitude.pitchDegrees,
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
      captureFeedback.impactOccurred(intensity: 0.6)
      noticeMessage = nil
      lastProgressWallClock = Date()
      isNudgingStalledSweep = false
      ExperimentalCaptureLog.event(
        "capture line=\(line.rawValue) index=\(index)/\(configuration.imageCount(for: line)) targetYaw=\(targetYaw.formatted(.number.precision(.fractionLength(1))))° actualYaw=\(update.yawOffsetDegrees.formatted(.number.precision(.fractionLength(1))))° tracking=\(keyframe.trackingState.rawValue) file=\(record.imageFilename)"
      )
      publishGuidance(pose: keyframe.pose)
      if capturedCount(for: line) + progressor.skippedIndexCount
        >= configuration.imageCount(for: line)
      {
        completeCurrentLine(line, pose: keyframe.pose)
      }
    } catch {
      progressor.noteCaptureFinished(success: false)
      noticeMessage = "A photo could not be saved. Keep turning to try again."
      ExperimentalCaptureLog.event(
        "capture failed line=\(line.rawValue) index=\(index) error=\(error.localizedDescription)"
      )
      publishGuidance(pose: pose)
    }
  }

  private func completeCurrentLine(_ line: PanoramaScanLine, pose: ExperimentalLivePose) {
    guard case .scanning = phase else { return }
    let skipped = progressor.skippedIndexList
    for index in skipped {
      skippedTargets.append(
        ExperimentalSkippedTarget(
          scanLine: line,
          indexInLine: index,
          targetYawOffsetDegrees: configuration.targetYawOffsetDegrees(for: line, index: index)
        )
      )
    }
    ExperimentalCaptureLog.event(
      "line complete \(line.rawValue) captured=\(capturedCount(for: line))/\(configuration.imageCount(for: line)) skipped=\(skipped.count)"
    )
    progressor.endLine()
    arkit.livePanoPreview.endLine()
    Task { [store] in
      do {
        try await store.markLineCompleted(line, skippedCount: skipped.count)
      } catch {
        ExperimentalCaptureLog.event("line completion not recorded: \(error.localizedDescription)")
      }
    }
    UINotificationFeedbackGenerator().notificationOccurred(.success)

    lineIndex += 1
    phase = .rowComplete(line)
    publishGuidance(pose: pose)

    let generation = packageGeneration
    rowAdvance?.cancel()
    rowAdvance = Task { [weak self] in
      try? await Task.sleep(for: .seconds(Self.rowCompleteHoldSeconds))
      guard let self, !Task.isCancelled else { return }
      guard generation == self.packageGeneration, case .rowComplete = self.phase else { return }
      if let next = self.lineOrder[safe: self.lineIndex] {
        self.enterAlignment(for: next, pose: self.arkit.livePose)
      } else {
        await self.finalizeSweep()
      }
    }
  }

  private func finalizeSweep() async {
    guard phase != .finishing else { return }
    cancelBackgroundWork()
    phase = .finishing
    publishGuidance(pose: arkit.livePose)
    arkit.livePanoPreview.endLine()
    arkit.stop()
    motion.stop()
    UIApplication.shared.isIdleTimerDisabled = false
    hasActivePackage = false
    recordUnreachedTargetsAsSkipped()

    do {
      let package = try await store.finalize(skippedTargets: skippedTargets)
      savedPackage = package
      phase = .idle
      ExperimentalCaptureLog.event(
        "session complete frames=\(package.manifest.frames.count) skipped=\(skippedTargets.count) id=\(package.manifest.sessionID.uuidString)"
      )
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      onSaved?(package)
    } catch {
      await store.abandon()
      ExperimentalCaptureLog.event("finalize failed \(error.localizedDescription)")
      failClosed(ExperimentalCaptureFailure(error: error))
    }
  }

  /// Writes down every planned angle that never got a photo, including whole
  /// rows a cut-short sweep never reached, so the saved dataset says exactly
  /// which parts of the sphere are missing. A complete sweep leaves this alone.
  private func recordUnreachedTargetsAsSkipped() {
    struct Slot: Hashable {
      let line: PanoramaScanLine
      let index: Int
    }
    let captured = Set(capturedFrames.map { Slot(line: $0.scanLine, index: $0.indexInLine) })
    var recorded = Set(skippedTargets.map { Slot(line: $0.scanLine, index: $0.indexInLine) })
    for line in configuration.scanLineOrder {
      for index in 0..<configuration.imageCount(for: line) {
        let slot = Slot(line: line, index: index)
        guard !captured.contains(slot), recorded.insert(slot).inserted else { continue }
        skippedTargets.append(
          ExperimentalSkippedTarget(
            scanLine: line,
            indexInLine: index,
            targetYawOffsetDegrees: configuration.targetYawOffsetDegrees(for: line, index: index)
          )
        )
      }
    }
  }

  // MARK: - Interruptions

  private func enterPause(_ reason: ExperimentalPauseReason) {
    guard phase.stage.isSweepInProgress else { return }
    cancelBackgroundWork()
    phaseBeforePause = phase
    pauseReason = reason
    phase = .paused(reason)
    arkit.pause()
    arkit.livePanoPreview.endLine()
    ExperimentalCaptureLog.event(
      "sweep paused (\(reason)) after \(capturedFrames.count) frames"
    )
    publishGuidance(pose: arkit.livePose)
  }

  private func handleSessionFailure(_ error: Error) {
    ExperimentalCaptureLog.event("session failure \(error.localizedDescription)")
    let failure = ExperimentalCaptureFailure(error: error)
    // The photos taken so far are already on disk. A dead session should cost
    // the rest of the sweep, not the part that worked.
    guard hasActivePackage,
      capturedFrames.count >= configuration.minimumUsableFrameCount
    else {
      failClosed(failure)
      return
    }
    ExperimentalCaptureLog.event("salvaging \(capturedFrames.count) frames after session failure")
    Task { await finalizeSweep() }
  }

  private func handleInterruption() {
    if phase.stage.isSweepInProgress {
      enterPause(.interrupted)
    }
  }

  private func handleInterruptionEnded() {
    guard isTabActive, !isStopping else { return }
    guard UIApplication.shared.applicationState == .active else { return }
    switch phase {
    case .ready, .starting:
      resumeTracking()
    default:
      break
    }
  }

  private func resumeTracking() {
    do {
      try arkit.resume()
      lastPoseWallClock = Date()
    } catch {
      handleSessionFailure(error)
    }
  }

  private func failClosed(_ failure: ExperimentalCaptureFailure) {
    packageGeneration += 1
    cancelBackgroundWork()
    resetSweepState()
    hasActivePackage = false
    phase = .unavailable
    arkit.livePanoPreview.endLine()
    arkit.stop()
    motion.stop()
    Task { [store] in await store.abandon() }
    UIApplication.shared.isIdleTimerDisabled = false
    pauseReason = nil
    noticeMessage = nil
    guidance = .idle
    onFatalError?(failure)
  }

  // MARK: - Watchdog

  private func startWatchdog() {
    watchdog?.cancel()
    watchdog = Task { [weak self] in
      while !Task.isCancelled {
        try? await Task.sleep(for: .milliseconds(500))
        guard let self, !Task.isCancelled else { return }
        self.checkForStall()
      }
    }
  }

  /// The sweep must never go quiet. Either frames are arriving and something
  /// explains the wait, or tracking is gone and the user is told.
  private func checkForStall() {
    guard phase.stage.isSweepInProgress else {
      watchdog?.cancel()
      watchdog = nil
      return
    }
    if let last = lastPoseWallClock,
      Date().timeIntervalSince(last) > Self.poseTimeoutSeconds
    {
      ExperimentalCaptureLog.event("no ARKit frames for \(Self.poseTimeoutSeconds)s")
      enterPause(.trackingLost)
      return
    }
    guard case .scanning = phase, let last = lastProgressWallClock else { return }
    let stalled = Date().timeIntervalSince(last) > Self.stallNudgeSeconds
    guard stalled != isNudgingStalledSweep else { return }
    isNudgingStalledSweep = stalled
    publishGuidance(pose: arkit.livePose)
  }

  private func cancelBackgroundWork() {
    watchdog?.cancel()
    watchdog = nil
    rowAdvance?.cancel()
    rowAdvance = nil
  }

  // MARK: - Packages

  private func openCapturePackage() async throws {
    _ = try await store.begin(
      configuration: configuration,
      coreMotionReferenceFrame: ExperimentalCaptureManifest.worldTrackingReferenceFrame
    )
    hasActivePackage = true
  }

  /// Throws away the in-progress sweep and gets back to an armed camera.
  private func discardSweepAndRearm() {
    packageGeneration += 1
    let generation = packageGeneration
    cancelBackgroundWork()
    resetSweepState()
    hasActivePackage = false
    pauseReason = nil
    noticeMessage = nil
    phase = .starting
    publishGuidance(pose: arkit.livePose)

    Task { [weak self] in
      guard let self else { return }
      await self.store.abandon()
      guard generation == self.packageGeneration, !self.isStopping else { return }
      if !self.arkit.isRunning {
        self.failClosed(
          ExperimentalCaptureFailure(message: ARKitTrackingError.notRunning.localizedDescription)
        )
        return
      }
      if self.isTabActive, UIApplication.shared.applicationState == .active {
        self.resumeTracking()
      }
      do {
        try await self.openCapturePackage()
        guard generation == self.packageGeneration, !self.isStopping else {
          self.hasActivePackage = false
          await self.store.abandon()
          return
        }
        self.phase = .ready
        self.publishGuidance(pose: self.arkit.livePose)
      } catch {
        guard generation == self.packageGeneration, !self.isStopping else { return }
        ExperimentalCaptureLog.event("package reopen failed \(error.localizedDescription)")
        self.failClosed(ExperimentalCaptureFailure(error: error))
      }
    }
  }

  private func resetSweepState() {
    progressor.reset()
    capturedFrames = []
    skippedTargets = []
    latestUpdate = .empty
    lineOrder = []
    lineIndex = 0
    alignedSince = nil
    alignmentHoldFraction = 0
    sessionStartTransform = nil
    phaseBeforePause = nil
    isCapturingPhoto = false
    lastDirectedOffset = 0
    isNudgingStalledSweep = false
    lastProgressWallClock = nil
  }

  private func beginLivePanoLine(pose: ExperimentalLivePose) {
    let fov = ExperimentalPoseMath.portraitHorizontalFOVDegrees(
      intrinsics: Matrix3x3Value(pose.intrinsics),
      imageWidth: pose.imageResolution.x,
      imageHeight: pose.imageResolution.y
    )
    let direction = configuration.captureDirection == .automatic
      ? nil
      : configuration.captureDirection
    arkit.livePanoPreview.beginLine(
      startYawDegrees: pose.yawDegrees,
      scanRangeDegrees: configuration.scanRangeDegrees,
      horizontalFOVDegrees: fov,
      direction: direction
    )
  }

  // MARK: - Sampling

  private func capturedCount(for line: PanoramaScanLine) -> Int {
    capturedFrames.reduce(into: 0) { $0 += ($1.scanLine == line ? 1 : 0) }
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

  private func makeScanSample(
    from pose: ExperimentalLivePose,
    line: PanoramaScanLine
  ) -> ExperimentalScanSample {
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

  // MARK: - Guidance

  private func publishGuidance(pose: ExperimentalLivePose?) {
    let line = phase.line ?? lineOrder[safe: lineIndex] ?? .horizontal
    let attitude = pose.map { makeAttitude(pose: $0, line: line) }
    let tracking = pose?.trackingState ?? .notAvailable
    // The row's own tallies survive into the confirmation beat, but a row that
    // has not started yet shows an empty, correctly sized track.
    let update: ExperimentalScanUpdate
    switch phase {
    case .scanning, .rowComplete:
      update = latestUpdate
    case .aligning:
      update = ExperimentalScanUpdate.pending(count: configuration.imageCount(for: line))
    default:
      update = .empty
    }

    let pitchError = attitude?.pitchErrorDegrees ?? 0
    let roll = attitude?.rollDegrees ?? 0
    let isLevel =
      attitude.map { !$0.shouldBlockCapture } ?? false

    let blockReason: ExperimentalCaptureBlockReason?
    switch phase {
    case .scanning:
      blockReason = update.blockReason
    case .aligning:
      blockReason = alignmentBlockReason(attitude: attitude, tracking: tracking, line: line)
    default:
      blockReason = nil
    }

    let capturedInLine = capturedCount(for: line)
    let copy = guidanceCopy(
      line: line,
      blockReason: blockReason,
      update: update,
      tracking: tracking
    )

    let snapshot = ExperimentalGuidanceSnapshot(
      stage: phase.stage,
      line: line,
      // Numbered from the row on screen, so the pill never disagrees with the
      // instruction underneath it during the between-rows beat.
      passIndex: (configuration.scanLineOrder.firstIndex(of: line) ?? lineIndex) + 1,
      passCount: configuration.scanLineOrder.count,
      capturedInLine: capturedInLine,
      skippedInLine: update.skippedCount,
      targetInLine: configuration.imageCount(for: line),
      capturedTotal: capturedFrames.count,
      targetTotal: configuration.totalImageCount,
      targetStates: update.targetStates,
      sweepFraction: rounded(update.sweepFraction, places: 3),
      directedYawOffsetDegrees: rounded(update.directedYawOffsetDegrees, places: 1),
      scanRangeDegrees: configuration.scanRangeDegrees,
      pitchErrorDegrees: rounded(pitchError, places: 1),
      rollDegrees: rounded(roll, places: 1),
      pitchGuideScaleDegrees: configuration.pitchGuideScaleDegrees,
      isLevelForCapture: isLevel,
      alignmentHoldFraction: rounded(alignmentHoldFraction, places: 2),
      blockReason: blockReason,
      trackingState: tracking,
      rotationDirection: update.lockedDirection,
      title: copy.title,
      subtitle: copy.subtitle
    )
    guard snapshot != guidance else { return }
    guidance = snapshot
  }

  private func alignmentBlockReason(
    attitude: ExperimentalAttitudeReading?,
    tracking: ARKitTrackingStateRecord,
    line: PanoramaScanLine
  ) -> ExperimentalCaptureBlockReason? {
    guard let attitude else { return .trackingLimited }
    if tracking.isSeverelyLimited { return .trackingLimited }
    if attitude.isRollBlockingCapture { return .rolled }
    // While getting into position, the row's own tilt is the instruction, not
    // an error, so only flag it once the phone is close.
    if abs(attitude.pitchErrorDegrees) > configuration.pitchToleranceDegrees,
      abs(attitude.pitchErrorDegrees) <= configuration.pitchGuideScaleDegrees
    {
      return attitude.pitchErrorDegrees > 0 ? .pitchTooHigh : .pitchTooLow
    }
    _ = line
    return nil
  }

  private func guidanceCopy(
    line: PanoramaScanLine,
    blockReason: ExperimentalCaptureBlockReason?,
    update: ExperimentalScanUpdate,
    tracking: ARKitTrackingStateRecord
  ) -> (title: String, subtitle: String?) {
    switch phase {
    case .idle, .starting:
      return ("Starting camera", "Hold iPhone still for a moment.")
    case .ready:
      if let advice = tracking.userAdvice {
        return ("Getting ready", advice)
      }
      return ("Ready to sweep", "Tap the shutter, then turn slowly all the way around.")
    case .aligning:
      if let blockReason, blockReason == .trackingLimited {
        return (blockReason.instruction, tracking.userAdvice)
      }
      return (
        line.alignmentInstruction,
        lineIndex == 0
          ? "Then turn slowly all the way around."
          : "Keep the same spot on the floor."
      )
    case .scanning:
      if let blockReason {
        if blockReason == .reversedDirection, let direction = update.lockedDirection {
          return (direction.continuedInstruction, "You turned back for a moment.")
        }
        return (blockReason.instruction, nil)
      }
      if isNudgingStalledSweep {
        return (
          update.lockedDirection?.continuedInstruction ?? "Keep turning",
          "The next photo is a little further around."
        )
      }
      guard let direction = update.lockedDirection else {
        return ("Turn either way", "Start turning to begin the row.")
      }
      return (
        update.sweepFraction > 0.02 ? direction.continuedInstruction : direction.rotationInstruction,
        nil
      )
    case .rowComplete:
      let next = lineOrder[safe: lineIndex]
      return (
        "\(line.rowName) done",
        next.map { "Next: \($0.rowName.lowercased())" } ?? "Saving your sweep"
      )
    case .finishing:
      return ("Saving your sweep", "\(capturedFrames.count) photos")
    case .paused(let reason):
      return (reason.title, reason.message)
    case .unavailable:
      return ("Sweep unavailable", nil)
    }
  }

  private func rounded(_ value: Double, places: Int) -> Double {
    guard value.isFinite else { return 0 }
    let scale = pow(10, Double(places))
    return (value * scale).rounded() / scale
  }
}

private extension Array {
  subscript(safe index: Int) -> Element? {
    guard indices.contains(index) else { return nil }
    return self[index]
  }
}
