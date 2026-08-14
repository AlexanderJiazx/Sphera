import Combine
import Foundation
import UIKit

struct CaptureCompletion: Equatable, Sendable {
  let package: CapturePackage
  let stitchingResult: StitchingResult?
  let stitchingMessage: String?
}

enum CaptureWorkflowPhase: Equatable {
  case setup
  case preparing
  case awaitingPrimary
  case capturingPoints
  case saved(CapturePackage)
  case stitching
  case completed(CaptureCompletion)
  case failed(String)
}

@MainActor
final class CaptureViewModel: ObservableObject {
  @Published var configuration = CaptureConfiguration.debugPreset
  @Published private(set) var phase: CaptureWorkflowPhase = .setup
  @Published private(set) var plan = CapturePlan(configuration: .debugPreset)
  @Published private(set) var capturedFrames: [CapturedFrameRecord] = []
  @Published private(set) var currentMotionSample: MotionSample?
  @Published private(set) var navigationReading = CaptureNavigationReading.unavailable
  @Published private(set) var guidePoints: [CapturePointProjection] = []
  @Published private(set) var activeTarget: CaptureTarget?
  @Published private(set) var stableHoldProgress = 0.0
  @Published private(set) var isCapturingPhoto = false
  @Published private(set) var statusMessage = "Ready"
  @Published private(set) var stitchProgress: Double?
  @Published private(set) var captureErrorMessage: String?
  @Published private(set) var galleryPackages: [CapturePackage] = []
  @Published private(set) var galleryErrorMessage: String?
  @Published private(set) var isRefreshingGallery = false

  let motion: MotionTrackingService
  let camera: CameraCaptureService

  private let packageStore: CapturePackageStore
  /// When set (tests), compute always uses this stitcher and ignores the toggle.
  private let stitcherOverride: (any PanoramaStitching)?
  private static let experimentalMetalStitchKey = "useExperimentalMetalStitch"
  /// Settings toggle, off by default. Does not change the stable OpenCV path
  /// unless the user turns this on.
  @Published var useExperimentalMetalStitch: Bool {
    didSet {
      UserDefaults.standard.set(
        useExperimentalMetalStitch,
        forKey: Self.experimentalMetalStitchKey
      )
    }
  }
  private var subscriptions = Set<AnyCancellable>()
  private var captureReference: CaptureReferenceFrame?
  private var alignmentHoldTracker = AlignmentHoldTracker()
  private var autoCaptureBlockedUntil = 0.0
  private var lastNavigationTraceTimestamp = -Double.infinity
  private let navigationTraceEnabled =
    ProcessInfo.processInfo.arguments.contains("--navigation-trace")
  private var isCaptureTabActive = false

  init(
    packageStore: CapturePackageStore = CapturePackageStore(),
    stitcher: (any PanoramaStitching)? = nil
  ) {
    let motion = MotionTrackingService()
    self.motion = motion
    camera = CameraCaptureService(motionStore: motion.sampleStore)
    self.packageStore = packageStore
    self.stitcherOverride = stitcher
    useExperimentalMetalStitch = UserDefaults.standard.bool(
      forKey: Self.experimentalMetalStitchKey
    )

    motion.$currentSample
      .sink { [weak self] sample in
        self?.handleMotionSample(sample)
      }
      .store(in: &subscriptions)
  }

  /// Default remains the stable OpenCV engine. The experimental Metal engine
  /// is selected only when the Settings toggle is on and no test override is set.
  private func activeStitcher() -> any PanoramaStitching {
    if let stitcherOverride {
      return stitcherOverride
    }
    if useExperimentalMetalStitch {
      return SpheraEngineAdapter(nativeEngine: ExperimentalSpheraEngine())
    }
    return SpheraEngineAdapter(nativeEngine: OpenCVSpheraEngine())
  }

  var totalFrameCount: Int { plan.targets.count }

  var remainingTargets: [CaptureTarget] {
    let capturedIDs = Set(capturedFrames.map(\.target.id))
    return plan.targets.filter { !capturedIDs.contains($0.id) }
  }

  var progressFraction: Double {
    guard totalFrameCount > 0 else { return 0 }
    return Double(capturedFrames.count) / Double(totalFrameCount)
  }

  func rebuildPlan() {
    configuration.horizontalCount = min(16, max(4, configuration.horizontalCount))
    configuration.downwardCount = min(6, max(4, configuration.downwardCount))
    configuration.upwardCount = min(6, max(4, configuration.upwardCount))
    switch phase {
    case .setup, .saved, .completed, .failed:
      plan = CapturePlan(configuration: configuration)
    default:
      break
    }
  }

  func startCapture() {
    guard phase == .setup || phase.isTerminal else { return }
    rebuildPlan()
    phase = .preparing
    statusMessage = "Starting camera and motion"
    captureErrorMessage = nil
    capturedFrames = []
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    autoCaptureBlockedUntil = 0
    captureReference = nil

    Task {
      do {
        try motion.start()
        try await startCameraIfCaptureTabActive(restoreAuto: true)
        _ = try await motion.waitForFirstSample()
        plan = CapturePlan(configuration: configuration)
        _ = try await packageStore.begin(
          plan: plan,
          coreMotionReferenceFrame: motion.referenceFrameName
        )
        phase = .awaitingPrimary
        statusMessage = "Frame the first shot, then capture"
        stopCameraIfCaptureTabInactive()
      } catch {
        camera.stop()
        motion.stop()
        await packageStore.abandon()
        phase = .failed(error.localizedDescription)
        statusMessage = "Capture unavailable"
      }
    }
  }

  func capturePrimary() {
    guard phase == .awaitingPrimary, !isCapturingPhoto else { return }
    isCapturingPhoto = true
    captureErrorMessage = nil
    statusMessage = "Capturing primary"
    Task {
      await performPrimaryCapture()
    }
  }

  /// Abandons the in-progress session and returns to setup so the Capture tab
  /// can start a fresh session.
  func resetCapture() {
    guard phase == .capturingPoints else { return }
    motion.stop()
    isCapturingPhoto = false
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    statusMessage = "Resetting capture"
    Task {
      await camera.unlockExposureFocusWhiteBalance()
      camera.stop()
      await packageStore.abandon()
      phase = .setup
      statusMessage = "Ready"
      plan = CapturePlan(configuration: configuration)
    }
  }

  func stopCapture() {
    camera.stop()
    motion.stop()
    isCapturingPhoto = false
    Task {
      await packageStore.abandon()
    }
    phase = .setup
    statusMessage = "Ready"
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    plan = CapturePlan(configuration: configuration)
  }

  func prepareNewCapture() {
    stopCapture()
  }

  func returnToSetup() {
    camera.stop()
    motion.stop()
    isCapturingPhoto = false
    phase = .setup
    statusMessage = "Ready"
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    plan = CapturePlan(configuration: configuration)
  }

  func refreshGallery() async {
    isRefreshingGallery = true
    defer { isRefreshingGallery = false }
    do {
      galleryPackages = try await packageStore.listCompletedPackages()
      galleryErrorMessage = nil
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func computeOnDevice(package: CapturePackage, replaceExisting: Bool = false) async {
    phase = .stitching
    stitchProgress = nil
    let usingExperimental =
      stitcherOverride == nil && useExperimentalMetalStitch
    statusMessage = replaceExisting
      ? (usingExperimental
        ? "Recomputing with experimental Metal stitch"
        : "Recomputing panorama on device")
      : (usingExperimental
        ? "Starting experimental Metal stitch"
        : "Starting native stitch")
    camera.stop()
    motion.stop()

    do {
      if replaceExisting {
        try await packageStore.clearEngineOutput(for: package)
      }
      let stitcher = activeStitcher()
      let result = try await stitcher.stitch(package: package) { [weak self] update in
        Task { @MainActor in
          guard let self else { return }
          // Live pipeline stage text; fraction is kept only for diagnostics.
          self.stitchProgress = update.fraction
          self.statusMessage = update.message
        }
      }
      stitchProgress = nil
      phase = .completed(
        CaptureCompletion(
          package: package,
          stitchingResult: result,
          stitchingMessage: nil
        )
      )
      statusMessage = "Panorama complete"
      await refreshGallery()
    } catch {
      stitchProgress = nil
      phase = .completed(
        CaptureCompletion(
          package: package,
          stitchingResult: nil,
          stitchingMessage: error.localizedDescription
        )
      )
      statusMessage = replaceExisting
        ? "Panorama recompute failed"
        : "On-device compute failed"
      galleryErrorMessage = error.localizedDescription
    }
  }

  func deleteFromGallery(_ package: CapturePackage) async {
    do {
      try await packageStore.deletePackage(package)
      await refreshGallery()
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func importEnginePanorama(
    into package: CapturePackage,
    panoramaURL: URL,
    reportURL: URL? = nil
  ) async {
    do {
      let accessing = panoramaURL.startAccessingSecurityScopedResource()
      defer {
        if accessing {
          panoramaURL.stopAccessingSecurityScopedResource()
        }
      }
      var reportAccessing = false
      if let reportURL {
        reportAccessing = reportURL.startAccessingSecurityScopedResource()
      }
      defer {
        if reportAccessing, let reportURL {
          reportURL.stopAccessingSecurityScopedResource()
        }
      }
      try await packageStore.importEnginePanorama(
        into: package,
        panoramaURL: panoramaURL,
        reportURL: reportURL
      )
      galleryErrorMessage = nil
      await refreshGallery()
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func makeShareArchive(for package: CapturePackage) async throws -> URL {
    try await packageStore.makeShareArchive(for: package)
  }

  func reportGalleryError(_ message: String) {
    galleryErrorMessage = message
  }

  func clearGalleryError() {
    galleryErrorMessage = nil
  }

  /// Starts or stops the camera when the Capture tab becomes visible or hidden.
  /// Motion keeps running so the capture-reference frame stays valid.
  func setCaptureTabActive(_ active: Bool) {
    let wasActive = isCaptureTabActive
    isCaptureTabActive = active
    guard wasActive != active else { return }

    if active {
      switch phase {
      case .setup:
        startCapture()
      case .awaitingPrimary, .capturingPoints:
        Task { await resumeCameraIfCaptureTabActive() }
      default:
        break
      }
    } else {
      pauseCameraForHiddenCaptureTab()
    }
  }

  private func startCameraIfCaptureTabActive(restoreAuto: Bool) async throws {
    guard isCaptureTabActive else {
      camera.stop()
      return
    }
    try await camera.start(restoreAuto: restoreAuto)
    guard isCaptureTabActive else {
      camera.stop()
      return
    }
    switch camera.state {
    case .running, .configuring:
      break
    default:
      try await camera.start(restoreAuto: restoreAuto)
      if !isCaptureTabActive {
        camera.stop()
      }
    }
  }

  private func resumeCameraIfCaptureTabActive() async {
    guard isCaptureTabActive else { return }
    do {
      try await startCameraIfCaptureTabActive(restoreAuto: false)
    } catch {
      captureErrorMessage = error.localizedDescription
    }
  }

  private func stopCameraIfCaptureTabInactive() {
    if !isCaptureTabActive {
      camera.stop()
    }
  }

  private func pauseCameraForHiddenCaptureTab() {
    switch phase {
    case .preparing, .awaitingPrimary, .capturingPoints:
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      if !isCapturingPhoto {
        camera.stop()
      }
    default:
      break
    }
  }

  private func handleMotionSample(_ sample: MotionSample?) {
    currentMotionSample = sample
    guard phase == .capturingPoints,
      isCaptureTabActive,
      !isCapturingPhoto,
      let sample,
      let captureReference
    else {
      if phase != .capturingPoints {
        guidePoints = []
        activeTarget = nil
        navigationReading = .unavailable
      }
      return
    }

    let remaining = remainingTargets
    let projections = remaining.map { target in
      OrientationMath.projectTargetToScreen(
        sample: sample,
        captureReference: captureReference,
        target: target
      )
    }

    guard
      let nearest = OrientationMath.nearestTarget(
        among: remaining,
        sample: sample,
        captureReference: captureReference,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
    else {
      guidePoints = projections
      activeTarget = nil
      navigationReading = .unavailable
      stableHoldProgress = 0
      alignmentHoldTracker.reset()
      return
    }

    let reading = nearest.reading
    navigationReading = reading
    activeTarget = nearest.target
    guidePoints = projections.map { point in
      var updated = point
      if point.targetID == nearest.target.id {
        updated = CapturePointProjection(
          targetID: point.targetID,
          ring: point.ring,
          offsetX: point.offsetX,
          offsetY: point.offsetY,
          directionErrorDegrees: point.directionErrorDegrees,
          isInFront: point.isInFront,
          isAligned: reading.isAligned
        )
      }
      return updated
    }

    let holdUpdate = alignmentHoldTracker.update(
      isAligned: reading.isAligned,
      timestamp: sample.monotonicTimestampSeconds,
      requiredDuration: configuration.stableHoldDurationSeconds,
      blockedUntilTimestamp: autoCaptureBlockedUntil
    )
    stableHoldProgress = holdUpdate.progress
    traceNavigationIfEnabled(
      sample: sample,
      target: nearest.target,
      reading: reading
    )

    if reading.isAligned {
      statusMessage =
        stableHoldProgress < 1
        ? "Hold on point"
        : "Capturing"
    } else {
      statusMessage =
        "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
    }

    guard reading.isAligned, holdUpdate.shouldCapture else { return }
    isCapturingPhoto = true
    Task {
      await capture(target: nearest.target, reference: captureReference)
    }
  }

  private func performPrimaryCapture() async {
    do {
      let photo = try await camera.capturePhoto()
      let reference = OrientationMath.makeCaptureReference(from: photo.motionSample)
      captureReference = reference
      traceCaptureReferenceIfEnabled()

      let pitch = OrientationMath.currentPitchDegrees(
        sample: photo.motionSample,
        captureReference: reference
      )
      let ring = OrientationMath.classifyCaptureRing(
        pitchDegrees: pitch,
        configuration: configuration
      )
      guard
        let target = OrientationMath.closestTarget(
          in: ring,
          targets: plan.targets,
          sample: photo.motionSample,
          captureReference: reference
        )
      else {
        throw CameraCaptureError.sessionNotConfigured
      }

      let exposureAlignment = OrientationMath.navigationReading(
        sample: photo.motionSample,
        captureReference: reference,
        target: target,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
      let pose = OrientationMath.poseMetadata(
        sample: photo.motionSample,
        captureReference: reference,
        referenceFrameName: motion.referenceFrameName
      )
      let record = try await packageStore.append(
        photo: photo,
        target: target,
        pose: pose,
        alignment: AlignmentMetadata(
          directionErrorDegrees: exposureAlignment.directionErrorDegrees,
          yawErrorDegrees: exposureAlignment.yawErrorDegrees,
          pitchErrorDegrees: exposureAlignment.pitchErrorDegrees,
          requiredToleranceDegrees: configuration.alignmentToleranceDegrees,
          requiredStableDurationSeconds: configuration.stableHoldDurationSeconds
        ),
        primaryCapture: PrimaryCaptureMetadata(
          imageFilename: "",
          targetId: target.id,
          classifiedRing: ring
        )
      )

      capturedFrames.append(record)
      try await camera.lockExposureFocusWhiteBalance()
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationReading = .unavailable
      isCapturingPhoto = false

      if capturedFrames.count == totalFrameCount {
        await finalizeCapture()
      } else {
        phase = .capturingPoints
        statusMessage =
          "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
        stopCameraIfCaptureTabInactive()
      }
    } catch {
      isCapturingPhoto = false
      captureReference = nil
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      captureErrorMessage = error.localizedDescription
      statusMessage = "Primary capture failed; try again"
      stopCameraIfCaptureTabInactive()
    }
  }

  private func capture(
    target: CaptureTarget,
    reference: CaptureReferenceFrame
  ) async {
    captureErrorMessage = nil
    do {
      let photo = try await camera.capturePhoto()
      let exposureAlignment = OrientationMath.navigationReading(
        sample: photo.motionSample,
        captureReference: reference,
        target: target,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
      let pose = OrientationMath.poseMetadata(
        sample: photo.motionSample,
        captureReference: reference,
        referenceFrameName: motion.referenceFrameName
      )
      let record = try await packageStore.append(
        photo: photo,
        target: target,
        pose: pose,
        alignment: AlignmentMetadata(
          directionErrorDegrees: exposureAlignment.directionErrorDegrees,
          yawErrorDegrees: exposureAlignment.yawErrorDegrees,
          pitchErrorDegrees: exposureAlignment.pitchErrorDegrees,
          requiredToleranceDegrees: configuration.alignmentToleranceDegrees,
          requiredStableDurationSeconds: configuration.stableHoldDurationSeconds
        )
      )
      capturedFrames.append(record)
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationReading = .unavailable
      activeTarget = nil
      isCapturingPhoto = false

      if capturedFrames.count == totalFrameCount {
        await finalizeCapture()
      } else {
        statusMessage =
          "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
        stopCameraIfCaptureTabInactive()
      }
    } catch {
      isCapturingPhoto = false
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      autoCaptureBlockedUntil = ProcessInfo.processInfo.systemUptime + 1
      captureErrorMessage = error.localizedDescription
      statusMessage = "Capture failed; realign to retry"
      stopCameraIfCaptureTabInactive()
    }
  }

  private func finalizeCapture() async {
    statusMessage = "Saving capture to gallery"
    await camera.unlockExposureFocusWhiteBalance()
    camera.stop()
    motion.stop()

    do {
      let package = try await packageStore.finalize()
      phase = .saved(package)
      statusMessage = "Capture saved"
      await refreshGallery()
    } catch {
      phase = .failed(error.localizedDescription)
      statusMessage = "Could not save capture package"
    }
  }

  private func traceCaptureReferenceIfEnabled() {
    guard navigationTraceEnabled, let captureReference else { return }
    print(
      "SPHERA_NAV reference quaternion=\(captureReference.motionQuaternionInterpretation.rawValue)"
    )
  }

  private func traceNavigationIfEnabled(
    sample: MotionSample,
    target: CaptureTarget,
    reading: CaptureNavigationReading
  ) {
    guard navigationTraceEnabled else { return }
    guard sample.monotonicTimestampSeconds - lastNavigationTraceTimestamp >= 0.25 else {
      return
    }
    lastNavigationTraceTimestamp = sample.monotonicTimestampSeconds
    print(
      "SPHERA_NAV target=\(target.id) yawError=\(reading.yawErrorDegrees.formatted(.number.precision(.fractionLength(1)))) pitchError=\(reading.pitchErrorDegrees.formatted(.number.precision(.fractionLength(1)))) directionError=\(reading.directionErrorDegrees.formatted(.number.precision(.fractionLength(1)))) aligned=\(reading.isAligned)"
    )
  }
}

extension CaptureWorkflowPhase {
  fileprivate var isTerminal: Bool {
    switch self {
    case .saved, .completed, .failed:
      true
    default:
      false
    }
  }
}
