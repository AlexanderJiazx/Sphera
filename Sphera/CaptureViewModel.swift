import AVFoundation
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
  case capturing
  case stitching
  case completed(CaptureCompletion)
  case failed(String)
}

@MainActor
final class CaptureViewModel: ObservableObject {
  @Published var configuration = CaptureConfiguration.debugPreset
  @Published private(set) var phase: CaptureWorkflowPhase = .setup
  @Published private(set) var plan = CapturePlan(configuration: .debugPreset)
  @Published private(set) var currentTargetIndex = 0
  @Published private(set) var capturedFrames: [CapturedFrameRecord] = []
  @Published private(set) var currentMotionSample: MotionSample?
  @Published private(set) var navigationReading = CaptureNavigationReading.unavailable
  @Published private(set) var navigationInstruction = CaptureNavigationInstruction.preparing
  @Published private(set) var stableHoldProgress = 0.0
  @Published private(set) var isCapturingPhoto = false
  @Published private(set) var statusMessage = "Ready"
  @Published private(set) var captureErrorMessage: String?

  let motion: MotionTrackingService
  let camera: CameraCaptureService

  private let packageStore: CapturePackageStore
  private let stitcher: any PanoramaStitching
  private var subscriptions = Set<AnyCancellable>()
  private var captureReference: CaptureReferenceFrame?
  private var alignmentHoldTracker = AlignmentHoldTracker()
  private var guidanceState = CaptureGuidanceState()
  private var autoCaptureBlockedUntil = 0.0
  private var lastNavigationTraceTimestamp = -Double.infinity
  private let navigationTraceEnabled =
    ProcessInfo.processInfo.arguments.contains("--navigation-trace")

  init(
    packageStore: CapturePackageStore = CapturePackageStore(),
    stitcher: any PanoramaStitching = SpheraEngineAdapter(
      nativeEngine: OpenCVSpheraEngine()
    )
  ) {
    let motion = MotionTrackingService()
    self.motion = motion
    camera = CameraCaptureService(motionStore: motion.sampleStore)
    self.packageStore = packageStore
    self.stitcher = stitcher

    motion.$currentSample
      .sink { [weak self] sample in
        self?.handleMotionSample(sample)
      }
      .store(in: &subscriptions)
  }

  var currentTarget: CaptureTarget? {
    guard plan.targets.indices.contains(currentTargetIndex) else { return nil }
    return plan.targets[currentTargetIndex]
  }

  var totalFrameCount: Int { plan.targets.count }

  var progressFraction: Double {
    guard totalFrameCount > 0 else { return 0 }
    return Double(capturedFrames.count) / Double(totalFrameCount)
  }

  func rebuildPlan() {
    guard phase == .setup else { return }
    configuration.horizontalCount = min(16, max(4, configuration.horizontalCount))
    configuration.downwardCount = min(6, max(4, configuration.downwardCount))
    configuration.upwardCount = min(6, max(4, configuration.upwardCount))
    plan = CapturePlan(configuration: configuration)
  }

  func startCapture() {
    guard phase == .setup || phase.isTerminal else { return }
    rebuildPlan()
    phase = .preparing
    statusMessage = "Starting camera and motion"
    captureErrorMessage = nil
    capturedFrames = []
    currentTargetIndex = 0
    navigationReading = .unavailable
    navigationInstruction = .preparing
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    guidanceState.reset()
    autoCaptureBlockedUntil = 0
    captureReference = nil

    Task {
      do {
        try motion.start()
        try await camera.start()
        _ = try await motion.waitForFirstSample()
        guard let referenceSample = motion.currentSample else {
          throw MotionTrackingError.timedOut
        }
        captureReference = OrientationMath.makeCaptureReference(from: referenceSample)
        traceCaptureReferenceIfEnabled()
        _ = try await packageStore.begin(
          plan: plan,
          coreMotionReferenceFrame: motion.referenceFrameName
        )
        phase = .capturing
        statusMessage = "Align with target 1 of \(totalFrameCount)"
      } catch {
        camera.stop()
        motion.stop()
        phase = .failed(error.localizedDescription)
        statusMessage = "Capture unavailable"
      }
    }
  }

  func stopCapture() {
    camera.stop()
    motion.stop()
    isCapturingPhoto = false
    phase = .setup
    statusMessage = "Ready"
    captureErrorMessage = nil
    navigationReading = .unavailable
    navigationInstruction = .preparing
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    guidanceState.reset()
    captureReference = nil
    currentTargetIndex = 0
    capturedFrames = []
    plan = CapturePlan(configuration: configuration)
  }

  func prepareNewCapture() {
    stopCapture()
  }

  private func handleMotionSample(_ sample: MotionSample?) {
    currentMotionSample = sample
    guard phase == .capturing,
      !isCapturingPhoto,
      let sample,
      let captureReference,
      let target = currentTarget
    else {
      return
    }

    let reading = OrientationMath.navigationReading(
      sample: sample,
      captureReference: captureReference,
      target: target,
      toleranceDegrees: configuration.alignmentToleranceDegrees
    )
    navigationReading = reading

    let holdUpdate = alignmentHoldTracker.update(
      isAligned: reading.isAligned,
      timestamp: sample.monotonicTimestampSeconds,
      requiredDuration: configuration.stableHoldDurationSeconds,
      blockedUntilTimestamp: autoCaptureBlockedUntil
    )
    stableHoldProgress = holdUpdate.progress
    navigationInstruction = guidanceState.update(
      targetID: target.id,
      reading: reading,
      toleranceDegrees: configuration.alignmentToleranceDegrees,
      stableHoldProgress: stableHoldProgress,
      isReadingAvailable: true,
      isCapturingPhoto: false
    )
    traceNavigationIfEnabled(
      sample: sample,
      target: target,
      reading: reading,
      instruction: navigationInstruction
    )

    guard reading.isAligned else {
      statusMessage = navigationInstruction.movement.statusMessage
      return
    }

    statusMessage = stableHoldProgress < 1 ? "Hold aligned" : "Capturing"

    guard holdUpdate.shouldCapture else { return }
    isCapturingPhoto = true
    navigationInstruction = .capturing
    Task {
      await capture(target: target, reference: captureReference)
    }
  }

  private func capture(target: CaptureTarget, reference: CaptureReferenceFrame) async {
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
      currentTargetIndex += 1
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationReading = .unavailable
      navigationInstruction = .preparing
      isCapturingPhoto = false

      if capturedFrames.count == totalFrameCount {
        await finalizeCapture()
      } else {
        statusMessage = "Move to target \(currentTargetIndex + 1) of \(totalFrameCount)"
      }
    } catch {
      isCapturingPhoto = false
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationInstruction = .preparing
      autoCaptureBlockedUntil = ProcessInfo.processInfo.systemUptime + 1
      captureErrorMessage = error.localizedDescription
      statusMessage = "Capture failed; realign to retry"
    }
  }

  private func finalizeCapture() async {
    phase = .stitching
    statusMessage = "Passing pose-initialized capture to Sphera engine"
    camera.stop()
    motion.stop()

    do {
      let package = try await packageStore.finalize()
      do {
        let result = try await stitcher.stitch(package: package)
        phase = .completed(
          CaptureCompletion(
            package: package,
            stitchingResult: result,
            stitchingMessage: nil
          )
        )
        statusMessage = "Panorama complete"
      } catch {
        phase = .completed(
          CaptureCompletion(
            package: package,
            stitchingResult: nil,
            stitchingMessage: error.localizedDescription
          )
        )
        statusMessage = "Capture package complete"
      }
    } catch {
      phase = .failed(error.localizedDescription)
      statusMessage = "Could not finalize capture package"
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
    reading: CaptureNavigationReading,
    instruction: CaptureNavigationInstruction
  ) {
    guard navigationTraceEnabled else { return }
    guard sample.monotonicTimestampSeconds - lastNavigationTraceTimestamp >= 0.25 else {
      return
    }
    lastNavigationTraceTimestamp = sample.monotonicTimestampSeconds
    print(
      "SPHERA_NAV target=\(target.id) yawError=\(reading.yawErrorDegrees.formatted(.number.precision(.fractionLength(1)))) pitchError=\(reading.pitchErrorDegrees.formatted(.number.precision(.fractionLength(1)))) directionError=\(reading.directionErrorDegrees.formatted(.number.precision(.fractionLength(1)))) command=\(String(describing: instruction.movement))"
    )
  }
}

extension CaptureMovement {
  fileprivate var statusMessage: String {
    switch self {
    case .turnLeft: "Turn left"
    case .turnRight: "Turn right"
    case .tiltUp: "Tilt up"
    case .tiltDown: "Tilt down"
    case .holdStill: "Hold still"
    case .capturing: "Capturing"
    case .preparing: "Reading phone orientation"
    }
  }
}

extension CaptureWorkflowPhase {
  fileprivate var isTerminal: Bool {
    switch self {
    case .completed, .failed:
      true
    default:
      false
    }
  }
}
