import Foundation

enum ExperimentalTrackingQuality: Equatable, Sendable {
  case normal
  case limited
  case severelyLimited
}

struct ExperimentalScanSample: Equatable, Sendable {
  var yawDegrees: Double
  var pitchDegrees: Double
  var rollDegrees: Double
  var timestamp: TimeInterval
  var trackingQuality: ExperimentalTrackingQuality
  var rotationRateMagnitude: Double
  var translationMeters: Double
}

struct ExperimentalScanUpdate: Equatable, Sendable {
  var captureIndex: Int?
  var progress: Double
  var capturedCount: Int
  var targetCount: Int
  var yawOffsetDegrees: Double
  var pitchErrorDegrees: Double
  var rollDegrees: Double
  var isAbovePath: Bool
  var isBelowPath: Bool
  var isRolled: Bool
  var isWrongDirection: Bool
  var isTranslatingTooMuch: Bool
  var isLineComplete: Bool
  var nextTargetYawOffsetDegrees: Double?
  var lockedDirection: ExperimentalCaptureDirection?
  var qualityNotes: [String]
}

/// Angular keyframe selector for one scan line. Capture is driven by unwrapped
/// yaw progress, not elapsed time.
struct ExperimentalScanProgressor: Sendable {
  var configuration: ExperimentalPanoramaConfiguration

  private(set) var activeLine: PanoramaScanLine?
  private var lineStartYaw: Double?
  private var lastRawYaw: Double?
  private var unwrappedYaw: Double?
  private var smoothedYaw: Double?
  private var capturedIndices: Set<Int> = []
  private var inFlightIndex: Int?
  private var lockedDirection: ExperimentalCaptureDirection?
  private var peakDirectedOffset: Double = 0
  private var previousSmoothedYaw: Double?

  init(configuration: ExperimentalPanoramaConfiguration = .default) {
    self.configuration = configuration
  }

  mutating func reset() {
    activeLine = nil
    lineStartYaw = nil
    lastRawYaw = nil
    unwrappedYaw = nil
    smoothedYaw = nil
    capturedIndices = []
    inFlightIndex = nil
    lockedDirection = nil
    peakDirectedOffset = 0
    previousSmoothedYaw = nil
  }

  mutating func beginLine(_ line: PanoramaScanLine, currentYawDegrees: Double) {
    activeLine = line
    lineStartYaw = currentYawDegrees
    lastRawYaw = currentYawDegrees
    unwrappedYaw = currentYawDegrees
    smoothedYaw = currentYawDegrees
    capturedIndices = []
    inFlightIndex = nil
    lockedDirection = nil
    peakDirectedOffset = 0
    previousSmoothedYaw = currentYawDegrees
  }

  mutating func endLine() {
    activeLine = nil
    inFlightIndex = nil
  }

  mutating func noteCaptureFinished(success: Bool) {
    if !success, let index = inFlightIndex {
      capturedIndices.remove(index)
    }
    inFlightIndex = nil
  }

  mutating func update(_ sample: ExperimentalScanSample) -> ExperimentalScanUpdate {
    guard let line = activeLine, let startYaw = lineStartYaw else {
      return ExperimentalScanUpdate(
        captureIndex: nil,
        progress: 0,
        capturedCount: 0,
        targetCount: 0,
        yawOffsetDegrees: 0,
        pitchErrorDegrees: 0,
        rollDegrees: 0,
        isAbovePath: false,
        isBelowPath: false,
        isRolled: false,
        isWrongDirection: false,
        isTranslatingTooMuch: false,
        isLineComplete: false,
        nextTargetYawOffsetDegrees: nil,
        lockedDirection: nil,
        qualityNotes: []
      )
    }

    let previousSmoothed = previousSmoothedYaw ?? smoothedYaw ?? sample.yawDegrees
    unwrapAndSmooth(sample.yawDegrees)
    let currentSmoothed = smoothedYaw ?? sample.yawDegrees
    let yawDelta = currentSmoothed - previousSmoothed
    previousSmoothedYaw = currentSmoothed

    let offset = currentSmoothed - startYaw
    updateDirectionLock(offset: offset)

    let pitchTarget = configuration.targetPitchDegrees(for: line)
    let pitchError = sample.pitchDegrees - pitchTarget
    let isAbove = pitchError > configuration.maxPitchErrorForCaptureDegrees
    let isBelow = pitchError < -configuration.maxPitchErrorForCaptureDegrees
    let isRolled = configuration.isRollBlockingCapture(sample.rollDegrees)
    let isWrongDirection = isMovingAgainstLockedDirection(
      offset: offset,
      yawDelta: yawDelta
    )
    let isTranslating =
      sample.translationMeters > configuration.maxTranslationWarningMeters

    let directedOffset = directedProgress(offset)
    let targetCount = configuration.imageCount(for: line)
    let span = max(configuration.yawStepDegrees(for: line) * Double(max(targetCount - 1, 1)), 1)
    let progress = min(1, max(0, directedOffset / span))
    let capturedCount = capturedIndices.count
    let isComplete =
      targetCount > 0 && capturedCount >= targetCount && inFlightIndex == nil

    var qualityNotes: [String] = []
    if sample.trackingQuality == .severelyLimited {
      qualityNotes.append("tracking-severely-limited")
    } else if sample.trackingQuality == .limited {
      qualityNotes.append("tracking-limited")
    }
    if sample.rotationRateMagnitude > configuration.maxRotationRateRadiansPerSecond {
      qualityNotes.append("excessive-rotation-rate")
    }
    if isTranslating {
      qualityNotes.append("excessive-translation")
    }
    if configuration.isPitchErrorBlockingCapture(pitchError) {
      qualityNotes.append("off-path-pitch")
    }
    if configuration.isRollBlockingCapture(sample.rollDegrees) {
      qualityNotes.append("off-upright-roll")
    }
    if isWrongDirection {
      qualityNotes.append("wrong-rotation-direction")
    }

    var captureIndex: Int?
    var nextTarget: Double?
    let canAdvancePastStart = lockedDirection != nil || capturedIndices.isEmpty
    if !isComplete, inFlightIndex == nil, targetCount > 0, !isWrongDirection, canAdvancePastStart {
      for index in 0..<targetCount where !capturedIndices.contains(index) {
        let target = configuration.targetYawOffsetDegrees(for: line, index: index)
        nextTarget = target
        let progressForTarget = index == 0 ? 0 : directedOffset
        let reached = progressForTarget + configuration.captureHysteresisDegrees >= target
        guard reached else { break }

        let shouldDefer = shouldDeferCapture(
          sample: sample,
          pitchError: pitchError
        )
        if shouldDefer {
          break
        }

        capturedIndices.insert(index)
        inFlightIndex = index
        captureIndex = index
        break
      }
    }

    if nextTarget == nil, !isComplete, targetCount > 0 {
      if let next = (0..<targetCount).first(where: { !capturedIndices.contains($0) }) {
        nextTarget = configuration.targetYawOffsetDegrees(for: line, index: next)
      }
    }

    return ExperimentalScanUpdate(
      captureIndex: captureIndex,
      progress: isComplete ? 1 : progress,
      capturedCount: capturedIndices.count,
      targetCount: targetCount,
      yawOffsetDegrees: offset,
      pitchErrorDegrees: pitchError,
      rollDegrees: sample.rollDegrees,
      isAbovePath: isAbove,
      isBelowPath: isBelow,
      isRolled: isRolled,
      isWrongDirection: isWrongDirection,
      isTranslatingTooMuch: isTranslating,
      isLineComplete: isComplete,
      nextTargetYawOffsetDegrees: nextTarget,
      lockedDirection: lockedDirection,
      qualityNotes: qualityNotes
    )
  }

  private mutating func unwrapAndSmooth(_ rawYaw: Double) {
    if let lastRawYaw, let unwrappedYaw {
      let delta = ExperimentalPoseMath.shortestDeltaDegrees(from: lastRawYaw, to: rawYaw)
      let updated = unwrappedYaw + delta
      self.unwrappedYaw = updated
      let alpha = min(1, max(0.01, configuration.yawSmoothingAlpha))
      let previous = smoothedYaw ?? updated
      smoothedYaw = previous + alpha * (updated - previous)
    } else {
      unwrappedYaw = rawYaw
      smoothedYaw = rawYaw
    }
    lastRawYaw = rawYaw
  }

  private mutating func updateDirectionLock(offset: Double) {
    if lockedDirection != nil { return }
    switch configuration.captureDirection {
    case .clockwise:
      lockedDirection = .clockwise
    case .counterclockwise:
      lockedDirection = .counterclockwise
    case .automatic:
      if offset >= configuration.directionLockDegrees {
        lockedDirection = .clockwise
      } else if offset <= -configuration.directionLockDegrees {
        lockedDirection = .counterclockwise
      }
    }
  }

  private func directedProgress(_ offset: Double) -> Double {
    switch lockedDirection {
    case .counterclockwise:
      -offset
    case .clockwise:
      offset
    case .automatic, .none:
      abs(offset)
    }
  }

  private mutating func isMovingAgainstLockedDirection(
    offset: Double,
    yawDelta: Double
  ) -> Bool {
    guard let lockedDirection else {
      return false
    }

    let directed = lockedDirection == .clockwise ? offset : -offset
    peakDirectedOffset = max(peakDirectedOffset, directed)
    let rewind = peakDirectedOffset - directed
    let reverseMotion: Bool
    switch lockedDirection {
    case .clockwise:
      reverseMotion = yawDelta < -configuration.reverseMotionDegrees
    case .counterclockwise:
      reverseMotion = yawDelta > configuration.reverseMotionDegrees
    case .automatic:
      reverseMotion = false
    }
    return rewind >= configuration.wrongDirectionDegrees || reverseMotion
  }

  private func shouldDeferCapture(
    sample: ExperimentalScanSample,
    pitchError: Double
  ) -> Bool {
    if configuration.isPitchErrorBlockingCapture(pitchError) {
      return true
    }
    if configuration.isRollBlockingCapture(sample.rollDegrees) {
      return true
    }
    if configuration.isTranslationBlockingCapture(sample.translationMeters) {
      return true
    }
    if sample.trackingQuality == .severelyLimited { return true }
    if configuration.isRotationRateBlockingCapture(sample.rotationRateMagnitude) {
      return true
    }
    return false
  }
}
