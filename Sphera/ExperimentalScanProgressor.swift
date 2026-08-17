import Foundation

enum ExperimentalTrackingQuality: Equatable, Sendable {
  case normal
  case limited
  case severelyLimited
}

/// Whether a planned angle in the current row still needs a photo.
enum ExperimentalTargetState: String, Equatable, Sendable {
  case pending
  case captured
  /// Passed by while a gate was closed. The row continues without it.
  case skipped
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
  var skippedCount: Int
  var targetCount: Int
  var targetStates: [ExperimentalTargetState]
  var yawOffsetDegrees: Double
  var directedYawOffsetDegrees: Double
  var sweepFraction: Double
  var pitchErrorDegrees: Double
  var rollDegrees: Double
  var isAbovePath: Bool
  var isBelowPath: Bool
  var isRolled: Bool
  var isWrongDirection: Bool
  var isTranslatingTooMuch: Bool
  var isLineComplete: Bool
  var blockReason: ExperimentalCaptureBlockReason?
  var nextTargetYawOffsetDegrees: Double?
  var lockedDirection: ExperimentalCaptureDirection?
  var qualityNotes: [String]

  static let empty = ExperimentalScanUpdate(
    captureIndex: nil,
    progress: 0,
    capturedCount: 0,
    skippedCount: 0,
    targetCount: 0,
    targetStates: [],
    yawOffsetDegrees: 0,
    directedYawOffsetDegrees: 0,
    sweepFraction: 0,
    pitchErrorDegrees: 0,
    rollDegrees: 0,
    isAbovePath: false,
    isBelowPath: false,
    isRolled: false,
    isWrongDirection: false,
    isTranslatingTooMuch: false,
    isLineComplete: false,
    blockReason: nil,
    nextTargetYawOffsetDegrees: nil,
    lockedDirection: nil,
    qualityNotes: []
  )

  /// A row that has not started yet: the right number of targets, none of them
  /// resolved.
  static func pending(count: Int) -> ExperimentalScanUpdate {
    var update = ExperimentalScanUpdate.empty
    update.targetCount = count
    update.targetStates = Array(repeating: .pending, count: max(count, 0))
    return update
  }
}

/// Angular keyframe selector for one scan line. Capture is driven by unwrapped
/// yaw progress, not elapsed time.
///
/// Every planned angle ends up either captured or explicitly skipped, so a row
/// always terminates: a gate that stays closed costs one photo, never the
/// whole session.
struct ExperimentalScanProgressor: Sendable {
  /// A target that keeps failing to save is abandoned rather than retried
  /// forever at the same angle.
  static let maximumAttemptsPerTarget = 3

  var configuration: ExperimentalPanoramaConfiguration

  private(set) var activeLine: PanoramaScanLine?
  private var lineStartYaw: Double?
  private var lastRawYaw: Double?
  private var unwrappedYaw: Double?
  private var smoothedYaw: Double?
  private var capturedIndices: Set<Int> = []
  private var skippedIndices: Set<Int> = []
  private var failedAttempts: [Int: Int] = [:]
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
    skippedIndices = []
    failedAttempts = [:]
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
    skippedIndices = []
    failedAttempts = [:]
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
    guard let index = inFlightIndex else {
      inFlightIndex = nil
      return
    }
    inFlightIndex = nil
    guard !success else {
      failedAttempts[index] = nil
      return
    }
    capturedIndices.remove(index)
    let attempts = (failedAttempts[index] ?? 0) + 1
    failedAttempts[index] = attempts
    if attempts >= Self.maximumAttemptsPerTarget {
      skippedIndices.insert(index)
    }
  }

  var capturedIndexCount: Int { capturedIndices.count }
  var skippedIndexCount: Int { skippedIndices.count }
  var skippedIndexList: [Int] { skippedIndices.sorted() }

  mutating func update(
    _ sample: ExperimentalScanSample,
    canCapture: Bool = true
  ) -> ExperimentalScanUpdate {
    guard let line = activeLine, let startYaw = lineStartYaw else {
      return .empty
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
    let isWrongDirection = isMovingAgainstLockedDirection(offset: offset, yawDelta: yawDelta)
    let isTranslating = configuration.isTranslationExcessive(sample.translationMeters)

    let directedOffset = directedProgress(offset)
    let targetCount = configuration.imageCount(for: line)
    let range = max(configuration.scanRangeDegrees, 1)
    let sweepFraction = min(1, max(0, directedOffset / range))

    let blockReason = blockReason(
      sample: sample,
      pitchError: pitchError,
      isWrongDirection: isWrongDirection
    )

    var captureIndex: Int?
    var nextTarget: Double?
    if targetCount > 0, inFlightIndex == nil {
      let slack = configuration.missedTargetSlackDegrees(for: line)
      while let index = firstUnresolvedIndex(in: targetCount) {
        let target = configuration.targetYawOffsetDegrees(for: line, index: index)
        nextTarget = target

        // The first frame anchors the row at wherever the user is standing.
        let hasReachedTarget =
          index == 0 || directedOffset + configuration.captureHysteresisDegrees >= target
        guard hasReachedTarget else { break }

        if blockReason != nil {
          if directedOffset > target + slack {
            skippedIndices.insert(index)
            continue
          }
          break
        }

        // The controller is still writing the previous photo. Leave the target
        // unresolved so it is offered again on the next frame.
        guard canCapture else { break }

        capturedIndices.insert(index)
        inFlightIndex = index
        captureIndex = index
        break
      }
    }

    // A sweep that has come all the way around cannot reach anything it has
    // not already passed.
    if captureIndex == nil, inFlightIndex == nil, directedOffset >= range {
      for index in 0..<targetCount
      where !capturedIndices.contains(index) && !skippedIndices.contains(index) {
        skippedIndices.insert(index)
      }
    }

    let resolvedCount = capturedIndices.count + skippedIndices.count
    let isComplete = targetCount > 0 && resolvedCount >= targetCount && inFlightIndex == nil
    if isComplete { nextTarget = nil }

    var qualityNotes: [String] = []
    if sample.trackingQuality == .severelyLimited {
      qualityNotes.append("tracking-severely-limited")
    } else if sample.trackingQuality == .limited {
      qualityNotes.append("tracking-limited")
    }
    if configuration.isRotationRateBlockingCapture(sample.rotationRateMagnitude) {
      qualityNotes.append("excessive-rotation-rate")
    }
    if isTranslating {
      qualityNotes.append("excessive-translation")
    }
    if configuration.isPitchErrorBlockingCapture(pitchError) {
      qualityNotes.append("off-path-pitch")
    }
    if isRolled {
      qualityNotes.append("off-upright-roll")
    }
    if isWrongDirection {
      qualityNotes.append("wrong-rotation-direction")
    }

    return ExperimentalScanUpdate(
      captureIndex: captureIndex,
      progress: targetCount > 0
        ? min(1, Double(resolvedCount) / Double(targetCount))
        : 0,
      capturedCount: capturedIndices.count,
      skippedCount: skippedIndices.count,
      targetCount: targetCount,
      targetStates: targetStates(in: targetCount),
      yawOffsetDegrees: offset,
      directedYawOffsetDegrees: directedOffset,
      sweepFraction: sweepFraction,
      pitchErrorDegrees: pitchError,
      rollDegrees: sample.rollDegrees,
      isAbovePath: isAbove,
      isBelowPath: isBelow,
      isRolled: isRolled,
      isWrongDirection: isWrongDirection,
      isTranslatingTooMuch: isTranslating,
      isLineComplete: isComplete,
      blockReason: blockReason,
      nextTargetYawOffsetDegrees: nextTarget,
      lockedDirection: lockedDirection,
      qualityNotes: qualityNotes
    )
  }

  private func firstUnresolvedIndex(in targetCount: Int) -> Int? {
    (0..<targetCount).first {
      !capturedIndices.contains($0) && !skippedIndices.contains($0)
    }
  }

  private func targetStates(in targetCount: Int) -> [ExperimentalTargetState] {
    (0..<targetCount).map { index in
      if capturedIndices.contains(index) { return .captured }
      if skippedIndices.contains(index) { return .skipped }
      return .pending
    }
  }

  /// Ordered by which correction the user should make first.
  private func blockReason(
    sample: ExperimentalScanSample,
    pitchError: Double,
    isWrongDirection: Bool
  ) -> ExperimentalCaptureBlockReason? {
    if configuration.isRollBlockingCapture(sample.rollDegrees) {
      return .rolled
    }
    if configuration.isPitchErrorBlockingCapture(pitchError) {
      return pitchError > 0 ? .pitchTooHigh : .pitchTooLow
    }
    if sample.trackingQuality == .severelyLimited {
      return .trackingLimited
    }
    if isWrongDirection {
      return .reversedDirection
    }
    if configuration.isRotationRateBlockingCapture(sample.rotationRateMagnitude) {
      return .tooFast
    }
    return nil
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

    let directedDelta = lockedDirection == .clockwise ? yawDelta : -yawDelta
    // Turning the right way again clears the warning immediately, even before
    // the lost ground is made up. Otherwise the user obeys the instruction and
    // watches nothing change, which reads as a frozen app.
    if directedDelta > Self.motionEpsilonDegrees { return false }

    let rewind = peakDirectedOffset - directed
    let isReversing = directedDelta < -configuration.reverseMotionDegrees
    return rewind >= configuration.wrongDirectionDegrees || isReversing
  }

  /// Smoothed yaw never sits perfectly still; anything under this is hand shake.
  private static let motionEpsilonDegrees = 0.05
}
