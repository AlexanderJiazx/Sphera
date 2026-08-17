import Foundation

enum CaptureSessionMode: String, CaseIterable, Codable, Sendable {
  case standard
  case experimentalARKit

  /// Short label for the on-camera chip.
  var title: String {
    switch self {
    case .standard: "Points"
    case .experimentalARKit: "Sweep"
    }
  }

  /// Longer label for the picker, where there is room to explain.
  var menuTitle: String {
    switch self {
    case .standard: "Points"
    case .experimentalARKit: "Sweep (Beta)"
    }
  }

  var symbolName: String {
    switch self {
    case .standard: "circle.grid.3x3"
    case .experimentalARKit: "pano"
    }
  }

  var accessibilityLabel: String {
    switch self {
    case .standard: "Points capture"
    case .experimentalARKit: "Guided sweep capture, beta"
    }
  }
}

enum PanoramaScanLine: String, Codable, CaseIterable, Sendable {
  case upward
  case horizontal
  case downward

  /// Engineering name, used in logs and in the exported manifest.
  var displayName: String {
    switch self {
    case .upward: "Upward"
    case .horizontal: "Horizontal"
    case .downward: "Downward"
    }
  }

  /// What the row is called in the interface.
  var rowName: String {
    switch self {
    case .upward: "Upper row"
    case .horizontal: "Middle row"
    case .downward: "Lower row"
    }
  }

  var shortRowName: String {
    switch self {
    case .upward: "Upper"
    case .horizontal: "Middle"
    case .downward: "Lower"
    }
  }

  /// Shown while the user is getting into position for the row.
  var alignmentInstruction: String {
    switch self {
    case .upward: "Tilt up to the guide"
    case .horizontal: "Hold iPhone upright"
    case .downward: "Tilt down to the guide"
    }
  }

  var expectedOrientationLabel: String {
    switch self {
    case .upward: "Tilt up"
    case .horizontal: "Look ahead"
    case .downward: "Tilt down"
    }
  }
}

enum ExperimentalCaptureDirection: String, Codable, Sendable {
  case clockwise
  case counterclockwise
  /// Lock to the first clear rotation after the line starts.
  case automatic

  var rotationInstruction: String {
    switch self {
    case .clockwise: "Turn right"
    case .counterclockwise: "Turn left"
    case .automatic: "Turn either way"
    }
  }

  /// Used once the row is under way, so the instruction reads as encouragement
  /// rather than as a fresh command every frame.
  var continuedInstruction: String {
    switch self {
    case .clockwise: "Keep turning right"
    case .counterclockwise: "Keep turning left"
    case .automatic: "Keep turning"
    }
  }
}

/// Why the sweep is not taking a photo right now. Every deferral has a reason
/// the interface can show, so capture can never stall without an explanation.
enum ExperimentalCaptureBlockReason: String, Codable, Equatable, Sendable {
  case pitchTooHigh
  case pitchTooLow
  case rolled
  case reversedDirection
  case tooFast
  case trackingLimited

  var instruction: String {
    switch self {
    case .pitchTooHigh: "Tilt down to the guide"
    case .pitchTooLow: "Tilt up to the guide"
    case .rolled: "Keep iPhone level"
    case .reversedDirection: "Keep turning the same way"
    case .tooFast: "Slow down"
    case .trackingLimited: "Hold still"
    }
  }

  /// Short form for the heads-up badge over the viewfinder.
  var badge: String {
    switch self {
    case .pitchTooHigh: "Tilt down"
    case .pitchTooLow: "Tilt up"
    case .rolled: "Hold level"
    case .reversedDirection: "Wrong way"
    case .tooFast: "Slow down"
    case .trackingLimited: "Hold still"
    }
  }

  var qualityNote: String {
    switch self {
    case .pitchTooHigh, .pitchTooLow: "off-path-pitch"
    case .rolled: "off-upright-roll"
    case .reversedDirection: "wrong-rotation-direction"
    case .tooFast: "excessive-rotation-rate"
    case .trackingLimited: "tracking-severely-limited"
    }
  }
}

struct ExperimentalPanoramaConfiguration: Codable, Equatable, Sendable {
  var horizontalImageCount: Int
  var upwardImageCount: Int
  var downwardImageCount: Int
  var scanRangeDegrees: Double
  var horizontalPitchDegrees: Double
  var upwardPitchDegrees: Double
  var downwardPitchDegrees: Double
  var pitchToleranceDegrees: Double
  var pathDriftWarningDegrees: Double
  var maxPitchErrorForCaptureDegrees: Double
  var pitchGuideScaleDegrees: Double
  var rollWarningDegrees: Double
  var maxRollForCaptureDegrees: Double
  var wrongDirectionDegrees: Double
  var directionLockDegrees: Double
  var reverseMotionDegrees: Double
  var yawSmoothingAlpha: Double
  var captureHysteresisDegrees: Double
  var maxTranslationWarningMeters: Double
  var maxRotationRateRadiansPerSecond: Double
  var captureDirection: ExperimentalCaptureDirection

  /// Tolerances are what a person can actually hold while turning a full
  /// circle by hand. Tighter gates look precise on paper and read as "the app
  /// stopped working" in the field.
  static let `default` = ExperimentalPanoramaConfiguration(
    horizontalImageCount: 16,
    upwardImageCount: 12,
    downwardImageCount: 12,
    scanRangeDegrees: 360,
    horizontalPitchDegrees: 0,
    upwardPitchDegrees: 40,
    downwardPitchDegrees: -40,
    pitchToleranceDegrees: 7,
    pathDriftWarningDegrees: 5,
    maxPitchErrorForCaptureDegrees: 9,
    pitchGuideScaleDegrees: 14,
    rollWarningDegrees: 5,
    maxRollForCaptureDegrees: 9,
    wrongDirectionDegrees: 8,
    directionLockDegrees: 6,
    reverseMotionDegrees: 2.5,
    yawSmoothingAlpha: 0.18,
    captureHysteresisDegrees: 0.75,
    // Pivoting on your feet moves the camera by roughly an arm's length, which
    // is normal for a handheld sweep. This only annotates the dataset; it never
    // withholds a photo.
    maxTranslationWarningMeters: 1.2,
    maxRotationRateRadiansPerSecond: 2.5,
    captureDirection: .automatic
  )

  var totalImageCount: Int {
    horizontalImageCount + upwardImageCount + downwardImageCount
  }

  /// Horizontal first, then the two elevation passes. The UI starts on the
  /// middle line so the user can begin at a natural phone posture.
  var scanLineOrder: [PanoramaScanLine] {
    [.horizontal, .upward, .downward]
  }

  func imageCount(for line: PanoramaScanLine) -> Int {
    switch line {
    case .horizontal: horizontalImageCount
    case .upward: upwardImageCount
    case .downward: downwardImageCount
    }
  }

  func targetPitchDegrees(for line: PanoramaScanLine) -> Double {
    switch line {
    case .horizontal: horizontalPitchDegrees
    case .upward: upwardPitchDegrees
    case .downward: downwardPitchDegrees
    }
  }

  func yawStepDegrees(for line: PanoramaScanLine) -> Double {
    let count = imageCount(for: line)
    guard count > 0 else { return scanRangeDegrees }
    return scanRangeDegrees / Double(count)
  }

  func targetYawOffsetDegrees(for line: PanoramaScanLine, index: Int) -> Double {
    Double(index) * yawStepDegrees(for: line)
  }

  /// How far past a target the sweep may travel before that angle is given up
  /// on. Without this the pass waits forever on an angle the user has already
  /// turned past while a gate was closed.
  func missedTargetSlackDegrees(for line: PanoramaScanLine) -> Double {
    yawStepDegrees(for: line)
  }

  /// A sweep that lost some angles is still worth keeping, but a mostly empty
  /// one is not.
  var minimumUsableFrameCount: Int {
    max(Int((Double(totalImageCount) * 0.6).rounded()), 1)
  }

  /// Warning UI and capture blocking use this same pitch gate.
  func isPitchErrorBlockingCapture(_ pitchErrorDegrees: Double) -> Bool {
    abs(pitchErrorDegrees) > maxPitchErrorForCaptureDegrees
  }

  /// Warning UI and capture blocking use this same screen-plane roll gate.
  func isRollBlockingCapture(_ rollDegrees: Double) -> Bool {
    abs(rollDegrees) > maxRollForCaptureDegrees
  }

  /// Advisory only: large translation is recorded as a quality note so the
  /// dataset carries the parallax warning, but it never blocks a photo.
  func isTranslationExcessive(_ translationMeters: Double) -> Bool {
    translationMeters > maxTranslationWarningMeters
  }

  func isRotationRateBlockingCapture(_ rotationRateMagnitude: Double) -> Bool {
    rotationRateMagnitude > maxRotationRateRadiansPerSecond
  }
}
