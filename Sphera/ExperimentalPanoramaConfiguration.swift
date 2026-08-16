import Foundation

enum CaptureSessionMode: String, CaseIterable, Codable, Sendable {
  case standard
  case experimentalARKit

  var title: String {
    switch self {
    case .standard: "Points"
    case .experimentalARKit: "ARKit Exp"
    }
  }

  var accessibilityLabel: String {
    switch self {
    case .standard: "Standard point capture"
    case .experimentalARKit: "Experimental ARKit panorama capture"
    }
  }
}

enum PanoramaScanLine: String, Codable, CaseIterable, Sendable {
  case upward
  case horizontal
  case downward

  var displayName: String {
    switch self {
    case .upward: "Upward"
    case .horizontal: "Horizontal"
    case .downward: "Downward"
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
    case .clockwise: "Rotate right"
    case .counterclockwise: "Rotate left"
    case .automatic: "Rotate left or right"
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

  static let `default` = ExperimentalPanoramaConfiguration(
    horizontalImageCount: 16,
    upwardImageCount: 12,
    downwardImageCount: 12,
    scanRangeDegrees: 360,
    horizontalPitchDegrees: 0,
    upwardPitchDegrees: 40,
    downwardPitchDegrees: -40,
    pitchToleranceDegrees: 4,
    pathDriftWarningDegrees: 4,
    maxPitchErrorForCaptureDegrees: 4,
    pitchGuideScaleDegrees: 5,
    rollWarningDegrees: 4,
    maxRollForCaptureDegrees: 4,
    wrongDirectionDegrees: 6,
    directionLockDegrees: 8,
    reverseMotionDegrees: 1.5,
    yawSmoothingAlpha: 0.18,
    captureHysteresisDegrees: 0.75,
    maxTranslationWarningMeters: 0.35,
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

  /// Warning UI and capture blocking use this same pitch gate.
  func isPitchErrorBlockingCapture(_ pitchErrorDegrees: Double) -> Bool {
    abs(pitchErrorDegrees) > maxPitchErrorForCaptureDegrees
  }

  /// Warning UI and capture blocking use this same screen-plane roll gate.
  func isRollBlockingCapture(_ rollDegrees: Double) -> Bool {
    abs(rollDegrees) > maxRollForCaptureDegrees
  }

  func isTranslationBlockingCapture(_ translationMeters: Double) -> Bool {
    translationMeters > maxTranslationWarningMeters
  }

  func isRotationRateBlockingCapture(_ rotationRateMagnitude: Double) -> Bool {
    rotationRateMagnitude > maxRotationRateRadiansPerSecond
  }
}
