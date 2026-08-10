import Foundation

enum CaptureMovement: Equatable, Sendable {
  case preparing
  case turnLeft
  case turnRight
  case tiltUp
  case tiltDown
  case holdStill
  case capturing
}

struct CaptureNavigationInstruction: Equatable, Sendable {
  let movement: CaptureMovement
  let holdProgress: Double

  static let preparing = CaptureNavigationInstruction(
    movement: .preparing,
    holdProgress: 0
  )

  static let capturing = CaptureNavigationInstruction(
    movement: .capturing,
    holdProgress: 1
  )
}

/// Produces one unambiguous, gravity-referenced instruction for a target that
/// stays locked until capture. Correction-axis hysteresis prevents the UI from
/// alternating between TURN and TILT around a threshold.
struct CaptureGuidanceState: Sendable {
  private enum CorrectionAxis: Sendable {
    case yaw
    case pitch
  }

  private var activeTargetID: String?
  private var correctionAxis: CorrectionAxis?

  mutating func reset() {
    activeTargetID = nil
    correctionAxis = nil
  }

  mutating func update(
    targetID: String?,
    reading: CaptureNavigationReading,
    toleranceDegrees: Double,
    stableHoldProgress: Double,
    isReadingAvailable: Bool,
    isCapturingPhoto: Bool
  ) -> CaptureNavigationInstruction {
    if targetID != activeTargetID {
      activeTargetID = targetID
      correctionAxis = nil
    }

    if isCapturingPhoto {
      return .capturing
    }
    guard targetID != nil, isReadingAvailable else {
      correctionAxis = nil
      return .preparing
    }
    if reading.isAligned {
      correctionAxis = nil
      return CaptureNavigationInstruction(
        movement: .holdStill,
        holdProgress: min(1, max(0, stableHoldProgress))
      )
    }

    // Enter an axis correction only outside the wide threshold, then remain on
    // that axis until it reaches the tighter exit threshold. This is the key
    // anti-flicker behavior missing from the previous guide.
    let correctionEntryDegrees = max(8, toleranceDegrees * 0.75)
    let correctionExitDegrees = max(3, toleranceDegrees * 0.35)

    if correctionAxis == .yaw {
      if abs(reading.yawErrorDegrees) > correctionExitDegrees {
        return yawInstruction(for: reading.yawErrorDegrees)
      }
      correctionAxis = nil
    } else if correctionAxis == .pitch {
      if abs(reading.pitchErrorDegrees) > correctionExitDegrees {
        return pitchInstruction(for: reading.pitchErrorDegrees)
      }
      correctionAxis = nil
    }

    if abs(reading.yawErrorDegrees) > correctionEntryDegrees {
      correctionAxis = .yaw
      return yawInstruction(for: reading.yawErrorDegrees)
    }

    if abs(reading.pitchErrorDegrees) > correctionExitDegrees {
      correctionAxis = .pitch
      return pitchInstruction(for: reading.pitchErrorDegrees)
    }

    // A combined spherical error can be just outside tolerance while neither
    // component reaches the entry threshold. Correct the larger component.
    if abs(reading.yawErrorDegrees) >= abs(reading.pitchErrorDegrees) {
      correctionAxis = .yaw
      return yawInstruction(for: reading.yawErrorDegrees)
    }
    correctionAxis = .pitch
    return pitchInstruction(for: reading.pitchErrorDegrees)
  }

  private func yawInstruction(for errorDegrees: Double) -> CaptureNavigationInstruction {
    CaptureNavigationInstruction(
      movement: errorDegrees >= 0 ? .turnRight : .turnLeft,
      holdProgress: 0
    )
  }

  private func pitchInstruction(for errorDegrees: Double) -> CaptureNavigationInstruction {
    CaptureNavigationInstruction(
      movement: errorDegrees >= 0 ? .tiltUp : .tiltDown,
      holdProgress: 0
    )
  }
}
