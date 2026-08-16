import Foundation
import simd

enum ExperimentalPoseMath {
  /// Camera look direction in world space. ARKit cameras look along local -Z.
  static func lookDirection(cameraToWorld transform: simd_float4x4) -> SIMD3<Float> {
    let cameraZ = SIMD3<Float>(
      transform.columns.2.x,
      transform.columns.2.y,
      transform.columns.2.z
    )
    let look = -cameraZ
    let length = simd_length(look)
    guard length > 1e-6 else { return SIMD3<Float>(0, 0, -1) }
    return look / length
  }

  static func position(cameraToWorld transform: simd_float4x4) -> SIMD3<Float> {
    SIMD3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
  }

  /// Yaw is heading around world +Y (gravity up with `.gravity` alignment).
  /// Zero yaw looks along world -Z. Clockwise when viewed from above increases yaw.
  static func yawPitchDegrees(cameraToWorld transform: simd_float4x4) -> (
    yaw: Double, pitch: Double
  ) {
    let look = lookDirection(cameraToWorld: transform)
    let yaw = atan2(Double(look.x), Double(-look.z)) * 180 / .pi
    let horizontal = hypot(Double(look.x), Double(look.z))
    let pitch = atan2(Double(look.y), horizontal) * 180 / .pi
    return (yaw, pitch)
  }

  static func orientation(cameraToWorld transform: simd_float4x4) -> simd_quatf {
    let rotation = simd_float3x3(
      SIMD3(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z),
      SIMD3(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z),
      SIMD3(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
    )
    return simd_quatf(rotation)
  }

  static func relativeTransform(
    from start: simd_float4x4,
    to current: simd_float4x4
  ) -> simd_float4x4 {
    simd_inverse(start) * current
  }

  static func shortestDeltaDegrees(from current: Double, to next: Double) -> Double {
    var delta = (next - current).truncatingRemainder(dividingBy: 360)
    if delta > 180 { delta -= 360 }
    if delta < -180 { delta += 360 }
    return delta
  }

  static func eulerDegrees(cameraToWorld transform: simd_float4x4) -> Vector3Value {
    let look = lookDirection(cameraToWorld: transform)
    let yawPitch = yawPitchDegrees(cameraToWorld: transform)
    let worldUp = SIMD3<Float>(0, 1, 0)
    let cameraX = SIMD3<Float>(
      transform.columns.0.x,
      transform.columns.0.y,
      transform.columns.0.z
    )
    let projectedUp = worldUp - look * simd_dot(worldUp, look)
    let upLength = simd_length(projectedUp)
    let roll: Double
    if upLength < 1e-5 {
      roll = 0
    } else {
      let desiredRight = simd_normalize(simd_cross(look, projectedUp / upLength))
      let right = simd_normalize(cameraX)
      let sinRoll = simd_dot(simd_cross(desiredRight, right), look)
      let cosRoll = simd_dot(desiredRight, right)
      roll = atan2(Double(sinRoll), Double(cosRoll)) * 180 / .pi
    }
    return Vector3Value(x: yawPitch.yaw, y: yawPitch.pitch, z: roll)
  }

  /// SwiftUI Y offset for the path arrow. Positive Y is down, so aiming
  /// below the intended line produces a positive offset.
  static func arrowScreenYOffset(
    pitchErrorDegrees: Double,
    scaleDegrees: Double = 6,
    maxOffset: Double = 56
  ) -> Double {
    let scale = max(scaleDegrees, 0.1)
    let normalized = max(-1, min(1, -pitchErrorDegrees / scale))
    return normalized * maxOffset
  }

  /// Back-camera elevation from CoreMotion gravity in device coordinates.
  /// Device: +X right, +Y top, +Z out of the screen. The camera looks along -Z.
  /// Zero is the horizon (phone upright). Positive is looking up.
  static func cameraElevationDegrees(gravityDevice: Vector3Value) -> Double {
    let length = hypot(hypot(gravityDevice.x, gravityDevice.y), gravityDevice.z)
    guard length > 0.2 else { return 0 }
    let nx = gravityDevice.x / length
    let ny = gravityDevice.y / length
    let nz = gravityDevice.z / length
    return atan2(nz, hypot(nx, ny)) * 180 / .pi
  }

  /// True when gravity has a usable component in the screen plane. Roll is
  /// ill-defined if the phone is nearly flat on its back or face.
  static func isScreenPlaneRollDefined(
    gravityDevice: Vector3Value,
    minimumScreenGravity: Double = 0.15
  ) -> Bool {
    hypot(gravityDevice.x, gravityDevice.y) >= minimumScreenGravity
  }

  /// Rotation in the plane of the screen. Zero is upright portrait.
  /// Positive roll is a clockwise steering-wheel turn: the top of the phone
  /// tilts to the right.
  static func screenPlaneRollDegrees(gravityDevice: Vector3Value) -> Double {
    guard isScreenPlaneRollDefined(gravityDevice: gravityDevice) else { return 0 }
    return atan2(gravityDevice.x, -gravityDevice.y) * 180 / .pi
  }

  /// Gravity (down) in ARKit camera axes. World +Y is up with `.gravity`.
  static func gravityInCameraSpace(cameraToWorld transform: simd_float4x4) -> Vector3Value {
    let cameraX = SIMD3<Float>(
      transform.columns.0.x, transform.columns.0.y, transform.columns.0.z
    )
    let cameraY = SIMD3<Float>(
      transform.columns.1.x, transform.columns.1.y, transform.columns.1.z
    )
    let cameraZ = SIMD3<Float>(
      transform.columns.2.x, transform.columns.2.y, transform.columns.2.z
    )
    let worldDown = SIMD3<Float>(0, -1, 0)
    return Vector3Value(
      x: Double(simd_dot(cameraX, worldDown)),
      y: Double(simd_dot(cameraY, worldDown)),
      z: Double(simd_dot(cameraZ, worldDown))
    )
  }

  /// Landscape camera → portrait device, 90° clockwise display.
  /// Camera +X (image right) becomes device -Y (toward the bottom).
  static func deviceGravityPortraitClockwise(gravityCamera: Vector3Value) -> Vector3Value {
    Vector3Value(x: gravityCamera.y, y: -gravityCamera.x, z: gravityCamera.z)
  }

  /// Landscape camera → portrait device, 90° counter-clockwise display.
  /// Camera +X (image right) becomes device +Y (toward the top).
  static func deviceGravityPortraitCounterclockwise(gravityCamera: Vector3Value) -> Vector3Value {
    Vector3Value(x: -gravityCamera.y, y: gravityCamera.x, z: gravityCamera.z)
  }

  static func screenPlaneRollDegrees(
    cameraToWorld transform: simd_float4x4,
    portraitRotationClockwise: Bool
  ) -> Double {
    let gravityCamera = gravityInCameraSpace(cameraToWorld: transform)
    let gravityDevice = portraitRotationClockwise
      ? deviceGravityPortraitClockwise(gravityCamera: gravityCamera)
      : deviceGravityPortraitCounterclockwise(gravityCamera: gravityCamera)
    return screenPlaneRollDegrees(gravityDevice: gravityDevice)
  }

  static func screenPlaneRollInstruction(rollDegrees: Double) -> String {
    if rollDegrees > 0.5 {
      return "Rotate the phone left until it's upright"
    }
    if rollDegrees < -0.5 {
      return "Rotate the phone right until it's upright"
    }
    return "Keep the phone upright"
  }
}

/// Phone attitude used to gate experimental capture. Pitch prefers whichever
/// of ARKit look-direction and CoreMotion gravity is farther from the scan
/// line, so pointing at the sky cannot hide behind one noisy sensor.
struct ExperimentalAttitudeReading: Equatable, Sendable {
  var pitchDegrees: Double
  var rollDegrees: Double
  var pitchErrorDegrees: Double
  var isAbovePath: Bool
  var isBelowPath: Bool
  var isRolled: Bool
  var isPitchBlockingCapture: Bool
  var isRollBlockingCapture: Bool
  var rollCorrectionInstruction: String
  var pitchCorrectionInstruction: String

  var shouldBlockCapture: Bool {
    isPitchBlockingCapture || isRollBlockingCapture
  }

  var primaryWarning: String? {
    if isRollBlockingCapture {
      return "Level the phone"
    }
    if isAbovePath {
      return "Lower the phone"
    }
    if isBelowPath {
      return "Raise the phone"
    }
    return nil
  }

  var blockReason: String? {
    if isRollBlockingCapture {
      return rollCorrectionInstruction
    }
    if isPitchBlockingCapture {
      return pitchCorrectionInstruction
    }
    return nil
  }

  static func make(
    gravity: Vector3Value?,
    arkitPitchDegrees: Double,
    arkitRollDegrees: Double = 0,
    targetPitchDegrees: Double,
    configuration: ExperimentalPanoramaConfiguration
  ) -> ExperimentalAttitudeReading {
    let gravityPitch = gravity.map {
      ExperimentalPoseMath.cameraElevationDegrees(gravityDevice: $0)
    }
    let pitch: Double
    if let gravityPitch {
      pitch =
        abs(gravityPitch - targetPitchDegrees) >= abs(arkitPitchDegrees - targetPitchDegrees)
        ? gravityPitch
        : arkitPitchDegrees
    } else {
      pitch = arkitPitchDegrees
    }

    let roll: Double
    if let gravity, ExperimentalPoseMath.isScreenPlaneRollDefined(gravityDevice: gravity) {
      roll = ExperimentalPoseMath.screenPlaneRollDegrees(gravityDevice: gravity)
    } else {
      roll = arkitRollDegrees
    }

    let pitchError = pitch - targetPitchDegrees
    let isAbove = pitchError > configuration.maxPitchErrorForCaptureDegrees
    let isBelow = pitchError < -configuration.maxPitchErrorForCaptureDegrees
    let isRolled = configuration.isRollBlockingCapture(roll)
    let isPitchBlocking = configuration.isPitchErrorBlockingCapture(pitchError)
    return ExperimentalAttitudeReading(
      pitchDegrees: pitch,
      rollDegrees: roll,
      pitchErrorDegrees: pitchError,
      isAbovePath: isAbove,
      isBelowPath: isBelow,
      isRolled: isRolled,
      isPitchBlockingCapture: isPitchBlocking,
      isRollBlockingCapture: isRolled,
      rollCorrectionInstruction: ExperimentalPoseMath.screenPlaneRollInstruction(
        rollDegrees: roll
      ),
      pitchCorrectionInstruction: pitchError > 0
        ? "Lower the phone onto the line"
        : "Raise the phone onto the line"
    )
  }
}

extension QuaternionValue {
  init(_ quaternion: simd_quatf) {
    let q = simd_normalize(quaternion)
    self.init(w: Double(q.real), x: Double(q.imag.x), y: Double(q.imag.y), z: Double(q.imag.z))
  }
}

extension Vector3Value {
  init(_ value: SIMD3<Float>) {
    self.init(x: Double(value.x), y: Double(value.y), z: Double(value.z))
  }
}

extension Matrix3x3Value {
  /// Row-major values, matching `Matrix3x3Value`'s documented storage.
  init(_ matrix: simd_float3x3) {
    self.init(
      values: [
        Double(matrix.columns.0.x), Double(matrix.columns.1.x), Double(matrix.columns.2.x),
        Double(matrix.columns.0.y), Double(matrix.columns.1.y), Double(matrix.columns.2.y),
        Double(matrix.columns.0.z), Double(matrix.columns.1.z), Double(matrix.columns.2.z),
      ]
    )
  }
}
