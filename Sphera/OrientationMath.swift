import Foundation
import simd

enum OrientationMath {
  /// Display-oriented OpenCV camera axes expressed in the physical device
  /// frame while the app is locked to portrait: x right, y down, z optical-forward.
  private static let deviceFromCamera = simd_quatd(
    angle: .pi,
    axis: SIMD3<Double>(1, 0, 0)
  )

  /// Builds a local panorama frame whose Y axis is gravity-up and whose yaw-zero
  /// direction is the horizontal projection of the camera's initial heading.
  /// This avoids inheriting the phone's initial tilt or roll into the sphere.
  static func makeCaptureReference(from sample: MotionSample) -> CaptureReferenceFrame {
    let interpretation = resolveQuaternionInterpretation(from: sample)
    let deviceFromMotion = deviceFromMotionReference(
      sample,
      interpretation: interpretation
    )
    let motionFromCamera = deviceFromMotion.inverse * deviceFromCamera
    let vertical = SIMD3<Double>(0, 0, 1)
    let cameraForward = simd_act(motionFromCamera, SIMD3<Double>(0, 0, 1))
    let cameraRight = simd_act(motionFromCamera, SIMD3<Double>(1, 0, 0))

    var horizontalForward = rejecting(cameraForward, from: vertical)
    if simd_length_squared(horizontalForward) < 0.000_001 {
      let horizontalRight = normalizedOrFallback(
        rejecting(cameraRight, from: vertical),
        fallback: SIMD3<Double>(1, 0, 0)
      )
      horizontalForward = simd_cross(vertical, horizontalRight)
    }
    horizontalForward = normalizedOrFallback(
      horizontalForward,
      fallback: SIMD3<Double>(0, 1, 0)
    )

    let right = normalizedOrFallback(
      simd_cross(horizontalForward, vertical),
      fallback: SIMD3<Double>(1, 0, 0)
    )
    let motionFromCapture = simd_double3x3(columns: (right, vertical, -horizontalForward))
    let captureFromMotion = motionFromCapture.transpose
    let captureFromMotionQuaternion = normalized(simd_quatd(captureFromMotion))

    return CaptureReferenceFrame(
      monotonicTimestampSeconds: sample.monotonicTimestampSeconds,
      initialDeviceQuaternion: sample.attitudeQuaternion,
      motionQuaternionInterpretation: interpretation,
      motionReferenceToCaptureQuaternion: QuaternionValue(captureFromMotionQuaternion),
      motionReferenceToCaptureRotationMatrix: Matrix3x3Value(captureFromMotion),
      coordinateConvention:
        "right-handed gravity-level local panorama frame; +x right at session heading, +y gravity-up, -z initial horizontal viewing direction"
    )
  }

  static func cameraToCaptureReference(
    sample: MotionSample,
    captureReference: CaptureReferenceFrame
  ) -> simd_quatd {
    let deviceFromMotion = deviceFromMotionReference(
      sample,
      interpretation: captureReference.motionQuaternionInterpretation
    )
    let captureFromMotion = normalized(
      captureReference.motionReferenceToCaptureQuaternion.simdQuaternion
    )
    return normalized(captureFromMotion * deviceFromMotion.inverse * deviceFromCamera)
  }

  static func targetCameraToCaptureReference(_ target: CaptureTarget) -> simd_quatd {
    let yaw = target.yawDegrees.radians
    let pitch = target.pitchDegrees.radians
    let forward = simd_normalize(
      SIMD3<Double>(
        sin(yaw) * cos(pitch),
        sin(pitch),
        -cos(yaw) * cos(pitch)
      )
    )

    let vertical = SIMD3<Double>(0, 1, 0)
    var right = simd_cross(forward, vertical)
    if simd_length_squared(right) < 0.000_001 {
      right = SIMD3<Double>(1, 0, 0)
    } else {
      right = simd_normalize(right)
    }
    let visualUp = simd_normalize(simd_cross(right, forward))
    let down = -visualUp
    return normalized(simd_quatd(simd_double3x3(columns: (right, down, forward))))
  }

  static func navigationReading(
    sample: MotionSample,
    captureReference: CaptureReferenceFrame,
    target: CaptureTarget,
    toleranceDegrees: Double
  ) -> CaptureNavigationReading {
    let currentCamera = cameraToCaptureReference(
      sample: sample,
      captureReference: captureReference
    )
    let targetCamera = targetCameraToCaptureReference(target)
    let targetDirectionInCapture = simd_act(targetCamera, SIMD3<Double>(0, 0, 1))
    let currentDirectionInCapture = simd_act(currentCamera, SIMD3<Double>(0, 0, 1))
    let directionError = acos(
      min(1, max(-1, simd_dot(currentDirectionInCapture, targetDirectionInCapture)))
    ).degrees
    let currentYaw = atan2(
      currentDirectionInCapture.x,
      -currentDirectionInCapture.z
    ).degrees
    let currentPitch = asin(
      min(1, max(-1, currentDirectionInCapture.y))
    ).degrees

    return CaptureNavigationReading(
      directionErrorDegrees: directionError,
      yawErrorDegrees: wrappedDegrees(target.yawDegrees - currentYaw),
      pitchErrorDegrees: target.pitchDegrees - currentPitch,
      isAligned: directionError <= toleranceDegrees
    )
  }

  static func poseMetadata(
    sample: MotionSample,
    captureReference: CaptureReferenceFrame,
    referenceFrameName: String
  ) -> CameraPoseMetadata {
    let cameraRotation = cameraToCaptureReference(
      sample: sample,
      captureReference: captureReference
    )
    return CameraPoseMetadata(
      monotonicTimestampSeconds: sample.monotonicTimestampSeconds,
      coreMotionReferenceFrame: referenceFrameName,
      rawDeviceQuaternion: sample.attitudeQuaternion,
      captureReference: captureReference,
      cameraToCaptureReferenceQuaternion: QuaternionValue(cameraRotation),
      cameraToCaptureReferenceRotationMatrix: Matrix3x3Value(simd_double3x3(cameraRotation)),
      coordinateConvention:
        "right-handed; capture +y is gravity-up; camera +x image-right, +y image-down, +z optical-forward"
    )
  }

  /// Classifies a primary capture into a ring using midpoints between the
  /// horizontal plane and the configured upward/downward ring pitches.
  static func classifyCaptureRing(
    pitchDegrees: Double,
    configuration: CaptureConfiguration
  ) -> CaptureRing {
    let upwardBoundary = configuration.upwardPitchDegrees * 0.5
    let downwardBoundary = configuration.downwardPitchDegrees * 0.5
    if pitchDegrees >= upwardBoundary {
      return .upward
    }
    if pitchDegrees <= downwardBoundary {
      return .downward
    }
    return .horizontal
  }

  /// Current camera pitch in the capture frame (degrees, gravity-up positive).
  static func currentPitchDegrees(
    sample: MotionSample,
    captureReference: CaptureReferenceFrame
  ) -> Double {
    let currentCamera = cameraToCaptureReference(
      sample: sample,
      captureReference: captureReference
    )
    let forward = simd_act(currentCamera, SIMD3<Double>(0, 0, 1))
    return asin(min(1, max(-1, forward.y))).degrees
  }

  /// Picks the closest planned target in `ring` to the current camera pose.
  static func closestTarget(
    in ring: CaptureRing,
    targets: [CaptureTarget],
    sample: MotionSample,
    captureReference: CaptureReferenceFrame
  ) -> CaptureTarget? {
    let candidates = targets.filter { $0.ring == ring }
    guard !candidates.isEmpty else { return nil }
    return candidates.min { lhs, rhs in
      let left = navigationReading(
        sample: sample,
        captureReference: captureReference,
        target: lhs,
        toleranceDegrees: 180
      ).directionErrorDegrees
      let right = navigationReading(
        sample: sample,
        captureReference: captureReference,
        target: rhs,
        toleranceDegrees: 180
      ).directionErrorDegrees
      return left < right
    }
  }

  /// Among remaining targets, returns the nearest by spherical direction error.
  static func nearestTarget(
    among targets: [CaptureTarget],
    sample: MotionSample,
    captureReference: CaptureReferenceFrame,
    toleranceDegrees: Double
  ) -> (target: CaptureTarget, reading: CaptureNavigationReading)? {
    guard !targets.isEmpty else { return nil }
    var best: (CaptureTarget, CaptureNavigationReading)?
    for target in targets {
      let reading = navigationReading(
        sample: sample,
        captureReference: captureReference,
        target: target,
        toleranceDegrees: toleranceDegrees
      )
      if let current = best {
        if reading.directionErrorDegrees < current.1.directionErrorDegrees {
          best = (target, reading)
        }
      } else {
        best = (target, reading)
      }
    }
    return best.map { (target: $0.0, reading: $0.1) }
  }

  /// Projects a target direction into normalized screen offsets from the optical
  /// center. `x`/`y` are approximately radians of visual angle (positive x = right,
  /// positive y = down). `isInFront` is false when the target is behind the camera.
  /// `aimingAngleRadians` gives the screen angle pointing toward the target regardless of
  /// whether the target is in front or behind the optical axis.
  static func projectTargetToScreen(
    sample: MotionSample,
    captureReference: CaptureReferenceFrame,
    target: CaptureTarget
  ) -> CapturePointProjection {
    let currentCamera = cameraToCaptureReference(
      sample: sample,
      captureReference: captureReference
    )
    let targetCamera = targetCameraToCaptureReference(target)
    let targetDirectionInCapture = simd_act(targetCamera, SIMD3<Double>(0, 0, 1))
    let directionInCamera = simd_act(
      currentCamera.inverse,
      targetDirectionInCapture
    )
    let forward = directionInCamera.z
    let isInFront = forward > 0.02
    let safeForward = max(forward, 0.02)
    let offsetX = directionInCamera.x / safeForward
    let offsetY = directionInCamera.y / safeForward

    let planarLength = hypot(directionInCamera.x, directionInCamera.y)
    let aimingAngle: Double
    if planarLength > 0.001 {
      aimingAngle = atan2(directionInCamera.y, directionInCamera.x)
    } else if forward < 0 {
      let currentPitch = asin(min(1, max(-1, simd_act(currentCamera, SIMD3<Double>(0, 0, 1)).y))).degrees
      if target.pitchDegrees > currentPitch + 1 {
        aimingAngle = -.pi / 2
      } else if target.pitchDegrees < currentPitch - 1 {
        aimingAngle = .pi / 2
      } else {
        aimingAngle = 0
      }
    } else {
      aimingAngle = 0
    }

    let reading = navigationReading(
      sample: sample,
      captureReference: captureReference,
      target: target,
      toleranceDegrees: 180
    )
    return CapturePointProjection(
      targetID: target.id,
      ring: target.ring,
      offsetX: offsetX,
      offsetY: offsetY,
      aimingAngleRadians: aimingAngle,
      directionErrorDegrees: reading.directionErrorDegrees,
      isInFront: isInFront,
      isAligned: false
    )
  }

  private static func rejecting(
    _ vector: SIMD3<Double>,
    from axis: SIMD3<Double>
  ) -> SIMD3<Double> {
    vector - axis * simd_dot(vector, axis)
  }

  private static func normalizedOrFallback(
    _ vector: SIMD3<Double>,
    fallback: SIMD3<Double>
  ) -> SIMD3<Double> {
    guard simd_length_squared(vector) > 0.000_001 else { return fallback }
    return simd_normalize(vector)
  }

  private static func normalized(_ quaternion: simd_quatd) -> simd_quatd {
    simd_normalize(quaternion)
  }

  /// CoreMotion exposes gravity in device coordinates. Resolve the attitude
  /// quaternion's mapping once at session start, then keep that interpretation
  /// fixed. Re-deciding on every sample can flip the entire coordinate system
  /// while the user moves through an ambiguous attitude.
  private static func resolveQuaternionInterpretation(
    from sample: MotionSample
  ) -> MotionQuaternionInterpretation {
    let raw = normalized(sample.attitudeQuaternion.simdQuaternion)
    let gravity = SIMD3<Double>(sample.gravity.x, sample.gravity.y, sample.gravity.z)
    guard simd_length_squared(gravity) > 0.25 else { return .rawReferenceToDevice }

    let measuredUpInDevice = -simd_normalize(gravity)
    let verticalInMotionReference = SIMD3<Double>(0, 0, 1)
    let rawAgreement = simd_dot(
      simd_act(raw, verticalInMotionReference),
      measuredUpInDevice
    )
    let inverseAgreement = simd_dot(
      simd_act(raw.inverse, verticalInMotionReference),
      measuredUpInDevice
    )
    return rawAgreement >= inverseAgreement
      ? .rawReferenceToDevice
      : .inverseReferenceToDevice
  }

  private static func deviceFromMotionReference(
    _ sample: MotionSample,
    interpretation: MotionQuaternionInterpretation
  ) -> simd_quatd {
    let raw = normalized(sample.attitudeQuaternion.simdQuaternion)
    switch interpretation {
    case .rawReferenceToDevice:
      return raw
    case .inverseReferenceToDevice:
      return raw.inverse
    }
  }

  private static func wrappedDegrees(_ value: Double) -> Double {
    var wrapped = value.truncatingRemainder(dividingBy: 360)
    if wrapped > 180 { wrapped -= 360 }
    if wrapped <= -180 { wrapped += 360 }
    return wrapped
  }
}

struct CaptureNavigationReading: Equatable, Sendable {
  let directionErrorDegrees: Double
  /// Positive means turn clockwise around gravity when viewed from above.
  let yawErrorDegrees: Double
  /// Positive means tilt the optical axis upward against gravity.
  let pitchErrorDegrees: Double
  let isAligned: Bool

  static let unavailable = CaptureNavigationReading(
    directionErrorDegrees: 180,
    yawErrorDegrees: 0,
    pitchErrorDegrees: 0,
    isAligned: false
  )
}

struct CapturePointProjection: Equatable, Identifiable, Sendable {
  var id: String { targetID }
  let targetID: String
  let ring: CaptureRing
  /// Approximate radians of visual angle from optical center; +x right, +y down.
  let offsetX: Double
  let offsetY: Double
  /// Screen angle in radians pointing toward target (+x right, +y down).
  let aimingAngleRadians: Double
  let directionErrorDegrees: Double
  let isInFront: Bool
  let isAligned: Bool
}

extension QuaternionValue {
  init(_ quaternion: simd_quatd) {
    self.init(
      w: quaternion.real,
      x: quaternion.imag.x,
      y: quaternion.imag.y,
      z: quaternion.imag.z
    )
  }

  var simdQuaternion: simd_quatd {
    simd_quatd(ix: x, iy: y, iz: z, r: w)
  }
}

extension Matrix3x3Value {
  init(_ matrix: simd_double3x3) {
    self.init(
      values: [
        matrix[0, 0], matrix[1, 0], matrix[2, 0],
        matrix[0, 1], matrix[1, 1], matrix[2, 1],
        matrix[0, 2], matrix[1, 2], matrix[2, 2],
      ]
    )
  }
}

extension Double {
  fileprivate var radians: Double { self * .pi / 180 }
  fileprivate var degrees: Double { self * 180 / .pi }
}
