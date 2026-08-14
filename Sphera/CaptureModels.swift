import Foundation

struct CameraIntrinsicsSample: Equatable, Sendable {
  let monotonicTimestampSeconds: Double
  let fx: Double
  let fy: Double
  let cx: Double
  let cy: Double
  let referenceWidth: Int
  let referenceHeight: Int
  let captureRotationDegrees: Double
}

struct CapturedPhoto: Sendable {
  let data: Data
  let captureTimestamp: Date
  let motionSample: MotionSample
  let intrinsicsSample: CameraIntrinsicsSample
  let lensMetadata: LensMetadata
  let photoMetadata: PhotoMetadata
}

enum CaptureRing: String, Codable, CaseIterable, Sendable {
  case horizontal
  case downward
  case upward

  var displayName: String {
    switch self {
    case .horizontal: "Horizontal"
    case .downward: "Downward"
    case .upward: "Upward"
    }
  }
}

struct CaptureConfiguration: Codable, Equatable, Sendable {
  var horizontalCount: Int
  var downwardCount: Int
  var upwardCount: Int
  var downwardPitchDegrees: Double
  var upwardPitchDegrees: Double
  var alignmentToleranceDegrees: Double
  var stableHoldDurationSeconds: Double
  var maximumPoseRefinementDegrees: Double

  static let debugPreset = CaptureConfiguration(
    horizontalCount: 8,
    downwardCount: 5,
    upwardCount: 5,
    downwardPitchDegrees: -55,
    upwardPitchDegrees: 55,
    alignmentToleranceDegrees: 6,
    stableHoldDurationSeconds: 0.3,
    maximumPoseRefinementDegrees: 8
  )

  var totalImageCount: Int {
    horizontalCount + downwardCount + upwardCount
  }
}

struct CaptureTarget: Codable, Identifiable, Equatable, Sendable {
  let id: String
  let sequenceIndex: Int
  let ring: CaptureRing
  let ringIndex: Int
  let ringCount: Int
  let yawDegrees: Double
  let pitchDegrees: Double
  let rollDegrees: Double
}

struct CapturePlan: Codable, Equatable, Sendable {
  let configuration: CaptureConfiguration
  let targets: [CaptureTarget]

  init(configuration: CaptureConfiguration) {
    self.configuration = configuration
    var targets: [CaptureTarget] = []

    /// Appends one clockwise ring and returns the heading of its final target.
    /// The next ring starts at that same heading, so changing rings only ever
    /// requires a tilt rather than an arbitrary turn to a different azimuth.
    func appendRing(
      _ ring: CaptureRing,
      count: Int,
      pitch: Double,
      startingYawDegrees: Double
    ) -> Double {
      guard count > 0 else { return startingYawDegrees }
      let yawStep = 360 / Double(count)
      for ringIndex in 0..<count {
        let yaw = Self.normalizedYaw(
          startingYawDegrees + Double(ringIndex) * yawStep
        )
        let sequenceIndex = targets.count
        targets.append(
          CaptureTarget(
            id: "\(ring.rawValue)-\(ringIndex)",
            sequenceIndex: sequenceIndex,
            ring: ring,
            ringIndex: ringIndex,
            ringCount: count,
            yawDegrees: yaw,
            pitchDegrees: pitch,
            rollDegrees: 0
          )
        )
      }
      return Self.normalizedYaw(
        startingYawDegrees + Double(count - 1) * yawStep
      )
    }

    var routeHeading = appendRing(
      .horizontal,
      count: configuration.horizontalCount,
      pitch: 0,
      startingYawDegrees: 0
    )
    routeHeading = appendRing(
      .downward,
      count: configuration.downwardCount,
      pitch: configuration.downwardPitchDegrees,
      startingYawDegrees: routeHeading
    )
    _ = appendRing(
      .upward,
      count: configuration.upwardCount,
      pitch: configuration.upwardPitchDegrees,
      startingYawDegrees: routeHeading
    )
    self.targets = targets
  }

  private static func normalizedYaw(_ value: Double) -> Double {
    let remainder = value.truncatingRemainder(dividingBy: 360)
    return remainder >= 0 ? remainder : remainder + 360
  }
}

struct QuaternionValue: Codable, Equatable, Sendable {
  let w: Double
  let x: Double
  let y: Double
  let z: Double
}

struct Vector3Value: Codable, Equatable, Sendable {
  let x: Double
  let y: Double
  let z: Double
}

struct Matrix3x3Value: Codable, Equatable, Sendable {
  /// Row-major values.
  let values: [Double]
}

struct MotionSample: Codable, Equatable, Sendable {
  let monotonicTimestampSeconds: Double
  let wallClockTimestamp: Date
  let attitudeQuaternion: QuaternionValue
  let gravity: Vector3Value
  let rotationRateRadiansPerSecond: Vector3Value
}

enum MotionQuaternionInterpretation: String, Codable, Equatable, Sendable {
  /// The raw CoreMotion quaternion maps vectors from the motion reference frame
  /// into device coordinates.
  case rawReferenceToDevice
  /// The inverse raw CoreMotion quaternion maps vectors from the motion
  /// reference frame into device coordinates.
  case inverseReferenceToDevice
}

struct CaptureReferenceFrame: Codable, Equatable, Sendable {
  let monotonicTimestampSeconds: Double
  let initialDeviceQuaternion: QuaternionValue
  let motionQuaternionInterpretation: MotionQuaternionInterpretation
  let motionReferenceToCaptureQuaternion: QuaternionValue
  let motionReferenceToCaptureRotationMatrix: Matrix3x3Value
  let coordinateConvention: String
}

struct CameraPoseMetadata: Codable, Equatable, Sendable {
  let monotonicTimestampSeconds: Double
  let coreMotionReferenceFrame: String
  let rawDeviceQuaternion: QuaternionValue
  let captureReference: CaptureReferenceFrame
  let cameraToCaptureReferenceQuaternion: QuaternionValue
  let cameraToCaptureReferenceRotationMatrix: Matrix3x3Value
  let coordinateConvention: String
}

struct CameraIntrinsicsMetadata: Codable, Equatable, Sendable {
  let source: String
  let calibrationPresentationTimestampSeconds: Double
  let fx: Double
  let fy: Double
  let cx: Double
  let cy: Double
  let skew: Double
  let calibrationReferenceWidth: Int
  let calibrationReferenceHeight: Int
  let photoFx: Double
  let photoFy: Double
  let photoCx: Double
  let photoCy: Double
  let encodedPhotoWidth: Int
  let encodedPhotoHeight: Int
  let orientedPhotoWidth: Int
  let orientedPhotoHeight: Int
  let appliedEXIFOrientation: Int
  let photoIntrinsicsCoordinateSpace: String
  let captureRotationDegrees: Double
  let calibrationBufferGeometricDistortionCorrectionApplied: Bool
  let photoScalingAssumption: String
}

extension CameraIntrinsicsMetadata {
  init(sample: CameraIntrinsicsSample, photo: PhotoMetadata) {
    let orientedWidth = photo.orientedPixelWidth
    let orientedHeight = photo.orientedPixelHeight
    let scaleX =
      sample.referenceWidth > 0
      ? Double(orientedWidth) / Double(sample.referenceWidth)
      : 1
    let scaleY =
      sample.referenceHeight > 0
      ? Double(orientedHeight) / Double(sample.referenceHeight)
      : 1

    self.init(
      source: "kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix",
      calibrationPresentationTimestampSeconds: sample.monotonicTimestampSeconds,
      fx: sample.fx,
      fy: sample.fy,
      cx: sample.cx,
      cy: sample.cy,
      skew: 0,
      calibrationReferenceWidth: sample.referenceWidth,
      calibrationReferenceHeight: sample.referenceHeight,
      photoFx: sample.fx * scaleX,
      photoFy: sample.fy * scaleY,
      photoCx: sample.cx * scaleX,
      photoCy: sample.cy * scaleY,
      encodedPhotoWidth: photo.width,
      encodedPhotoHeight: photo.height,
      orientedPhotoWidth: orientedWidth,
      orientedPhotoHeight: orientedHeight,
      appliedEXIFOrientation: photo.exifOrientation ?? 1,
      photoIntrinsicsCoordinateSpace:
        "display-oriented pixels after applying the JPEG EXIF orientation",
      captureRotationDegrees: sample.captureRotationDegrees,
      calibrationBufferGeometricDistortionCorrectionApplied: true,
      photoScalingAssumption:
        "K comes from AVFoundation's physically rotated portrait video calibration buffer. The original JPEG remains encoded in sensor orientation and uses an EXIF orientation tag; K is scaled into the corresponding display-oriented full-resolution pixel space."
    )
  }
}

struct LensMetadata: Codable, Equatable, Sendable {
  let deviceType: String
  let position: String
  let uniqueID: String
  let modelID: String
  let localizedName: String
  let activeFormatDescription: String
  let videoFieldOfViewDegrees: Double
  let minimumZoomFactor: Double
  let maximumZoomFactor: Double
  let zoomFactor: Double
  let lensPosition: Double
  let maximumPhotoWidth: Int
  let maximumPhotoHeight: Int
}

struct PhotoMetadata: Codable, Equatable, Sendable {
  let codec: String
  let width: Int
  let height: Int
  let exifOrientation: Int?
  let exposureDurationSeconds: Double?
  let iso: Double?
  let aperture: Double?
  let focalLengthMillimeters: Double?
  let focalLength35mmEquivalent: Double?
  let brightnessValue: Double?
  let exposureBiasValue: Double?
  let lensMake: String?
  let lensModel: String?
}

extension PhotoMetadata {
  var orientedPixelWidth: Int {
    exifOrientationSwapsPixelAxes ? height : width
  }

  var orientedPixelHeight: Int {
    exifOrientationSwapsPixelAxes ? width : height
  }

  private var exifOrientationSwapsPixelAxes: Bool {
    switch exifOrientation {
    case 5, 6, 7, 8: true
    default: false
    }
  }
}

struct AlignmentMetadata: Codable, Equatable, Sendable {
  let directionErrorDegrees: Double
  let yawErrorDegrees: Double
  let pitchErrorDegrees: Double
  let requiredToleranceDegrees: Double
  let requiredStableDurationSeconds: Double
}

struct CapturedFrameRecord: Codable, Identifiable, Equatable, Sendable {
  let id: UUID
  let sequenceIndex: Int
  let imageFilename: String
  let captureTimestamp: Date
  let target: CaptureTarget
  let pose: CameraPoseMetadata
  let intrinsics: CameraIntrinsicsMetadata
  let lens: LensMetadata
  let photo: PhotoMetadata
  let alignment: AlignmentMetadata
}

struct EngineInitializationMetadata: Codable, Equatable, Sendable {
  /// `"recorded"` for sensor-first placement; older manifests may still say `"estimate"`.
  let placementSource: String
  let rotationField: String
  let usePosePriors: Bool
  /// Always `false` on the sensor-first product path.
  let allowGlobalArrangementRediscovery: Bool
  /// Soft cap for SIFT sensor-anchored correction degrees (`6` on-device).
  let maximumPoseRefinementDegrees: Double?
  let refinementPurpose: String
  let enabledPipelineStages: [String]
}

struct PrimaryCaptureMetadata: Codable, Equatable, Sendable {
  let imageFilename: String
  let targetId: String
  let classifiedRing: CaptureRing
}

struct CaptureSessionManifest: Codable, Equatable, Sendable {
  let schemaVersion: Int
  let sessionID: UUID
  let timestampConvention: String
  let createdAt: Date
  var completedAt: Date?
  let plan: CapturePlan
  let imageDirectory: String
  let coreMotionReferenceFrame: String
  let engineInitialization: EngineInitializationMetadata
  /// Present on schema 6+ sessions that used a user-triggered primary capture.
  /// Absent when decoding older packages for backward compatibility.
  var primaryCapture: PrimaryCaptureMetadata?
  var frames: [CapturedFrameRecord]
}

struct CapturePackage: Equatable, Sendable {
  let directoryURL: URL
  let manifestURL: URL
  let manifest: CaptureSessionManifest
}

struct StitchingResult: Equatable, Sendable {
  let panoramaURL: URL
  let reportURL: URL?
  /// Algorithm runtime from the experimental Swift/Metal engine, when known.
  let elapsedSeconds: Double?

  init(panoramaURL: URL, reportURL: URL?, elapsedSeconds: Double? = nil) {
    self.panoramaURL = panoramaURL
    self.reportURL = reportURL
    self.elapsedSeconds = elapsedSeconds
  }
}
