import Foundation
import simd

struct Matrix4x4Value: Codable, Equatable, Sendable {
  /// Column-major 16 values, matching `simd_float4x4` / ARKit `camera.transform`.
  let values: [Double]
  let storageOrder: String

  init(_ matrix: simd_float4x4) {
    values = [
      Double(matrix.columns.0.x), Double(matrix.columns.0.y), Double(matrix.columns.0.z),
      Double(matrix.columns.0.w),
      Double(matrix.columns.1.x), Double(matrix.columns.1.y), Double(matrix.columns.1.z),
      Double(matrix.columns.1.w),
      Double(matrix.columns.2.x), Double(matrix.columns.2.y), Double(matrix.columns.2.z),
      Double(matrix.columns.2.w),
      Double(matrix.columns.3.x), Double(matrix.columns.3.y), Double(matrix.columns.3.z),
      Double(matrix.columns.3.w),
    ]
    storageOrder = "column-major"
  }

  init(values: [Double], storageOrder: String = "column-major") {
    self.values = values
    self.storageOrder = storageOrder
  }

  var simdMatrix: simd_float4x4 {
    let v = values
    guard v.count == 16 else { return matrix_identity_float4x4 }
    return simd_float4x4(
      SIMD4(Float(v[0]), Float(v[1]), Float(v[2]), Float(v[3])),
      SIMD4(Float(v[4]), Float(v[5]), Float(v[6]), Float(v[7])),
      SIMD4(Float(v[8]), Float(v[9]), Float(v[10]), Float(v[11])),
      SIMD4(Float(v[12]), Float(v[13]), Float(v[14]), Float(v[15]))
    )
  }
}

enum ARKitTrackingStateRecord: String, Codable, Sendable {
  case notAvailable
  case limitedInitializing
  case limitedExcessiveMotion
  case limitedInsufficientFeatures
  case limitedRelocalizing
  case limitedOther
  case normal

  var isSeverelyLimited: Bool {
    switch self {
    case .notAvailable, .limitedInitializing, .limitedRelocalizing:
      true
    default:
      false
    }
  }

  var displayName: String {
    switch self {
    case .notAvailable: "Unavailable"
    case .limitedInitializing: "Initializing"
    case .limitedExcessiveMotion: "Moving too fast"
    case .limitedInsufficientFeatures: "Low features"
    case .limitedRelocalizing: "Relocalizing"
    case .limitedOther: "Limited"
    case .normal: "Normal"
    }
  }
}

struct ARKitCameraMetadata: Codable, Equatable, Sendable {
  let timestamp: TimeInterval
  let trackingState: ARKitTrackingStateRecord
  let transform: Matrix4x4Value
  let intrinsics: Matrix3x3Value
  let imageResolutionWidth: Int
  let imageResolutionHeight: Int
  let position: Vector3Value
  let orientation: QuaternionValue
  let translationFromSessionStart: Vector3Value
  let rotationFromSessionStart: QuaternionValue
  let relativeTransform: Matrix4x4Value
  let eulerDegrees: Vector3Value
}

struct ExperimentalCapturedFrame: Codable, Identifiable, Equatable, Sendable {
  let id: UUID
  let imageFilename: String
  let captureTimestamp: Date
  let scanLine: PanoramaScanLine
  let indexInLine: Int
  let lineImageCount: Int
  let sequenceIndex: Int
  let targetYawOffsetDegrees: Double
  let actualYawOffsetDegrees: Double
  let actualPitchDegrees: Double
  let arkit: ARKitCameraMetadata
  let motion: MotionSample?
  let photo: PhotoMetadata
  let qualityNotes: [String]
}

struct ExperimentalScanLineSummary: Codable, Equatable, Sendable {
  let scanLine: PanoramaScanLine
  let imageCount: Int
  let capturedCount: Int
  let startedAt: Date?
  let completedAt: Date?
}

struct ExperimentalCaptureManifest: Codable, Equatable, Sendable {
  let schemaVersion: Int
  let kind: String
  let sessionID: UUID
  let timestampConvention: String
  let createdAt: Date
  var completedAt: Date?
  var isComplete: Bool
  var incompleteReason: String?
  let configuration: ExperimentalPanoramaConfiguration
  let imageDirectory: String
  var sessionStartTransform: Matrix4x4Value?
  var sessionStartTimestamp: TimeInterval?
  let coreMotionReferenceFrame: String
  var frames: [ExperimentalCapturedFrame]
  var lineSummaries: [ExperimentalScanLineSummary]

  static let currentSchemaVersion = 1
  static let kindIdentifier = "experimental-arkit-panorama"
  /// ARKit world tracking is gravity-aligned; this is not Core Motion's
  /// `xArbitraryZVertical` frame used by standard capture.
  static let worldTrackingReferenceFrame = "arkitGravity"
}

enum ExperimentalCaptureImage {
  /// `ARFrame.capturedImage` is landscape sensor pixels. Locked portrait
  /// preview and AVCapture photos use EXIF 6 (`CGImagePropertyOrientation.right`).
  static let jpegEXIFOrientation = 6
}

struct ExperimentalCapturePackage: Equatable, Sendable {
  let directoryURL: URL
  let manifestURL: URL
  let manifest: ExperimentalCaptureManifest
}

extension ExperimentalCapturePackage {
  var firstImageURL: URL? {
    guard let filename = manifest.frames.first?.imageFilename else { return nil }
    return imageURL(for: filename)
  }

  var previewImageURL: URL? { firstImageURL }

  func imageURL(for filename: String) -> URL {
    directoryURL
      .appendingPathComponent(manifest.imageDirectory, isDirectory: true)
      .appendingPathComponent(filename)
  }
}

enum GalleryCaptureItem: Identifiable {
  case standard(CapturePackage)
  case experimental(ExperimentalCapturePackage)

  var id: UUID {
    switch self {
    case .standard(let package): package.manifest.sessionID
    case .experimental(let package): package.manifest.sessionID
    }
  }

  var sortDate: Date {
    switch self {
    case .standard(let package):
      package.manifest.completedAt ?? package.manifest.createdAt
    case .experimental(let package):
      package.manifest.completedAt ?? package.manifest.createdAt
    }
  }

  var isExperimental: Bool {
    if case .experimental = self { return true }
    return false
  }
}

struct ExperimentalLivePose: Equatable, Sendable {
  let timestamp: TimeInterval
  let trackingState: ARKitTrackingStateRecord
  let transform: simd_float4x4
  let intrinsics: simd_float3x3
  let imageResolution: SIMD2<Int>
  let yawDegrees: Double
  let pitchDegrees: Double
  let rollDegrees: Double
  let position: SIMD3<Float>
  let rotationRateMagnitude: Double
}

struct ExperimentalGuidanceSnapshot: Equatable, Sendable {
  var activeLine: PanoramaScanLine?
  var nextLine: PanoramaScanLine?
  var isTransitioning: Bool
  var isReadyToStart: Bool
  var lineProgress: Double
  var capturedInLine: Int
  var targetInLine: Int
  var capturedTotal: Int
  var targetTotal: Int
  var pitchErrorDegrees: Double
  var rollDegrees: Double
  var currentYawOffsetDegrees: Double
  var isAbovePath: Bool
  var isBelowPath: Bool
  var isRolled: Bool
  var isPitchBlockingCapture: Bool
  var isRollBlockingCapture: Bool
  var isWrongDirection: Bool
  var isTranslatingTooMuch: Bool
  var isLineComplete: Bool
  var trackingState: ARKitTrackingStateRecord
  var expectedOrientationLabel: String
  var instruction: String
  var warningMessage: String?
  var rotationDirection: ExperimentalCaptureDirection?
  var rollCorrectionInstruction: String
  var pitchGuideScaleDegrees: Double

  var shouldBlockCapture: Bool {
    isPitchBlockingCapture || isRollBlockingCapture
  }

  var chromeCaption: String {
    if let activeLine {
      let remaining = max(targetInLine - capturedInLine, 0)
      return "\(activeLine.displayName) · \(remaining) left"
    }
    return "\(targetTotal) photos · three lines"
  }

  var guideAccentIsBlocking: Bool {
    warningMessage != nil
  }

  static let idle = ExperimentalGuidanceSnapshot(
    activeLine: nil,
    nextLine: .horizontal,
    isTransitioning: false,
    isReadyToStart: true,
    lineProgress: 0,
    capturedInLine: 0,
    targetInLine: ExperimentalPanoramaConfiguration.default.horizontalImageCount,
    capturedTotal: 0,
    targetTotal: ExperimentalPanoramaConfiguration.default.totalImageCount,
    pitchErrorDegrees: 0,
    rollDegrees: 0,
    currentYawOffsetDegrees: 0,
    isAbovePath: false,
    isBelowPath: false,
    isRolled: false,
    isPitchBlockingCapture: false,
    isRollBlockingCapture: false,
    isWrongDirection: false,
    isTranslatingTooMuch: false,
    isLineComplete: false,
    trackingState: .notAvailable,
    expectedOrientationLabel: PanoramaScanLine.horizontal.expectedOrientationLabel,
    instruction: "Tap the shutter, then rotate",
    warningMessage: nil,
    rotationDirection: nil,
    rollCorrectionInstruction: "Keep the phone upright",
    pitchGuideScaleDegrees: ExperimentalPanoramaConfiguration.default.pitchGuideScaleDegrees
  )
}
