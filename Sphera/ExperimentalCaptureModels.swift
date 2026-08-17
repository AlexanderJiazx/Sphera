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

  /// Plain-language explanation for the viewfinder. `nil` means nothing worth
  /// interrupting the user about.
  var userAdvice: String? {
    switch self {
    case .normal, .limitedOther: nil
    case .notAvailable: "Waiting for motion tracking"
    case .limitedInitializing: "Move iPhone slowly to start tracking"
    case .limitedExcessiveMotion: "Slow down"
    case .limitedInsufficientFeatures: "Point at something with more detail"
    case .limitedRelocalizing: "Return to where you started"
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

extension ExperimentalCapturedFrame {
  /// Frames are renumbered when a row is discarded and re-shot, so the
  /// sequence stays contiguous in the exported manifest.
  func withSequenceIndex(_ index: Int) -> ExperimentalCapturedFrame {
    ExperimentalCapturedFrame(
      id: id,
      imageFilename: imageFilename,
      captureTimestamp: captureTimestamp,
      scanLine: scanLine,
      indexInLine: indexInLine,
      lineImageCount: lineImageCount,
      sequenceIndex: index,
      targetYawOffsetDegrees: targetYawOffsetDegrees,
      actualYawOffsetDegrees: actualYawOffsetDegrees,
      actualPitchDegrees: actualPitchDegrees,
      arkit: arkit,
      motion: motion,
      photo: photo,
      qualityNotes: qualityNotes
    )
  }
}

/// A planned angle the sweep passed without a usable photo. Recorded so a
/// consumer of the dataset can tell a gap from a truncated session.
struct ExperimentalSkippedTarget: Codable, Equatable, Sendable {
  let scanLine: PanoramaScanLine
  let indexInLine: Int
  let targetYawOffsetDegrees: Double
}

struct ExperimentalScanLineSummary: Codable, Equatable, Sendable {
  let scanLine: PanoramaScanLine
  let imageCount: Int
  let capturedCount: Int
  var skippedCount: Int?
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
  var skippedTargets: [ExperimentalSkippedTarget]?

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

extension ExperimentalCapturedFrame {
  func imageURL(inPackageDirectory directory: URL) -> URL {
    directory
      .appendingPathComponent("images", isDirectory: true)
      .appendingPathComponent(imageFilename)
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

// MARK: - Guidance

/// What the sweep is doing right now. Every visible piece of chrome is derived
/// from this, so the interface can never disagree with the capture engine.
enum ExperimentalCaptureStage: String, Equatable, Sendable {
  /// ARKit is warming up; there is nothing the user can do yet.
  case starting
  /// Tracking is live and the shutter is armed.
  case ready
  /// A row is queued and the user needs to get the phone onto its guide line.
  case aligning
  /// Photos are being taken as the user turns.
  case scanning
  /// The row just finished; a short confirmation before the next one.
  case rowComplete
  /// Writing the session to disk.
  case finishing
  /// Recoverable stop: backgrounded, a call arrived, tracking dropped out.
  case paused
  /// Unrecoverable; the failure screen takes over.
  case unavailable

  var isSweepInProgress: Bool {
    switch self {
    case .aligning, .scanning, .rowComplete: true
    case .starting, .ready, .finishing, .paused, .unavailable: false
    }
  }

  /// Whether the level guide and coverage track belong on screen.
  var showsSweepGuides: Bool {
    switch self {
    case .aligning, .scanning, .rowComplete: true
    case .starting, .ready, .finishing, .paused, .unavailable: false
    }
  }
}

struct ExperimentalGuidanceSnapshot: Equatable, Sendable {
  var stage: ExperimentalCaptureStage
  var line: PanoramaScanLine
  var passIndex: Int
  var passCount: Int
  var capturedInLine: Int
  var skippedInLine: Int
  var targetInLine: Int
  var capturedTotal: Int
  var targetTotal: Int
  var targetStates: [ExperimentalTargetState]
  var sweepFraction: Double
  var directedYawOffsetDegrees: Double
  var scanRangeDegrees: Double
  var pitchErrorDegrees: Double
  var rollDegrees: Double
  var pitchGuideScaleDegrees: Double
  var isLevelForCapture: Bool
  var alignmentHoldFraction: Double
  var blockReason: ExperimentalCaptureBlockReason?
  var trackingState: ARKitTrackingStateRecord
  var rotationDirection: ExperimentalCaptureDirection?
  var title: String
  var subtitle: String?

  /// Short heads-up shown over the viewfinder while something needs fixing.
  var alert: String? {
    blockReason?.badge
  }

  var isBlocked: Bool { blockReason != nil }

  var passCaption: String {
    "Pass \(passIndex) of \(passCount) · \(line.rowName)"
  }

  var photoCaption: String {
    "\(capturedInLine) of \(targetInLine) photos"
  }

  var totalProgress: Double {
    guard targetTotal > 0 else { return 0 }
    return min(1, Double(capturedTotal) / Double(targetTotal))
  }

  static let idle = ExperimentalGuidanceSnapshot(
    stage: .starting,
    line: .horizontal,
    passIndex: 1,
    passCount: ExperimentalPanoramaConfiguration.default.scanLineOrder.count,
    capturedInLine: 0,
    skippedInLine: 0,
    targetInLine: ExperimentalPanoramaConfiguration.default.horizontalImageCount,
    capturedTotal: 0,
    targetTotal: ExperimentalPanoramaConfiguration.default.totalImageCount,
    targetStates: [],
    sweepFraction: 0,
    directedYawOffsetDegrees: 0,
    scanRangeDegrees: ExperimentalPanoramaConfiguration.default.scanRangeDegrees,
    pitchErrorDegrees: 0,
    rollDegrees: 0,
    pitchGuideScaleDegrees: ExperimentalPanoramaConfiguration.default.pitchGuideScaleDegrees,
    isLevelForCapture: false,
    alignmentHoldFraction: 0,
    blockReason: nil,
    trackingState: .notAvailable,
    rotationDirection: nil,
    title: "Starting camera",
    subtitle: nil
  )
}
