import Foundation

actor CapturePackageStore {
  private let fileManager: FileManager
  private let captureSessionsRootURL: URL?
  private var packageDirectory: URL?
  private var manifest: CaptureSessionManifest?

  init(
    fileManager: FileManager = .default,
    captureSessionsRootURL: URL? = nil
  ) {
    self.fileManager = fileManager
    self.captureSessionsRootURL = captureSessionsRootURL
  }

  func begin(plan: CapturePlan, coreMotionReferenceFrame: String) throws -> CapturePackage {
    let root: URL
    if let captureSessionsRootURL {
      root = captureSessionsRootURL
    } else {
      root = try fileManager.url(
        for: .applicationSupportDirectory,
        in: .userDomainMask,
        appropriateFor: nil,
        create: true
      )
      .appendingPathComponent("CaptureSessions", isDirectory: true)
    }

    let sessionID = UUID()
    let directory = root.appendingPathComponent(sessionID.uuidString, isDirectory: true)
    let imageDirectory = directory.appendingPathComponent("images", isDirectory: true)
    try fileManager.createDirectory(at: imageDirectory, withIntermediateDirectories: true)

    let manifest = CaptureSessionManifest(
      schemaVersion: 5,
      sessionID: sessionID,
      timestampConvention: "UTC seconds since Unix epoch; sub-millisecond precision preserved",
      createdAt: Date(),
      completedAt: nil,
      plan: plan,
      imageDirectory: "images",
      coreMotionReferenceFrame: coreMotionReferenceFrame,
      engineInitialization: EngineInitializationMetadata(
        placementSource: "recorded-device-pose",
        rotationField: "frames[].pose.cameraToCaptureReferenceRotationMatrix",
        usePosePriors: true,
        allowGlobalArrangementRediscovery: false,
        maximumPoseRefinementDegrees: plan.configuration.maximumPoseRefinementDegrees,
        refinementPurpose:
          "Correct bounded CoreMotion sensor error without replacing the recorded arrangement.",
        enabledPipelineStages: [
          "feature-matching",
          "edge-alignment",
          "bounded-pose-refinement",
          "seam-optimization",
          "exposure-correction",
          "blending",
        ]
      ),
      frames: []
    )

    packageDirectory = directory
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
    return CapturePackage(
      directoryURL: directory,
      manifestURL: directory.appendingPathComponent("manifest.json"),
      manifest: manifest
    )
  }

  func append(
    photo: CapturedPhoto,
    target: CaptureTarget,
    pose: CameraPoseMetadata,
    alignment: AlignmentMetadata
  ) throws -> CapturedFrameRecord {
    guard let directory = packageDirectory, var manifest else {
      throw CapturePackageError.sessionNotStarted
    }
    guard !manifest.frames.contains(where: { $0.target.id == target.id }) else {
      throw CapturePackageError.duplicateTarget(target.id)
    }

    let captureSequenceIndex = manifest.frames.count

    let filename = String(
      format: "%03d_%@_%02d.jpg",
      captureSequenceIndex,
      target.ring.rawValue,
      target.ringIndex
    )
    let imageURL =
      directory
      .appendingPathComponent(manifest.imageDirectory, isDirectory: true)
      .appendingPathComponent(filename)
    try photo.data.write(to: imageURL, options: [.atomic, .completeFileProtection])

    let record = CapturedFrameRecord(
      id: UUID(),
      sequenceIndex: captureSequenceIndex,
      imageFilename: filename,
      captureTimestamp: photo.captureTimestamp,
      target: target,
      pose: pose,
      intrinsics: CameraIntrinsicsMetadata(
        sample: photo.intrinsicsSample, photo: photo.photoMetadata),
      lens: photo.lensMetadata,
      photo: photo.photoMetadata,
      alignment: alignment
    )
    manifest.frames.append(record)
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
    return record
  }

  func finalize() throws -> CapturePackage {
    guard let directory = packageDirectory, var manifest else {
      throw CapturePackageError.sessionNotStarted
    }
    guard manifest.frames.count == manifest.plan.targets.count else {
      throw CapturePackageError.incompleteCapture(
        expected: manifest.plan.targets.count,
        received: manifest.frames.count
      )
    }

    manifest.completedAt = Date()
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
    try write(manifest, named: "sphera-engine-request.json", in: directory)

    return CapturePackage(
      directoryURL: directory,
      manifestURL: directory.appendingPathComponent("manifest.json"),
      manifest: manifest
    )
  }

  private func write<T: Encodable>(_ value: T, named filename: String, in directory: URL) throws {
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes]
    encoder.dateEncodingStrategy = .secondsSince1970
    let data = try encoder.encode(value)
    try data.write(
      to: directory.appendingPathComponent(filename),
      options: [.atomic, .completeFileProtection]
    )
  }
}

enum CapturePackageError: LocalizedError {
  case sessionNotStarted
  case duplicateTarget(String)
  case incompleteCapture(expected: Int, received: Int)

  var errorDescription: String? {
    switch self {
    case .sessionNotStarted:
      "The capture package has not been started."
    case .duplicateTarget(let targetID):
      "The capture target \(targetID) was already saved."
    case .incompleteCapture(let expected, let received):
      "The package has \(received) of \(expected) required frames."
    }
  }
}
