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
    let root = try sessionsRootURL()

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
        placementSource: "estimate",
        rotationField: "frames[].pose.cameraToCaptureReferenceRotationMatrix",
        usePosePriors: true,
        allowGlobalArrangementRediscovery: true,
        maximumPoseRefinementDegrees: nil,
        refinementPurpose:
          "Match-based arrangement with locked shared intrinsics and CoreMotion ring pitch prior.",
        enabledPipelineStages: [
          "adaptive-ring-layout",
          "feature-matching",
          "homography-camera-estimate",
          "locked-shared-intrinsics",
          "ray-bundle-adjustment",
          "wave-correct",
          "normalize-world-orientation",
          "coremotion-ring-pitch-prior",
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
    try write(manifest, named: "manifest.json", in: directory)
    try write(manifest, named: "sphera-engine-request.json", in: directory)

    packageDirectory = nil
    self.manifest = nil

    return CapturePackage(
      directoryURL: directory,
      manifestURL: directory.appendingPathComponent("manifest.json"),
      manifest: manifest
    )
  }

  /// Removes an in-progress session directory if capture is cancelled before finalize.
  func abandon() {
    guard let directory = packageDirectory else { return }
    packageDirectory = nil
    manifest = nil
    try? fileManager.removeItem(at: directory)
  }

  func listCompletedPackages() throws -> [CapturePackage] {
    let root = try sessionsRootURL()
    guard fileManager.fileExists(atPath: root.path) else { return [] }

    let directories = try fileManager.contentsOfDirectory(
      at: root,
      includingPropertiesForKeys: [.isDirectoryKey],
      options: [.skipsHiddenFiles]
    )

    var packages: [CapturePackage] = []
    for directory in directories {
      var isDirectory: ObjCBool = false
      guard fileManager.fileExists(atPath: directory.path, isDirectory: &isDirectory),
        isDirectory.boolValue
      else {
        continue
      }
      guard let package = try? loadPackage(at: directory),
        package.manifest.completedAt != nil
      else {
        continue
      }
      packages.append(package)
    }

    return packages.sorted { lhs, rhs in
      let left = lhs.manifest.completedAt ?? lhs.manifest.createdAt
      let right = rhs.manifest.completedAt ?? rhs.manifest.createdAt
      return left > right
    }
  }

  func loadPackage(at directoryURL: URL) throws -> CapturePackage {
    let manifestURL = directoryURL.appendingPathComponent("manifest.json")
    guard fileManager.fileExists(atPath: manifestURL.path) else {
      throw CapturePackageError.manifestMissing(directoryURL)
    }

    let decoder = JSONDecoder()
    decoder.dateDecodingStrategy = .secondsSince1970
    let manifest = try decoder.decode(
      CaptureSessionManifest.self,
      from: Data(contentsOf: manifestURL)
    )
    return CapturePackage(
      directoryURL: directoryURL,
      manifestURL: manifestURL,
      manifest: manifest
    )
  }

  func deletePackage(_ package: CapturePackage) throws {
    if packageDirectory == package.directoryURL {
      packageDirectory = nil
      manifest = nil
    }
    try fileManager.removeItem(at: package.directoryURL)
  }

  func clearEngineOutput(for package: CapturePackage) throws {
    let outputDirectory = package.directoryURL
      .appendingPathComponent("engine-output", isDirectory: true)
    if fileManager.fileExists(atPath: outputDirectory.path) {
      try fileManager.removeItem(at: outputDirectory)
    }
  }

  func makeShareArchive(for package: CapturePackage) throws -> URL {
    try CaptureShareArchive.makeZip(for: package, fileManager: fileManager)
  }

  private func sessionsRootURL() throws -> URL {
    if let captureSessionsRootURL {
      try fileManager.createDirectory(at: captureSessionsRootURL, withIntermediateDirectories: true)
      return captureSessionsRootURL
    }
    let root = try fileManager.url(
      for: .applicationSupportDirectory,
      in: .userDomainMask,
      appropriateFor: nil,
      create: true
    )
    .appendingPathComponent("CaptureSessions", isDirectory: true)
    try fileManager.createDirectory(at: root, withIntermediateDirectories: true)
    return root
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
  case manifestMissing(URL)

  var errorDescription: String? {
    switch self {
    case .sessionNotStarted:
      "The capture package has not been started."
    case .duplicateTarget(let targetID):
      "The capture target \(targetID) was already saved."
    case .incompleteCapture(let expected, let received):
      "The package has \(received) of \(expected) required frames."
    case .manifestMissing:
      "The capture package is missing its manifest."
    }
  }
}

extension CapturePackage {
  var panoramaURL: URL {
    directoryURL
      .appendingPathComponent("engine-output", isDirectory: true)
      .appendingPathComponent("panorama_equirectangular.jpg")
  }

  var hasPanorama: Bool {
    FileManager.default.fileExists(atPath: panoramaURL.path)
  }

  var firstImageURL: URL? {
    guard let frame = manifest.frames.first else { return nil }
    return directoryURL
      .appendingPathComponent(manifest.imageDirectory, isDirectory: true)
      .appendingPathComponent(frame.imageFilename)
  }
}
