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
      schemaVersion: 6,
      sessionID: sessionID,
      timestampConvention: "UTC seconds since Unix epoch; sub-millisecond precision preserved",
      createdAt: Date(),
      completedAt: nil,
      plan: plan,
      imageDirectory: "images",
      coreMotionReferenceFrame: coreMotionReferenceFrame,
      engineInitialization: EngineInitializationMetadata(
        placementSource: "recorded",
        rotationField: "frames[].pose.cameraToCaptureReferenceRotationMatrix",
        usePosePriors: true,
        allowGlobalArrangementRediscovery: false,
        maximumPoseRefinementDegrees: 6,
        refinementPurpose:
          "On-device sensor-first S1 polar-cube v2: recorded capture_ref poses with per-frame locked intrinsics, pose-overlap SIFT pairs, bounded sensor-anchored refinement (6° cap), adaptive periodic ring ownership, connected exposure, concurrent ring-local graph cuts, five-band blend, projection-native zenith composition, and a gated nadir cube face. LoFTR remains an optional offline diagnostic only.",
        enabledPipelineStages: [
          "manifest-canonicalize",
          "pose-overlap-graph",
          "per-frame-locked-intrinsics",
          "sift-matching",
          "sensor-anchored-refinement",
          "adaptive-periodic-ring-seam",
          "exposure-gain-blocks",
          "concurrent-ring-local-structure-graph-cut",
          "five-band-blend",
          "projection-native-top-cube-face",
          "projection-native-bottom-cube-face",
          "residual-direct-sphere-fill",
        ]
      ),
      primaryCapture: nil,
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
    alignment: AlignmentMetadata,
    primaryCapture: PrimaryCaptureMetadata? = nil
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
    try photo.data.write(to: imageURL, options: [.atomic, .completeFileProtectionUntilFirstUserAuthentication])

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
    if let primaryCapture {
      manifest.primaryCapture = PrimaryCaptureMetadata(
        imageFilename: filename,
        targetId: primaryCapture.targetId,
        classifiedRing: primaryCapture.classifiedRing
      )
    }
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

  /// Imports an external equirectangular result into the capture package so
  /// the on-device gallery/viewer can display it (Engine sensor-first or
  /// optional offline diagnostic outputs).
  func importEnginePanorama(
    into package: CapturePackage,
    panoramaURL: URL,
    reportURL: URL? = nil
  ) throws {
    let outputDirectory = package.directoryURL
      .appendingPathComponent("engine-output", isDirectory: true)
    try fileManager.createDirectory(at: outputDirectory, withIntermediateDirectories: true)

    let destinationPanorama = outputDirectory
      .appendingPathComponent("panorama_equirectangular.jpg")
    if fileManager.fileExists(atPath: destinationPanorama.path) {
      try fileManager.removeItem(at: destinationPanorama)
    }
    try fileManager.copyItem(at: panoramaURL, to: destinationPanorama)

    let destinationReport = outputDirectory.appendingPathComponent("report.json")
    if fileManager.fileExists(atPath: destinationReport.path) {
      try fileManager.removeItem(at: destinationReport)
    }
    if let reportURL {
      try fileManager.copyItem(at: reportURL, to: destinationReport)
    } else {
      let imported: [String: Any] = [
        "engine": "sphera-engine-import",
        "engine_contract_version": 2,
        "status": "success",
        "recipe": "imported-hierarchical-loftr-or-external",
        "output": [
          "panorama_equirectangular": destinationPanorama.lastPathComponent
        ],
      ]
      let data = try JSONSerialization.data(withJSONObject: imported, options: [.prettyPrinted, .sortedKeys])
      try data.write(
        to: destinationReport,
        options: [.atomic, .completeFileProtectionUntilFirstUserAuthentication]
      )
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
      options: [.atomic, .completeFileProtectionUntilFirstUserAuthentication]
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
    return imageURL(for: frame.imageFilename)
  }

  var primaryImageURL: URL? {
    guard let filename = manifest.primaryCapture?.imageFilename, !filename.isEmpty else {
      return nil
    }
    return imageURL(for: filename)
  }

  /// Gallery thumbnails use the primary capture still, then the first frame.
  var previewImageURL: URL? {
    if let primaryImageURL, FileManager.default.fileExists(atPath: primaryImageURL.path) {
      return primaryImageURL
    }
    return firstImageURL
  }

  private func imageURL(for filename: String) -> URL {
    directoryURL
      .appendingPathComponent(manifest.imageDirectory, isDirectory: true)
      .appendingPathComponent(filename)
  }
}
