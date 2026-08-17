import Foundation

actor ExperimentalCapturePackageStore {
  private let fileManager: FileManager
  private let sessionsRootURL: URL?
  private var packageDirectory: URL?
  private var manifest: ExperimentalCaptureManifest?

  init(
    fileManager: FileManager = .default,
    sessionsRootURL: URL? = nil
  ) {
    self.fileManager = fileManager
    self.sessionsRootURL = sessionsRootURL
  }

  func begin(
    configuration: ExperimentalPanoramaConfiguration,
    coreMotionReferenceFrame: String
  ) throws -> ExperimentalCapturePackage {
    try cleanupIncompleteSessions()

    let root = try sessionsRoot()
    let sessionID = UUID()
    let directory = root.appendingPathComponent(sessionID.uuidString, isDirectory: true)
    let imageDirectory = directory.appendingPathComponent("images", isDirectory: true)
    try fileManager.createDirectory(at: imageDirectory, withIntermediateDirectories: true)

    let summaries = configuration.scanLineOrder.map { line in
      ExperimentalScanLineSummary(
        scanLine: line,
        imageCount: configuration.imageCount(for: line),
        capturedCount: 0,
        skippedCount: nil,
        startedAt: nil,
        completedAt: nil
      )
    }

    let manifest = ExperimentalCaptureManifest(
      schemaVersion: ExperimentalCaptureManifest.currentSchemaVersion,
      kind: ExperimentalCaptureManifest.kindIdentifier,
      sessionID: sessionID,
      timestampConvention: "UTC seconds since Unix epoch; ARKit frame.timestamp is seconds from a monotonic host clock",
      createdAt: Date(),
      completedAt: nil,
      isComplete: false,
      incompleteReason: nil,
      configuration: configuration,
      imageDirectory: "images",
      sessionStartTransform: nil,
      sessionStartTimestamp: nil,
      coreMotionReferenceFrame: coreMotionReferenceFrame,
      frames: [],
      lineSummaries: summaries,
      skippedTargets: nil
    )

    packageDirectory = directory
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
    return ExperimentalCapturePackage(
      directoryURL: directory,
      manifestURL: directory.appendingPathComponent("manifest.json"),
      manifest: manifest
    )
  }

  func recordSessionStart(transform: Matrix4x4Value, timestamp: TimeInterval) throws {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    manifest.sessionStartTransform = transform
    manifest.sessionStartTimestamp = timestamp
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
  }

  func markLineStarted(_ line: PanoramaScanLine) throws {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    if let index = manifest.lineSummaries.firstIndex(where: { $0.scanLine == line }) {
      let summary = manifest.lineSummaries[index]
      manifest.lineSummaries[index] = ExperimentalScanLineSummary(
        scanLine: summary.scanLine,
        imageCount: summary.imageCount,
        capturedCount: summary.capturedCount,
        skippedCount: summary.skippedCount,
        startedAt: summary.startedAt ?? Date(),
        completedAt: summary.completedAt
      )
    }
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
  }

  func markLineCompleted(_ line: PanoramaScanLine, skippedCount: Int = 0) throws {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    if let index = manifest.lineSummaries.firstIndex(where: { $0.scanLine == line }) {
      let summary = manifest.lineSummaries[index]
      let captured = manifest.frames.filter { $0.scanLine == line }.count
      manifest.lineSummaries[index] = ExperimentalScanLineSummary(
        scanLine: summary.scanLine,
        imageCount: summary.imageCount,
        capturedCount: captured,
        skippedCount: skippedCount > 0 ? skippedCount : nil,
        startedAt: summary.startedAt,
        completedAt: Date()
      )
    }
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
  }

  func append(
    imageData: Data,
    frameID: UUID,
    scanLine: PanoramaScanLine,
    indexInLine: Int,
    targetYawOffsetDegrees: Double,
    actualYawOffsetDegrees: Double,
    actualPitchDegrees: Double,
    arkit: ARKitCameraMetadata,
    motion: MotionSample?,
    photo: PhotoMetadata,
    qualityNotes: [String]
  ) throws -> ExperimentalCapturedFrame {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    guard !manifest.frames.contains(where: { $0.id == frameID }) else {
      throw ExperimentalCapturePackageError.duplicateFrame(frameID)
    }
    let lineCount = manifest.configuration.imageCount(for: scanLine)
    let filename = Self.imageFilename(
      scanLine: scanLine,
      indexInLine: indexInLine,
      frameID: frameID
    )
    let imageURL =
      directory
      .appendingPathComponent(manifest.imageDirectory, isDirectory: true)
      .appendingPathComponent(filename)
    try imageData.write(
      to: imageURL,
      options: [.atomic, .completeFileProtectionUntilFirstUserAuthentication]
    )

    let record = ExperimentalCapturedFrame(
      id: frameID,
      imageFilename: filename,
      captureTimestamp: Date(),
      scanLine: scanLine,
      indexInLine: indexInLine,
      lineImageCount: lineCount,
      sequenceIndex: manifest.frames.count,
      targetYawOffsetDegrees: targetYawOffsetDegrees,
      actualYawOffsetDegrees: actualYawOffsetDegrees,
      actualPitchDegrees: actualPitchDegrees,
      arkit: arkit,
      motion: motion,
      photo: photo,
      qualityNotes: qualityNotes
    )
    manifest.frames.append(record)
    if let index = manifest.lineSummaries.firstIndex(where: { $0.scanLine == scanLine }) {
      let summary = manifest.lineSummaries[index]
      manifest.lineSummaries[index] = ExperimentalScanLineSummary(
        scanLine: summary.scanLine,
        imageCount: summary.imageCount,
        capturedCount: manifest.frames.filter { $0.scanLine == scanLine }.count,
        skippedCount: summary.skippedCount,
        startedAt: summary.startedAt,
        completedAt: summary.completedAt
      )
    }
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
    return record
  }

  /// Removes everything captured for one row, so a sweep that was interrupted
  /// mid-row can shoot that row again without leaving duplicates behind.
  func discardFrames(for line: PanoramaScanLine) throws {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    let doomed = manifest.frames.filter { $0.scanLine == line }
    guard !doomed.isEmpty else { return }
    let imagesDirectory = directory.appendingPathComponent(
      manifest.imageDirectory,
      isDirectory: true
    )
    for frame in doomed {
      try? fileManager.removeItem(
        at: imagesDirectory.appendingPathComponent(frame.imageFilename)
      )
    }
    manifest.frames = manifest.frames
      .filter { $0.scanLine != line }
      .enumerated()
      .map { $0.element.withSequenceIndex($0.offset) }
    if let index = manifest.lineSummaries.firstIndex(where: { $0.scanLine == line }) {
      let summary = manifest.lineSummaries[index]
      manifest.lineSummaries[index] = ExperimentalScanLineSummary(
        scanLine: summary.scanLine,
        imageCount: summary.imageCount,
        capturedCount: 0,
        skippedCount: nil,
        startedAt: summary.startedAt,
        completedAt: nil
      )
    }
    self.manifest = manifest
    try write(manifest, named: "manifest.json", in: directory)
  }

  /// Saves the session. Angles the sweep could not reach are recorded rather
  /// than treated as a failure, so a usable capture is never thrown away over
  /// a handful of missing frames.
  func finalize(
    skippedTargets: [ExperimentalSkippedTarget] = []
  ) throws -> ExperimentalCapturePackage {
    guard let directory = packageDirectory, var manifest else {
      throw ExperimentalCapturePackageError.sessionNotStarted
    }
    let expected = manifest.configuration.totalImageCount
    guard manifest.frames.count >= manifest.configuration.minimumUsableFrameCount else {
      throw ExperimentalCapturePackageError.incompleteCapture(
        expected: expected,
        received: manifest.frames.count
      )
    }
    manifest.completedAt = Date()
    manifest.isComplete = true
    manifest.incompleteReason = nil
    manifest.skippedTargets = skippedTargets.isEmpty ? nil : skippedTargets
    try write(manifest, named: "manifest.json", in: directory)
    packageDirectory = nil
    self.manifest = nil
    return ExperimentalCapturePackage(
      directoryURL: directory,
      manifestURL: directory.appendingPathComponent("manifest.json"),
      manifest: manifest
    )
  }

  func abandon() {
    guard let directory = packageDirectory else { return }
    packageDirectory = nil
    manifest = nil
    try? fileManager.removeItem(at: directory)
  }

  func listCompletedPackages() throws -> [ExperimentalCapturePackage] {
    try cleanupIncompleteSessions()
    let root = try sessionsRoot()
    guard fileManager.fileExists(atPath: root.path) else { return [] }

    let directories = try fileManager.contentsOfDirectory(
      at: root,
      includingPropertiesForKeys: [.isDirectoryKey],
      options: [.skipsHiddenFiles]
    )

    var packages: [ExperimentalCapturePackage] = []
    for directory in directories {
      var isDirectory: ObjCBool = false
      guard fileManager.fileExists(atPath: directory.path, isDirectory: &isDirectory),
        isDirectory.boolValue
      else {
        continue
      }
      guard let package = try? loadPackage(at: directory),
        package.manifest.isComplete,
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

  func loadPackage(at directoryURL: URL) throws -> ExperimentalCapturePackage {
    let manifestURL = directoryURL.appendingPathComponent("manifest.json")
    guard fileManager.fileExists(atPath: manifestURL.path) else {
      throw ExperimentalCapturePackageError.manifestMissing(directoryURL)
    }
    let decoder = JSONDecoder()
    decoder.dateDecodingStrategy = .secondsSince1970
    let manifest = try decoder.decode(
      ExperimentalCaptureManifest.self,
      from: Data(contentsOf: manifestURL)
    )
    return ExperimentalCapturePackage(
      directoryURL: directoryURL,
      manifestURL: manifestURL,
      manifest: manifest
    )
  }

  func deletePackage(_ package: ExperimentalCapturePackage) throws {
    if packageDirectory == package.directoryURL {
      packageDirectory = nil
      manifest = nil
    }
    try fileManager.removeItem(at: package.directoryURL)
  }

  func makeShareArchive(for package: ExperimentalCapturePackage) throws -> URL {
    try CaptureShareArchive.makeZip(for: package, fileManager: fileManager)
  }

  static func imageFilename(
    scanLine: PanoramaScanLine,
    indexInLine: Int,
    frameID: UUID
  ) -> String {
    String(format: "%@_%02d_%@.jpg", scanLine.rawValue, indexInLine, frameID.uuidString)
  }

  private func cleanupIncompleteSessions() throws {
    if packageDirectory != nil { return }
    let root = try sessionsRoot()
    guard fileManager.fileExists(atPath: root.path) else { return }
    let directories = try fileManager.contentsOfDirectory(
      at: root,
      includingPropertiesForKeys: [.isDirectoryKey],
      options: [.skipsHiddenFiles]
    )
    for directory in directories {
      var isDirectory: ObjCBool = false
      guard fileManager.fileExists(atPath: directory.path, isDirectory: &isDirectory),
        isDirectory.boolValue
      else {
        continue
      }
      let package = try? loadPackage(at: directory)
      if package == nil || package?.manifest.completedAt == nil || package?.manifest.isComplete != true
      {
        try? fileManager.removeItem(at: directory)
      }
    }
  }

  private func sessionsRoot() throws -> URL {
    if let sessionsRootURL {
      try fileManager.createDirectory(at: sessionsRootURL, withIntermediateDirectories: true)
      return sessionsRootURL
    }
    let root = try fileManager.url(
      for: .applicationSupportDirectory,
      in: .userDomainMask,
      appropriateFor: nil,
      create: true
    )
    .appendingPathComponent("ExperimentalCaptureSessions", isDirectory: true)
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

enum ExperimentalCapturePackageError: LocalizedError {
  case sessionNotStarted
  case duplicateFrame(UUID)
  case incompleteCapture(expected: Int, received: Int)
  case manifestMissing(URL)

  var errorDescription: String? {
    switch self {
    case .sessionNotStarted:
      "The experimental capture package has not been started."
    case .duplicateFrame(let id):
      "The experimental frame \(id.uuidString) was already saved."
    case .incompleteCapture(let expected, let received):
      "Only \(received) of \(expected) photos were captured, which is too few to keep. Try the sweep again."
    case .manifestMissing:
      "The experimental capture package is missing its manifest."
    }
  }
}
