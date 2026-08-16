import Foundation

enum CaptureShareArchive {
  /// Builds a zip archive of an experimental ARKit capture. Includes images,
  /// the session manifest, and per-frame ARKit/gyro sidecars. Does not include
  /// an engine request — the current stitcher must not consume this dataset.
  static func makeZip(
    for package: ExperimentalCapturePackage,
    fileManager: FileManager = .default
  ) throws -> URL {
    let zipURL = try makeArchiveURL(forSessionID: package.manifest.sessionID, date: package.manifest.completedAt ?? package.manifest.createdAt, fileManager: fileManager, prefix: "SpheraARKitCapture")
    var entries: [(name: String, data: Data)] = []

    let manifestData = try Data(contentsOf: package.manifestURL)
    entries.append((name: "manifest.json", data: manifestData))

    let imageDirectory = package.directoryURL
      .appendingPathComponent(package.manifest.imageDirectory, isDirectory: true)
    for frame in package.manifest.frames {
      let imageURL = imageDirectory.appendingPathComponent(frame.imageFilename)
      let data = try Data(contentsOf: imageURL)
      entries.append((name: "\(package.manifest.imageDirectory)/\(frame.imageFilename)", data: data))

      let sidecarName = frame.imageFilename.replacingOccurrences(of: ".jpg", with: ".json")
      entries.append((name: "metadata/\(sidecarName)", data: try encodeExperimentalFrameSidecar(frame)))
    }

    let notes = """
    # Sphera experimental ARKit panorama capture

    This archive is from the experimental ARKit-guided capture mode.
    It is a data collection package for a future computer-side stitcher.

    Do not run it through the current on-device OpenCV/Metal stitching engine.

    Contents:
    - `manifest.json` — session configuration, scan-line summaries, and per-image records
    - `images/` — JPEG keyframes named `{scanLine}_{index}_{frameUUID}.jpg`
    - `metadata/` — the same records as `manifest.frames[]`, one file per image

    Every image is matched to ARKit pose data by `id` / filename, not array order.
    `arkit.transform` is the original column-major 4x4 camera matrix.
    `motion` is the CoreMotion sample nearest to the exposure, including gyroscope rates.
    """
    if let notesData = notes.data(using: .utf8) {
      entries.append((name: "ARKIT_EXPERIMENTAL_NOTES.md", data: notesData))
    }

    try ZipWriter.write(entries: entries, to: zipURL)
    return zipURL
  }

  /// Builds a zip archive of the capture package for the system share sheet.
  /// Includes images and per-photo metadata (`manifest.json`), plus the engine request copy when present.
  static func makeZip(
    for package: CapturePackage,
    fileManager: FileManager = .default
  ) throws -> URL {
    let zipURL = try makeArchiveURL(
      forSessionID: package.manifest.sessionID,
      date: package.manifest.completedAt ?? package.manifest.createdAt,
      fileManager: fileManager,
      prefix: "SpheraCapture"
    )

    var entries: [(name: String, data: Data)] = []

    let manifestData = try Data(contentsOf: package.manifestURL)
    entries.append((name: "manifest.json", data: manifestData))

    let engineRequestURL = package.directoryURL.appendingPathComponent("sphera-engine-request.json")
    if fileManager.fileExists(atPath: engineRequestURL.path) {
      entries.append((name: "sphera-engine-request.json", data: try Data(contentsOf: engineRequestURL)))
    }

    let imageDirectory = package.directoryURL
      .appendingPathComponent(package.manifest.imageDirectory, isDirectory: true)
    for frame in package.manifest.frames {
      let imageURL = imageDirectory.appendingPathComponent(frame.imageFilename)
      let data = try Data(contentsOf: imageURL)
      entries.append((name: "\(package.manifest.imageDirectory)/\(frame.imageFilename)", data: data))

      let frameMetadata = try encodeFrameSidecar(frame)
      let sidecarName = frame.imageFilename.replacingOccurrences(of: ".jpg", with: ".json")
      entries.append((name: "metadata/\(sidecarName)", data: frameMetadata))
    }

    let recipe = """
    # Sphera on-device sensor-first stitch

    The iOS app stitches with recorded CoreMotion poses, per-frame intrinsics,
    SIFT, and an adaptive periodic ring seam. No LoFTR/CoreML models are loaded
    on the default path.

    Optional offline diagnostic (not required for product quality):

    1. Unzip this archive and prepare display-oriented images if needed.
    2. From the Engine repo root, run the sensor-first Python recipe or a
       developer-only LoFTR diagnostic if comparing oracles.
    3. Import `panorama_equirectangular.jpg` (and optional `report.json`) back
       into the iOS gallery via **Import Engine panorama**.
    """
    if let recipeData = recipe.data(using: .utf8) {
      entries.append((name: "ENGINE_SENSOR_FIRST_NOTES.md", data: recipeData))
    }

    try ZipWriter.write(entries: entries, to: zipURL)
    return zipURL
  }

  private static func makeArchiveURL(
    forSessionID sessionID: UUID,
    date: Date,
    fileManager: FileManager,
    prefix: String
  ) throws -> URL {
    let formatter = DateFormatter()
    formatter.locale = Locale(identifier: "en_US_POSIX")
    formatter.timeZone = TimeZone(secondsFromGMT: 0)
    formatter.dateFormat = "yyyyMMdd-HHmmss"
    let stamp = formatter.string(from: date)
    let shortID = sessionID.uuidString.prefix(8)
    let archiveName = "\(prefix)-\(stamp)-\(shortID).zip"

    let temporaryRoot = fileManager.temporaryDirectory
      .appendingPathComponent("SpheraShare-\(UUID().uuidString)", isDirectory: true)
    try fileManager.createDirectory(at: temporaryRoot, withIntermediateDirectories: true)
    return temporaryRoot.appendingPathComponent(archiveName)
  }

  private static func encodeFrameSidecar(_ frame: CapturedFrameRecord) throws -> Data {
    try encodeJSON(frame)
  }

  private static func encodeExperimentalFrameSidecar(_ frame: ExperimentalCapturedFrame) throws -> Data {
    try encodeJSON(frame)
  }

  private static func encodeJSON<T: Encodable>(_ value: T) throws -> Data {
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes]
    encoder.dateEncodingStrategy = .secondsSince1970
    return try encoder.encode(value)
  }
}

/// Minimal ZIP writer (store method). JPEGs are already compressed.
enum ZipWriter {
  static func write(entries: [(name: String, data: Data)], to url: URL) throws {
    var centralDirectory = Data()
    var fileData = Data()
    var offset: UInt32 = 0

    for entry in entries {
      let nameData = Data(entry.name.utf8)
      let crc = crc32(entry.data)
      let size = UInt32(entry.data.count)
      let nameLength = UInt16(nameData.count)

      var local = Data()
      local.appendUInt32(0x0403_4b50)  // local file header signature
      local.appendUInt16(20)  // version needed
      local.appendUInt16(0)  // flags
      local.appendUInt16(0)  // compression: store
      local.appendUInt16(0)  // mod time
      local.appendUInt16(0)  // mod date
      local.appendUInt32(crc)
      local.appendUInt32(size)
      local.appendUInt32(size)
      local.appendUInt16(nameLength)
      local.appendUInt16(0)  // extra length
      local.append(nameData)
      local.append(entry.data)
      fileData.append(local)

      var central = Data()
      central.appendUInt32(0x0201_4b50)  // central directory header
      central.appendUInt16(20)  // version made by
      central.appendUInt16(20)  // version needed
      central.appendUInt16(0)  // flags
      central.appendUInt16(0)  // compression
      central.appendUInt16(0)  // mod time
      central.appendUInt16(0)  // mod date
      central.appendUInt32(crc)
      central.appendUInt32(size)
      central.appendUInt32(size)
      central.appendUInt16(nameLength)
      central.appendUInt16(0)  // extra
      central.appendUInt16(0)  // comment
      central.appendUInt16(0)  // disk start
      central.appendUInt16(0)  // internal attrs
      central.appendUInt32(0)  // external attrs
      central.appendUInt32(offset)
      central.append(nameData)
      centralDirectory.append(central)

      offset += UInt32(local.count)
    }

    var end = Data()
    end.appendUInt32(0x0605_4b50)
    end.appendUInt16(0)
    end.appendUInt16(0)
    end.appendUInt16(UInt16(entries.count))
    end.appendUInt16(UInt16(entries.count))
    end.appendUInt32(UInt32(centralDirectory.count))
    end.appendUInt32(offset)
    end.appendUInt16(0)

    var archive = Data()
    archive.append(fileData)
    archive.append(centralDirectory)
    archive.append(end)
    try archive.write(to: url, options: [.atomic])
  }

  private static func crc32(_ data: Data) -> UInt32 {
    var crc: UInt32 = 0xffff_ffff
    for byte in data {
      crc ^= UInt32(byte)
      for _ in 0..<8 {
        if crc & 1 != 0 {
          crc = (crc >> 1) ^ 0xedb8_8320
        } else {
          crc >>= 1
        }
      }
    }
    return crc ^ 0xffff_ffff
  }
}

private extension Data {
  mutating func appendUInt16(_ value: UInt16) {
    var le = value.littleEndian
    Swift.withUnsafeBytes(of: &le) { append(contentsOf: $0) }
  }

  mutating func appendUInt32(_ value: UInt32) {
    var le = value.littleEndian
    Swift.withUnsafeBytes(of: &le) { append(contentsOf: $0) }
  }
}
