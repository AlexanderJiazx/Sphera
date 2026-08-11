import Foundation

enum CaptureShareArchive {
  /// Builds a zip archive of the capture package for the system share sheet.
  /// Includes images and per-photo metadata (`manifest.json`), plus the engine request copy when present.
  static func makeZip(
    for package: CapturePackage,
    fileManager: FileManager = .default
  ) throws -> URL {
    let formatter = DateFormatter()
    formatter.locale = Locale(identifier: "en_US_POSIX")
    formatter.timeZone = TimeZone(secondsFromGMT: 0)
    formatter.dateFormat = "yyyyMMdd-HHmmss"
    let stamp = formatter.string(from: package.manifest.completedAt ?? package.manifest.createdAt)
    let shortID = package.manifest.sessionID.uuidString.prefix(8)
    let archiveName = "SpheraCapture-\(stamp)-\(shortID).zip"

    let temporaryRoot = fileManager.temporaryDirectory
      .appendingPathComponent("SpheraShare-\(UUID().uuidString)", isDirectory: true)
    try fileManager.createDirectory(at: temporaryRoot, withIntermediateDirectories: true)
    let zipURL = temporaryRoot.appendingPathComponent(archiveName)

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
    # Sphera Engine hierarchical LoFTR recipe

    On-device OpenCV can stitch without neural models. For the compact-LoFTR
    hierarchical quality path (~44 MB outdoor weights, competitive with full RoMa):

    1. Unzip this archive and prepare display-oriented images if needed.
    2. From the Engine repo root:

    ```bash
    .venv/bin/python scripts/run_hierarchical_loftr.py \\
      path/to/images_stitched \\
      outputs/from_iphone_hierarchical \\
      --capture-metadata path/to/sphera-engine-request.json \\
      --ml-python .venv-ml/bin/python
    ```

    3. Import `panorama_equirectangular.jpg` (and optional `report.json`) back
       into the iOS gallery via **Import Engine panorama**.
    """
    if let recipeData = recipe.data(using: .utf8) {
      entries.append((name: "ENGINE_HIERARCHICAL_RECIPE.md", data: recipeData))
    }

    try ZipWriter.write(entries: entries, to: zipURL)
    return zipURL
  }

  private static func encodeFrameSidecar(_ frame: CapturedFrameRecord) throws -> Data {
    let encoder = JSONEncoder()
    encoder.outputFormatting = [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes]
    encoder.dateEncodingStrategy = .secondsSince1970
    return try encoder.encode(frame)
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
