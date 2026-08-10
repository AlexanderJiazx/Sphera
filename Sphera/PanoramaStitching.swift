import Foundation

struct SpheraEngineRequest: Sendable {
  let packageDirectoryURL: URL
  let imageDirectoryURL: URL
  let manifestURL: URL
  let outputDirectoryURL: URL
  let initialCameraRotations: [Matrix3x3Value]
  let maximumPoseRefinementDegrees: Double
}

protocol NativeSpheraEngine: Sendable {
  func stitch(_ request: SpheraEngineRequest) async throws -> StitchingResult
}

protocol PanoramaStitching: Sendable {
  func stitch(package: CapturePackage) async throws -> StitchingResult
}

/// The only boundary between the Swift capture application and the panorama
/// engine. A native C++ wrapper can implement `NativeSpheraEngine` without
/// exposing AVFoundation, CoreMotion, SwiftUI, or capture state to the engine.
actor SpheraEngineAdapter: PanoramaStitching {
  private let nativeEngine: (any NativeSpheraEngine)?
  private let fileManager: FileManager

  init(
    nativeEngine: (any NativeSpheraEngine)? = nil,
    fileManager: FileManager = .default
  ) {
    self.nativeEngine = nativeEngine
    self.fileManager = fileManager
  }

  func stitch(package: CapturePackage) async throws -> StitchingResult {
    let outputDirectory = package.directoryURL
      .appendingPathComponent("engine-output", isDirectory: true)
    try fileManager.createDirectory(at: outputDirectory, withIntermediateDirectories: true)

    let request = SpheraEngineRequest(
      packageDirectoryURL: package.directoryURL,
      imageDirectoryURL: package.directoryURL
        .appendingPathComponent(package.manifest.imageDirectory, isDirectory: true),
      manifestURL: package.manifestURL,
      outputDirectoryURL: outputDirectory,
      initialCameraRotations: package.manifest.frames.map {
        $0.pose.cameraToCaptureReferenceRotationMatrix
      },
      maximumPoseRefinementDegrees: package.manifest
        .engineInitialization.maximumPoseRefinementDegrees
    )

    if let nativeEngine {
      return try await nativeEngine.stitch(request)
    }

    // This also permits a debug build to display an engine result restored
    // into the capture package without changing the UI layer.
    let panoramaURL = outputDirectory.appendingPathComponent("panorama_equirectangular.jpg")
    if fileManager.fileExists(atPath: panoramaURL.path) {
      let reportURL = outputDirectory.appendingPathComponent("report.json")
      return StitchingResult(
        panoramaURL: panoramaURL,
        reportURL: fileManager.fileExists(atPath: reportURL.path) ? reportURL : nil
      )
    }

    throw SpheraEngineAdapterError.nativeEngineNotLinked(request.manifestURL)
  }
}

enum SpheraEngineAdapterError: LocalizedError {
  case nativeEngineNotLinked(URL)

  var errorDescription: String? {
    switch self {
    case .nativeEngineNotLinked:
      "Capture completed, but no iOS-native Sphera engine implementation is linked."
    }
  }
}
