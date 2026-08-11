import Foundation

struct SpheraEngineRequest: Sendable {
  let packageDirectoryURL: URL
  let imageDirectoryURL: URL
  let manifestURL: URL
  let outputDirectoryURL: URL
  let initialCameraRotations: [Matrix3x3Value]
}

struct StitchProgress: Equatable, Sendable {
  /// Overall compute progress in `0...1`.
  var fraction: Double
  var message: String
}

/// Fire-and-forget on purpose: must not `await` the main actor from the stitch
/// worker or UIKit/CoreML can deadlock the UI at the first progress tick.
typealias StitchProgressHandler = @Sendable (StitchProgress) -> Void

protocol NativeSpheraEngine: Sendable {
  func stitch(
    _ request: SpheraEngineRequest,
    progress: StitchProgressHandler?
  ) async throws -> StitchingResult
}

protocol PanoramaStitching: Sendable {
  func stitch(
    package: CapturePackage,
    progress: StitchProgressHandler?
  ) async throws -> StitchingResult
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

  func stitch(
    package: CapturePackage,
    progress: StitchProgressHandler? = nil
  ) async throws -> StitchingResult {
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
      }
    )

    if let nativeEngine {
      // Keep the adapter actor free — all heavy work runs detached.
      return try await Task.detached(priority: .userInitiated) {
        try await nativeEngine.stitch(request, progress: progress)
      }.value
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
