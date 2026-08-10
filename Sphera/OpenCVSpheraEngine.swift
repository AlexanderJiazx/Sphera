import Foundation

enum OpenCVSpheraEngineError: LocalizedError {
  case missingResult

  var errorDescription: String? {
    switch self {
    case .missingResult:
      "The native Sphera engine finished without returning a panorama."
    }
  }
}

/// Swift concurrency wrapper around the Objective-C++ engine boundary.
final class OpenCVSpheraEngine: NativeSpheraEngine, @unchecked Sendable {
  func stitch(_ request: SpheraEngineRequest) async throws -> StitchingResult {
    try await withCheckedThrowingContinuation { continuation in
      SpheraNativeEngineBridge.stitch(
        manifestURL: request.manifestURL,
        outputDirectoryURL: request.outputDirectoryURL,
        maximumPoseRefinementDegrees: request.maximumPoseRefinementDegrees
      ) { artifacts, error in
        if let error {
          continuation.resume(throwing: error)
        } else if let artifacts {
          continuation.resume(
            returning: StitchingResult(
              panoramaURL: artifacts.panoramaURL,
              reportURL: artifacts.reportURL
            )
          )
        } else {
          continuation.resume(throwing: OpenCVSpheraEngineError.missingResult)
        }
      }
    }
  }
}
