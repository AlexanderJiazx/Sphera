import Foundation
import SpheraMetalEngine

/// Experimental Swift/Metal stitch. Selected only when the Settings toggle is on.
/// Default Compute on device still uses the stable OpenCV engine.
final class ExperimentalSpheraEngine: NativeSpheraEngine, @unchecked Sendable {
  /// Dedicated GCD queue. `SpheraMetalStitch.run` blocks on Metal
  /// `waitUntilCompleted` and CPU SIFT; doing that inside `Task.detached`
  /// occupies Swift's cooperative thread pool and can hang the stitch.
  private static let stitchQueue = DispatchQueue(
    label: "sphera.experimental.metal.stitch",
    qos: .userInitiated
  )

  func stitch(
    _ request: SpheraEngineRequest,
    progress: StitchProgressHandler?
  ) async throws -> StitchingResult {
    report(progress, fraction: 0, message: "Starting experimental Metal stitch")
    NSLog("Sphera experimental Metal stitch: Run scheme must be Release so SIFT is -O, not -Onone")
    await Task.yield()

    let captureURL = request.packageDirectoryURL
    let outputURL = request.outputDirectoryURL
    return try await withCheckedThrowingContinuation { continuation in
      Self.stitchQueue.async {
        do {
          let result = try SpheraMetalStitch.run(
            captureURL: captureURL,
            outputDirectoryURL: outputURL
          ) { fraction, message in
            progress?(
              StitchProgress(
                fraction: min(1, max(0, fraction)),
                message: message
              )
            )
          }
          continuation.resume(
            returning: StitchingResult(
              panoramaURL: result.panoramaURL,
              reportURL: result.reportURL
            )
          )
        } catch {
          continuation.resume(throwing: error)
        }
      }
    }
  }

  private func report(
    _ progress: StitchProgressHandler?,
    fraction: Double,
    message: String
  ) {
    progress?(
      StitchProgress(
        fraction: min(1, max(0, fraction)),
        message: message
      )
    )
  }
}
