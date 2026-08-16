import Combine
import CoreMotion
import Foundation

final class LatestMotionSampleStore: @unchecked Sendable {
  private let lock = NSLock()
  private var storedSample: MotionSample?

  var latest: MotionSample? {
    lock.lock()
    defer { lock.unlock() }
    return storedSample
  }

  func update(_ sample: MotionSample) {
    lock.lock()
    storedSample = sample
    lock.unlock()
  }

  func clear() {
    lock.lock()
    storedSample = nil
    lock.unlock()
  }
}

@MainActor
final class MotionTrackingService: ObservableObject {
  @Published private(set) var currentSample: MotionSample?
  @Published private(set) var isAvailable: Bool

  let sampleStore = LatestMotionSampleStore()
  private let motionManager = CMMotionManager()
  private(set) var referenceFrameName = "xArbitraryZVertical"

  init() {
    isAvailable = motionManager.isDeviceMotionAvailable
  }

  func start() throws {
    guard motionManager.isDeviceMotionAvailable else {
      throw MotionTrackingError.deviceMotionUnavailable
    }
    guard !motionManager.isDeviceMotionActive else { return }

    sampleStore.clear()
    currentSample = nil
    motionManager.deviceMotionUpdateInterval = 1 / 60

    // A panorama session only needs a stable local heading. The magnetometer-
    // corrected frame can visibly change yaw when magnetic conditions change,
    // so use the gravity-stabilized gyro frame without heading corrections.
    referenceFrameName = "xArbitraryZVertical"
    motionManager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: .main) {
      [weak self] motion, _ in
      guard let self, let motion else { return }
      let quaternion = motion.attitude.quaternion
      let sample = MotionSample(
        monotonicTimestampSeconds: motion.timestamp,
        wallClockTimestamp: Date(),
        attitudeQuaternion: QuaternionValue(
          w: quaternion.w,
          x: quaternion.x,
          y: quaternion.y,
          z: quaternion.z
        ),
        gravity: Vector3Value(
          x: motion.gravity.x,
          y: motion.gravity.y,
          z: motion.gravity.z
        ),
        rotationRateRadiansPerSecond: Vector3Value(
          x: motion.rotationRate.x,
          y: motion.rotationRate.y,
          z: motion.rotationRate.z
        )
      )
      self.sampleStore.update(sample)
      self.currentSample = sample
    }
  }

  var isRunning: Bool {
    motionManager.isDeviceMotionActive
  }

  func stop() {
    motionManager.stopDeviceMotionUpdates()
    sampleStore.clear()
    currentSample = nil
  }

  func waitForFirstSample(timeoutSeconds: Double = 3) async throws -> MotionSample {
    let deadline = ProcessInfo.processInfo.systemUptime + timeoutSeconds
    while ProcessInfo.processInfo.systemUptime < deadline {
      if let sample = currentSample {
        return sample
      }
      try await Task.sleep(for: .milliseconds(50))
    }
    throw MotionTrackingError.timedOut
  }
}

enum MotionTrackingError: LocalizedError {
  case deviceMotionUnavailable
  case timedOut

  var errorDescription: String? {
    switch self {
    case .deviceMotionUnavailable:
      "CoreMotion device orientation is unavailable on this device."
    case .timedOut:
      "No CoreMotion orientation sample was received."
    }
  }
}
