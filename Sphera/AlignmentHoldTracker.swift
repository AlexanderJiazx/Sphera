import Foundation

struct AlignmentHoldUpdate: Equatable, Sendable {
  let progress: Double
  let shouldCapture: Bool
}

struct AlignmentHoldTracker: Sendable {
  private var alignedSinceTimestamp: Double?

  mutating func update(
    isAligned: Bool,
    timestamp: Double,
    requiredDuration: Double,
    blockedUntilTimestamp: Double
  ) -> AlignmentHoldUpdate {
    guard isAligned, timestamp >= blockedUntilTimestamp else {
      reset()
      return AlignmentHoldUpdate(progress: 0, shouldCapture: false)
    }

    if alignedSinceTimestamp == nil || timestamp < (alignedSinceTimestamp ?? timestamp) {
      alignedSinceTimestamp = timestamp
    }
    let elapsed = max(0, timestamp - (alignedSinceTimestamp ?? timestamp))
    let hasCompleted = requiredDuration <= 0 || elapsed + 0.000_001 >= requiredDuration
    let progress = hasCompleted ? 1 : min(1, elapsed / requiredDuration)
    return AlignmentHoldUpdate(progress: progress, shouldCapture: hasCompleted)
  }

  mutating func reset() {
    alignedSinceTimestamp = nil
  }
}
