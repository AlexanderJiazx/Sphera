import SwiftUI

/// Intentionally contains no globe, target map, rotating arrow, degree readout,
/// or diagonal instruction. The user receives exactly one physical action.
struct CaptureGuideView: View {
  let instruction: CaptureNavigationInstruction

  var body: some View {
    VStack(spacing: 16) {
      Image(systemName: instruction.movement.symbolName)
        .font(.system(size: 92, weight: .black))
        .foregroundStyle(instruction.movement.color)
        .frame(width: 120, height: 108)

      Text(instruction.movement.title)
        .font(.system(size: 38, weight: .black, design: .rounded))
        .minimumScaleFactor(0.7)
        .multilineTextAlignment(.center)
        .lineLimit(1)

      Text(instruction.movement.detail)
        .font(.title3.bold())
        .foregroundStyle(instruction.movement.color)
        .frame(height: 28)

      if instruction.movement == .holdStill {
        ProgressView(value: instruction.holdProgress)
          .tint(.green)
          .scaleEffect(x: 1, y: 2, anchor: .center)
          .padding(.horizontal, 24)
      }
    }
    .padding(.horizontal, 24)
    .padding(.vertical, 28)
    .frame(maxWidth: .infinity)
    .background(.black.opacity(0.82), in: RoundedRectangle(cornerRadius: 18))
    .overlay {
      RoundedRectangle(cornerRadius: 18)
        .stroke(instruction.movement.color.opacity(0.8), lineWidth: 2)
    }
    .accessibilityElement(children: .ignore)
    .accessibilityLabel(instruction.accessibilityLabel)
  }
}

extension CaptureNavigationInstruction {
  fileprivate var accessibilityLabel: String {
    "\(movement.title), \(movement.detail)"
  }
}

extension CaptureMovement {
  fileprivate var title: String {
    switch self {
    case .preparing: "GET READY"
    case .turnLeft: "TURN LEFT"
    case .turnRight: "TURN RIGHT"
    case .tiltUp: "TILT UP"
    case .tiltDown: "TILT DOWN"
    case .holdStill: "HOLD STILL"
    case .capturing: "CAPTURING"
    }
  }

  fileprivate var detail: String {
    switch self {
    case .preparing: "READING ORIENTATION"
    case .turnLeft, .turnRight: "MOVE YOUR BODY THIS WAY"
    case .tiltUp, .tiltDown: "MOVE THE CAMERA THIS WAY"
    case .holdStill: "AUTO CAPTURE"
    case .capturing: "KEEP STILL"
    }
  }

  fileprivate var symbolName: String {
    switch self {
    case .preparing: "iphone"
    case .turnLeft: "arrow.left"
    case .turnRight: "arrow.right"
    case .tiltUp: "arrow.up"
    case .tiltDown: "arrow.down"
    case .holdStill: "scope"
    case .capturing: "camera.fill"
    }
  }

  fileprivate var color: Color {
    switch self {
    case .holdStill: .green
    case .capturing: .yellow
    case .preparing: .white
    default: .orange
    }
  }
}
