import SwiftUI

struct CaptureSessionModeSwitch: View {
  let mode: CaptureSessionMode
  var isEnabled: Bool = true
  let onSelect: (CaptureSessionMode) -> Void

  var body: some View {
    EqualizedCaptureSessionModeSwitch(mode: mode, isEnabled: isEnabled, onSelect: onSelect)
      .equatable()
  }
}

private struct EqualizedCaptureSessionModeSwitch: View, Equatable {
  let mode: CaptureSessionMode
  var isEnabled: Bool = true
  let onSelect: (CaptureSessionMode) -> Void

  static func == (lhs: Self, rhs: Self) -> Bool {
    lhs.mode == rhs.mode && lhs.isEnabled == rhs.isEnabled
  }

  var body: some View {
    Menu {
      Picker(
        "Capture mode",
        selection: Binding(
          get: { mode },
          set: { newMode in
            onSelect(newMode)
          }
        )
      ) {
        ForEach(CaptureSessionMode.allCases, id: \.self) { sessionMode in
          Label(
            sessionMode == .standard ? "Points" : "ARKit Experimental",
            systemImage: sessionMode == .experimentalARKit ? "move.3d" : "circle.grid.3x3.fill"
          )
          .tag(sessionMode)
        }
      }
    } label: {
      HStack(spacing: 6) {
        Image(systemName: mode == .experimentalARKit ? "move.3d" : "circle.grid.3x3.fill")
          .font(.subheadline.weight(.bold))
        Text(mode.title)
          .font(.subheadline.weight(.bold))
        Image(systemName: "chevron.down")
          .font(.caption2.weight(.bold))
      }
      .foregroundStyle(mode == .experimentalARKit ? .black : .white)
      .padding(.horizontal, 14)
      .padding(.vertical, 8)
      .background {
        if mode == .experimentalARKit {
          Capsule().fill(Color(red: 1, green: 0.72, blue: 0.12))
        }
      }
      .liquidGlassInteractive(in: Capsule())
    }
    .contentShape(Capsule())
    .frame(minHeight: 44)
    .disabled(!isEnabled)
    .opacity(isEnabled ? 1 : 0.55)
    .accessibilityLabel(mode.accessibilityLabel)
    .accessibilityHint("Switches between standard point capture and experimental ARKit panorama capture")
  }
}

struct ExperimentalCaptureGuideView: View {
  let guidance: ExperimentalGuidanceSnapshot
  let isCapturingPhoto: Bool

  var body: some View {
    GeometryReader { geometry in
      let size = geometry.size
      ZStack {
        ForEach(PanoramaScanLine.guideOrder) { line in
          scanLine(line, in: size)
        }
      }
      .rotationEffect(.degrees(-guidance.rollDegrees))
      .animation(
        .interactiveSpring(response: 0.2, dampingFraction: 0.86),
        value: guidance.rollDegrees
      )
      .frame(width: size.width, height: size.height)
    }
    .allowsHitTesting(false)
  }

  private func scanLine(_ line: PanoramaScanLine, in size: CGSize) -> some View {
    let isActive = guidance.activeLine == line
    let isNext = guidance.isTransitioning && guidance.activeLine == line
    let width = size.width - 48
    let progress = isActive ? guidance.lineProgress : 0
    let pitchOffset = isActive
      ? CGFloat(
        ExperimentalPoseMath.arrowScreenYOffset(
          pitchErrorDegrees: guidance.pitchErrorDegrees,
          scaleDegrees: guidance.pitchGuideScaleDegrees,
          maxOffset: 48
        )
      )
      : 0
    let arrowX = CGFloat(progress) * (width - 28) - width / 2 + 14
    let accent = accentColor(for: line, isActive: isActive)

    return ZStack {
      Capsule()
        .fill(accent.opacity(isActive ? 0.95 : 0.32))
        .frame(width: width, height: isActive ? 3 : 1.5)

      if isActive || isNext {
        ZStack {
          Image(systemName: arrowSymbol)
            .font(.system(size: 18, weight: .bold))
            .foregroundStyle(accent)
            .shadow(color: .black.opacity(0.55), radius: 2, y: 1)
            .opacity(isCapturingPhoto && isActive ? 0.2 : 1)
          if isCapturingPhoto, isActive {
            ProgressView()
              .controlSize(.mini)
              .tint(.white)
          }
        }
        .offset(x: arrowX, y: pitchOffset)
        .animation(
          .interactiveSpring(response: 0.22, dampingFraction: 0.86),
          value: pitchOffset
        )
        .animation(
          .interactiveSpring(response: 0.28, dampingFraction: 0.9),
          value: progress
        )
      }
    }
    .frame(width: width, height: 56)
    .position(x: size.width / 2, y: size.height * line.guideYFraction)
    .opacity(isActive || isNext || guidance.activeLine == nil ? 1 : 0.35)
    .accessibilityLabel("\(line.displayName) scan line")
  }

  private var arrowSymbol: String {
    if guidance.isWrongDirection {
      return guidance.rotationDirection == .counterclockwise ? "arrow.right" : "arrow.left"
    }
    switch guidance.rotationDirection {
    case .counterclockwise:
      return "arrow.left"
    case .clockwise:
      return "arrow.right"
    case .automatic, .none:
      return "arrow.left.and.right"
    }
  }

  private func accentColor(for line: PanoramaScanLine, isActive: Bool) -> Color {
    if isActive, guidance.guideAccentIsBlocking {
      return .red
    }
    if isActive {
      return Color(red: 1, green: 0.84, blue: 0.08)
    }
    switch line {
    case .upward:
      return Color.mint
    case .horizontal:
      return Color.white
    case .downward:
      return Color.orange
    }
  }
}

extension PanoramaScanLine: Identifiable {
  var id: String { rawValue }
}

private extension PanoramaScanLine {
  static var guideOrder: [PanoramaScanLine] { [.upward, .horizontal, .downward] }

  var guideYFraction: CGFloat {
    switch self {
    case .upward: 0.38
    case .horizontal: 0.50
    case .downward: 0.62
    }
  }
}
