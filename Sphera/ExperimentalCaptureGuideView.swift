import SwiftUI
import UIKit

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
        "Capture Mode",
        selection: Binding(
          get: { mode },
          set: { onSelect($0) }
        )
      ) {
        ForEach(CaptureSessionMode.allCases, id: \.self) { sessionMode in
          Label(sessionMode.menuTitle, systemImage: sessionMode.symbolName)
            .tag(sessionMode)
        }
      }
    } label: {
      HStack(spacing: 6) {
        Image(systemName: mode.symbolName)
          .font(.subheadline.weight(.semibold))
        Text(mode.title)
          .font(.subheadline.weight(.semibold))
      }
      .foregroundStyle(.white)
      .padding(.horizontal, 14)
      .padding(.vertical, 8)
      .liquidGlassInteractive(in: Capsule())
    }
    .contentShape(Capsule())
    .frame(minHeight: 44)
    .disabled(!isEnabled)
    .opacity(isEnabled ? 1 : 0.4)
    .accessibilityLabel(mode.accessibilityLabel)
    .accessibilityHint("Changes how Sphera captures a panorama")
  }
}

// MARK: - Sweep guides

/// A level indicator in the spirit of the Camera app's: a fixed target line
/// and a moving line that carries the phone's tilt and roll. They merge and
/// turn yellow when the phone is on the row.
struct ExperimentalLevelGuide: View, Equatable {
  let pitchErrorDegrees: Double
  let rollDegrees: Double
  let guideScaleDegrees: Double
  let isAligned: Bool
  let isBlocked: Bool
  let holdFraction: Double

  private let lineWidth: CGFloat = 128
  private let maxOffset: Double = 96

  static func == (lhs: Self, rhs: Self) -> Bool {
    lhs.pitchErrorDegrees == rhs.pitchErrorDegrees
      && lhs.rollDegrees == rhs.rollDegrees
      && lhs.isAligned == rhs.isAligned
      && lhs.isBlocked == rhs.isBlocked
      && lhs.holdFraction == rhs.holdFraction
  }

  var body: some View {
    let offset = ExperimentalPoseMath.arrowScreenYOffset(
      pitchErrorDegrees: pitchErrorDegrees,
      scaleDegrees: guideScaleDegrees,
      maxOffset: maxOffset
    )

    ZStack {
      // Where the row wants the phone to be.
      Capsule()
        .fill(.white.opacity(isAligned ? 0 : 0.5))
        .frame(width: lineWidth, height: 2)
        .shadow(color: .black.opacity(0.4), radius: 2)

      // Where the phone actually is.
      Capsule()
        .fill(tint)
        .frame(width: lineWidth, height: isAligned ? 3 : 2)
        .rotationEffect(.degrees(-rollDegrees))
        .offset(y: CGFloat(offset))
        .shadow(color: .black.opacity(0.4), radius: 2)

      // Fills while the phone sits on the row, so the wait before a row starts
      // is visible rather than mysterious.
      if holdFraction > 0 {
        Circle()
          .trim(from: 0, to: holdFraction)
          .stroke(tint, style: StrokeStyle(lineWidth: 2.5, lineCap: .round))
          .rotationEffect(.degrees(-90))
          .frame(width: 44, height: 44)
          .offset(y: CGFloat(offset))
      }
    }
    .frame(height: 240)
    .animation(.interactiveSpring(response: 0.24, dampingFraction: 0.86), value: offset)
    .animation(.interactiveSpring(response: 0.24, dampingFraction: 0.86), value: rollDegrees)
    .animation(.easeOut(duration: 0.15), value: isAligned)
    .accessibilityHidden(true)
  }

  private var tint: Color {
    if isBlocked { return .white }
    return isAligned ? .yellow : .white
  }
}

/// Progress around the row. The live slit-scan sits behind evenly spaced ticks
/// so the user can see both what has been photographed and where they are.
struct ExperimentalCoverageTrack: View {
  let targetStates: [ExperimentalTargetState]
  let sweepFraction: Double
  let isDimmed: Bool
  let strip: UIImage?

  private let height: CGFloat = 56

  var body: some View {
    GeometryReader { geometry in
      let width = geometry.size.width
      let shape = RoundedRectangle(cornerRadius: 14, style: .continuous)

      ZStack(alignment: .leading) {
        shape.fill(.black.opacity(0.35))

        if let strip {
          Image(uiImage: strip)
            .resizable()
            .frame(width: width, height: height)
            .clipShape(shape)
            .opacity(0.9)
        }

        HStack(spacing: 0) {
          ForEach(Array(targetStates.enumerated()), id: \.offset) { _, state in
            Capsule()
              .fill(color(for: state))
              .frame(width: 3, height: 12)
              .frame(maxWidth: .infinity)
          }
        }
        .padding(.horizontal, 4)
        .frame(maxHeight: .infinity, alignment: .bottom)
        .padding(.bottom, 6)

        Capsule()
          .fill(.white)
          .frame(width: 3, height: height - 12)
          .shadow(color: .black.opacity(0.5), radius: 2)
          .offset(x: min(max(CGFloat(sweepFraction) * width, 2), width - 5))
          .animation(.interactiveSpring(response: 0.3, dampingFraction: 0.9), value: sweepFraction)
      }
      .overlay {
        shape.strokeBorder(.white.opacity(0.14), lineWidth: 1)
      }
      .clipShape(shape)
    }
    .frame(height: height)
    .opacity(isDimmed ? 0.6 : 1)
    .animation(.easeInOut(duration: 0.15), value: isDimmed)
    .accessibilityHidden(true)
  }

  private func color(for state: ExperimentalTargetState) -> Color {
    switch state {
    case .captured: .yellow
    case .skipped: .white.opacity(0.25)
    case .pending: .white.opacity(0.5)
    }
  }
}

/// The short "fix this" badge. Deliberately small and centered so it reads
/// like the Camera app's own hints rather than an error dialog.
struct ExperimentalAlertBadge: View {
  let text: String

  var body: some View {
    Text(text)
      .font(.subheadline.weight(.semibold))
      .foregroundStyle(.black)
      .padding(.horizontal, 14)
      .padding(.vertical, 7)
      .background(.yellow, in: Capsule())
      .accessibilityHidden(true)
  }
}
