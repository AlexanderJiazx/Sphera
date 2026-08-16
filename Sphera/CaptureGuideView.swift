import SwiftUI

/// Point-based capture guide: center crosshair, large target dots, and a freely
/// rotating arrow that always points toward the nearest remaining capture point.
struct CapturePointGuideView: View {
  let points: [CapturePointProjection]
  let isCapturingPhoto: Bool

  /// Radians of visual angle mapped across half the shorter screen axis.
  private let radiansToHalfExtent: Double = 0.55

  var body: some View {
    GeometryReader { geometry in
      let size = geometry.size
      let center = CGPoint(x: size.width * 0.5, y: size.height * 0.5)
      let scale = min(size.width, size.height) * 0.5 / radiansToHalfExtent
      let nearest = points.min(by: { $0.directionErrorDegrees < $1.directionErrorDegrees })
      let isAligned = nearest?.isAligned == true

      ZStack {
        CrosshairView()
          .position(center)

        ForEach(visiblePoints(in: size, scale: scale, center: center)) { item in
          GuidePointDot(
            ring: item.point.ring,
            isActive: item.point.isAligned
              || item.point.targetID == nearest?.targetID,
            isCapturing: isCapturingPhoto && item.point.isAligned
          )
          .position(item.position)
        }

        if let nearest, !isAligned {
          AimingArrow(
            angleRadians: nearest.aimingAngleRadians
          )
          .position(center)
        }
      }
      .frame(width: size.width, height: size.height)
    }
    .allowsHitTesting(false)
  }

  private func visiblePoints(
    in size: CGSize,
    scale: Double,
    center: CGPoint
  ) -> [PositionedPoint] {
    let margin: CGFloat = 40
    return points.compactMap { point in
      guard point.isInFront else { return nil }
      let x = center.x + CGFloat(point.offsetX * scale)
      let y = center.y + CGFloat(point.offsetY * scale)
      guard x > -margin,
        y > -margin,
        x < size.width + margin,
        y < size.height + margin
      else {
        return nil
      }
      return PositionedPoint(point: point, position: CGPoint(x: x, y: y))
    }
  }
}

private struct PositionedPoint: Identifiable {
  var id: String { point.targetID }
  let point: CapturePointProjection
  let position: CGPoint
}

/// Continuous 360° arrow. `angleRadians` uses screen coords (+x right, +y down);
/// 0 points right. Shortest-path angle unwrapping prevents rapid 360° spinning
/// when crossing the ±π boundary.
private struct AimingArrow: View {
  let angleRadians: Double
  @State private var continuousAngle: Double = 0
  @State private var hasInitialized = false

  var body: some View {
    Image(systemName: "arrow.right")
      .font(.system(size: 78, weight: .black))
      .foregroundStyle(.white)
      .shadow(color: .black.opacity(0.55), radius: 4, y: 2)
      .offset(x: 58)
      .rotationEffect(.radians(continuousAngle))
      .animation(.interactiveSpring(response: 0.22, dampingFraction: 0.86), value: continuousAngle)
      .accessibilityLabel("Aim toward nearest point")
      .onAppear {
        continuousAngle = angleRadians
        hasInitialized = true
      }
      .onChange(of: angleRadians) { _, newAngle in
        guard hasInitialized else {
          continuousAngle = newAngle
          hasInitialized = true
          return
        }
        var delta = (newAngle - continuousAngle).truncatingRemainder(dividingBy: 2 * .pi)
        if delta > .pi { delta -= 2 * .pi }
        if delta < -.pi { delta += 2 * .pi }
        continuousAngle += delta
      }
  }
}

private struct CrosshairView: View {
  var body: some View {
    ZStack {
      Circle()
        .strokeBorder(.white.opacity(0.9), lineWidth: 2.5)
        .frame(width: 52, height: 52)
      Rectangle()
        .fill(.white.opacity(0.95))
        .frame(width: 2.5, height: 22)
      Rectangle()
        .fill(.white.opacity(0.95))
        .frame(width: 22, height: 2.5)
    }
    .shadow(color: .black.opacity(0.45), radius: 2, y: 1)
  }
}

private struct GuidePointDot: View {
  let ring: CaptureRing
  let isActive: Bool
  let isCapturing: Bool

  var body: some View {
    ZStack {
      Circle()
        .fill(ringColor.opacity(isActive ? 0.95 : 0.8))
        .frame(width: isActive ? 44 : 36, height: isActive ? 44 : 36)
      Circle()
        .strokeBorder(.white.opacity(0.95), lineWidth: isActive ? 3.5 : 2.5)
        .frame(width: isActive ? 44 : 36, height: isActive ? 44 : 36)
      if isCapturing {
        ProgressView()
          .controlSize(.regular)
          .tint(.white)
      }
    }
    .shadow(color: .black.opacity(0.45), radius: 4, y: 2)
    .accessibilityLabel("\(ring.displayName) target")
  }

  private var ringColor: Color {
    switch ring {
    case .horizontal: .cyan
    case .downward: .orange
    case .upward: .mint
    }
  }
}

extension View {
  @ViewBuilder
  func liquidGlass<S: Shape>(in shape: S) -> some View {
    if #available(iOS 26, *) {
      self.glassEffect(.regular, in: shape)
    } else {
      self.background(.ultraThinMaterial, in: shape)
    }
  }

  @ViewBuilder
  func liquidGlassInteractive<S: Shape>(in shape: S) -> some View {
    if #available(iOS 26, *) {
      self.glassEffect(.regular.interactive(), in: shape)
    } else {
      self.background(.ultraThinMaterial, in: shape)
    }
  }

  @ViewBuilder
  func liquidGlassWhiteRing(thickness: CGFloat) -> some View {
    if #available(iOS 26, *) {
      self.glassEffect(
        .regular.tint(.white).interactive(),
        in: GlassRingShape(thickness: thickness)
      )
    } else {
      self.overlay {
        GlassRingShape(thickness: thickness)
          .fill(.white.opacity(0.45), style: FillStyle(eoFill: true))
          .background {
            GlassRingShape(thickness: thickness)
              .fill(.ultraThinMaterial, style: FillStyle(eoFill: true))
          }
      }
    }
  }
}

/// Annular ring: outer CW arc + inner CCW arc so non-zero fill leaves a hole.
struct GlassRingShape: Shape {
  var thickness: CGFloat

  func path(in rect: CGRect) -> Path {
    let outerRadius = min(rect.width, rect.height) / 2
    let innerRadius = max(0, outerRadius - thickness)
    let center = CGPoint(x: rect.midX, y: rect.midY)

    var path = Path()
    path.addArc(
      center: center,
      radius: outerRadius,
      startAngle: .degrees(0),
      endAngle: .degrees(360),
      clockwise: false
    )
    path.addArc(
      center: center,
      radius: innerRadius,
      startAngle: .degrees(0),
      endAngle: .degrees(360),
      clockwise: true
    )
    path.closeSubpath()
    return path
  }
}

struct CaptureSegmentProgress: View {
  let total: Int
  let capturedCount: Int

  var body: some View {
    HStack(spacing: 3) {
      ForEach(0..<max(total, 0), id: \.self) { index in
        segment(at: index)
      }
    }
    .frame(height: 8)
    .accessibilityElement(children: .ignore)
    .accessibilityLabel("Capture progress, \(capturedCount) of \(total) photos")
  }

  private func segment(at index: Int) -> some View {
    let captured = index < capturedCount
    let current = index == capturedCount
    let shape = RoundedRectangle(cornerRadius: 2.5, style: .continuous)
    return shape
      .fill(captured ? Color.blue : Color.clear)
      .overlay {
        shape.strokeBorder(
          current ? Color.white : Color.white.opacity(captured ? 0 : 0.28),
          lineWidth: current ? 1.6 : 1
        )
      }
      .frame(maxWidth: .infinity)
      .animation(.easeInOut(duration: 0.2), value: captured)
      .animation(.easeInOut(duration: 0.2), value: current)
  }
}

struct ElasticGlassCaptureButtonStyle: ButtonStyle {
  func makeBody(configuration: Configuration) -> some View {
    configuration.label
      .scaleEffect(configuration.isPressed ? 0.84 : 1)
      .animation(
        .spring(response: 0.26, dampingFraction: 0.48, blendDuration: 0),
        value: configuration.isPressed
      )
  }
}

extension ButtonStyle where Self == ElasticGlassCaptureButtonStyle {
  static var elasticGlassCapture: ElasticGlassCaptureButtonStyle {
    ElasticGlassCaptureButtonStyle()
  }
}

struct GlassStopButtonStyle: ButtonStyle {
  func makeBody(configuration: Configuration) -> some View {
    configuration.label
      .font(.subheadline.weight(.semibold))
      .padding(.horizontal, 14)
      .padding(.vertical, 8)
      .liquidGlassInteractive(in: Capsule())
      .scaleEffect(configuration.isPressed ? 0.94 : 1)
      .opacity(configuration.isPressed ? 0.8 : 1)
      .animation(
        .spring(response: 0.28, dampingFraction: 0.55),
        value: configuration.isPressed
      )
  }
}

extension ButtonStyle where Self == GlassStopButtonStyle {
  static var glassStop: GlassStopButtonStyle { GlassStopButtonStyle() }
}
