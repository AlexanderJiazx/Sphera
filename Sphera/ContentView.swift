import SwiftUI

struct ContentView: View {
  @StateObject private var model = CaptureViewModel()
  @State private var selectedTab = 0
  @State private var handledDebugLaunchArguments = false

  var body: some View {
    TabView(selection: $selectedTab) {
      CaptureTabView(model: model, isSelected: selectedTab == 0)
        .tabItem {
          Image(systemName: "camera.fill")
        }
        .accessibilityLabel("Capture")
        .tag(0)

      NavigationStack {
        GalleryView(model: model)
      }
      .tabItem {
        Image(systemName: "photo.on.rectangle.angled")
      }
      .accessibilityLabel("Gallery")
      .tag(1)
    }
    .preferredColorScheme(.dark)
    .onChange(of: selectedTab, initial: true) { _, tab in
      model.setCaptureTabActive(tab == 0)
    }
    .onAppear {
      #if DEBUG
        guard !handledDebugLaunchArguments else { return }
        handledDebugLaunchArguments = true
        let args = ProcessInfo.processInfo.arguments
        if args.contains("--auto-start-capture") {
          selectedTab = 0
          model.startCapture()
        }
        if args.contains("--recompute-first-gallery") {
          selectedTab = 1
          Task {
            await model.refreshGallery()
            guard let package = model.galleryPackages.first else {
              NSLog("LoFTR e2e: no gallery package found")
              return
            }
            NSLog("LoFTR e2e: recomputing %@", package.directoryURL.lastPathComponent)
            await model.computeOnDevice(package: package, replaceExisting: true)
            NSLog(
              "LoFTR e2e: finished phase=%@ status=%@",
              String(describing: model.phase),
              model.statusMessage
            )
          }
        }
      #endif
    }
    .overlay {
      if selectedTab != 0 {
        switch model.phase {
        case .stitching:
          ZStack {
            Color.black.opacity(0.55).ignoresSafeArea()
            StatusView(
              title: "Computing panorama",
              detail: model.statusMessage
            )
          }
        default:
          EmptyView()
        }
      }
    }
  }
}

private struct CaptureTabView: View {
  @ObservedObject var model: CaptureViewModel
  var isSelected: Bool

  var body: some View {
    Group {
      switch model.phase {
      case .setup, .preparing:
        StatusView(title: "Preparing capture", detail: model.statusMessage)
      case .awaitingPrimary, .capturingPoints:
        CaptureScreen(model: model)
      case .saved(let package):
        SavedCaptureView(package: package, model: model)
      case .stitching:
        StatusView(
          title: "Computing panorama",
          detail: model.statusMessage
        )
      case .failed(let message):
        FailureView(message: message) {
          model.returnToSetup()
        }
      }
    }
    .onChange(of: isSelected) { _, selected in
      if selected {
        switch model.phase {
        case .setup, .saved, .failed:
          model.returnToSetup()
          model.startCapture()
        default:
          break
        }
      }
    }
    .onChange(of: model.phase) { _, phase in
      if isSelected, phase == .setup {
        model.startCapture()
      }
    }
  }
}

private struct CaptureScreen: View {
  @ObservedObject var model: CaptureViewModel

  private var isAwaitingPrimary: Bool {
    model.phase == .awaitingPrimary
  }

  var body: some View {
    ZStack {
      Color.black.ignoresSafeArea()

      // Live camera preview with native UIVisualEffectView blur when connecting/resetting
      CameraPreviewView(session: model.camera.session, isSourceReady: model.isCameraSourceReady)
        .ignoresSafeArea()
        .allowsHitTesting(false)

      if !isAwaitingPrimary {
        CapturePointGuideView(
          points: model.guidePoints,
          isCapturingPhoto: model.isCapturingPhoto
        )
        .ignoresSafeArea()
        .allowsHitTesting(false)
      }

      VStack(spacing: 0) {
        if isAwaitingPrimary {
          primaryHeaderBar
        } else {
          topPanel
        }

        Spacer()

        if let error = model.captureErrorMessage {
          Text(error)
            .font(.callout.weight(.semibold))
            .foregroundStyle(.white)
            .multilineTextAlignment(.center)
            .frame(maxWidth: .infinity)
            .padding(12)
            .background(.red.opacity(0.82), in: RoundedRectangle(cornerRadius: 12, style: .continuous))
            .padding(.horizontal, 16)
            .padding(.bottom, 12)
        }

        if isAwaitingPrimary {
          VStack(spacing: 12) {
            if model.cameraMode == .manual {
              if let selectedParam = model.selectedManualParameter {
                AppleCameraDialPanel(model: model, parameter: selectedParam)
                  .id(selectedParam)
                  .transition(.asymmetric(
                    insertion: .opacity.combined(with: .scale(scale: 0.96)),
                    removal: .opacity
                  ))
              }
              proCameraParameterBar
            }
            primaryCaptureControls
          }
        } else if !model.autoFireCapture && model.navigationReading.isAligned {
          manualAlignedCaptureControls
        }
      }
      .padding(.vertical, 10)
    }
  }

  /// Top-left mode selector at the beginning of capture
  private var primaryHeaderBar: some View {
    HStack {
      Menu {
        Picker("Mode", selection: Binding(
          get: { model.cameraMode },
          set: { newMode in
            withAnimation(.easeInOut(duration: 0.2)) {
              model.setCameraMode(newMode)
            }
          }
        )) {
          ForEach(CameraMode.allCases, id: \.self) { mode in
            Label(
              mode.rawValue,
              systemImage: mode == .auto ? "camera.aperture" : "slider.horizontal.3"
            )
            .tag(mode)
          }
        }
      } label: {
        HStack(spacing: 6) {
          Image(systemName: model.cameraMode == .auto ? "camera.aperture" : "slider.horizontal.3")
            .font(.subheadline.weight(.semibold))
          Text(model.cameraMode.rawValue)
            .font(.subheadline.weight(.semibold))
          Image(systemName: "chevron.down")
            .font(.caption2.weight(.bold))
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 14)
        .padding(.vertical, 8)
        .liquidGlassInteractive(in: Capsule())
      }
      .contentShape(Capsule())
      .frame(minHeight: 44)

      Spacer()
    }
    .padding(.horizontal, 16)
    .padding(.top, 4)
  }

  private var topPanel: some View {
    VStack(spacing: 8) {
      HStack {
        VStack(alignment: .leading, spacing: 2) {
          Text("PHOTO \(model.capturedFrames.count) OF \(model.totalFrameCount)")
            .font(.headline)
          if let target = model.activeTarget {
            Text(
              "\(target.ring.displayName.uppercased()) · \(model.remainingTargets.count) LEFT"
            )
            .font(.caption.weight(.semibold))
            .foregroundStyle(.secondary)
          } else {
            Text("FOLLOW THE ARROW TO A POINT")
              .font(.caption.weight(.semibold))
              .foregroundStyle(.secondary)
          }
        }
        Spacer()
        Button("Reset") {
          model.resetCapture()
        }
        .buttonStyle(.glassStop)
      }
      CaptureSegmentProgress(
        total: model.totalFrameCount,
        capturedCount: model.capturedFrames.count
      )
    }
    .padding(14)
    .liquidGlass(in: RoundedRectangle(cornerRadius: 18, style: .continuous))
    .padding(.horizontal, 16)
  }

  private var proCameraParameterBar: some View {
    HStack(spacing: 10) {
      parameterChip(
        param: .shutter,
        title: "S",
        valueText: model.isShutterAuto ? "AUTO" : formattedShutterSpeed(model.manualExposureDurationSeconds),
        isAuto: model.isShutterAuto
      )
      parameterChip(
        param: .iso,
        title: "ISO",
        valueText: model.isISOAuto ? "AUTO" : "\(Int(model.manualISO))",
        isAuto: model.isISOAuto
      )
      parameterChip(
        param: .focus,
        title: "AF",
        valueText: model.isFocusAuto ? "AUTO" : formattedFocus(model.manualFocusLensPosition),
        isAuto: model.isFocusAuto
      )
      parameterChip(
        param: .whiteBalance,
        title: "WB",
        valueText: model.isWhiteBalanceAuto ? "AUTO" : "\(Int(model.manualTemperatureKelvin))K",
        isAuto: model.isWhiteBalanceAuto
      )
    }
    .padding(.horizontal, 16)
  }

  private func parameterChip(
    param: ManualParameter,
    title: String,
    valueText: String,
    isAuto: Bool
  ) -> some View {
    let isSelected = model.selectedManualParameter == param

    return Button {
      withAnimation(.spring(response: 0.28, dampingFraction: 0.82)) {
        model.selectManualParameter(param)
      }
      UISelectionFeedbackGenerator().selectionChanged()
    } label: {
      VStack(spacing: 2) {
        Text(title)
          .font(.system(size: 11, weight: .bold, design: .monospaced))
          .foregroundStyle(isSelected ? .white : (isAuto ? .white.opacity(0.85) : .white.opacity(0.95)))
        Text(valueText)
          .font(.system(size: 12, weight: .bold, design: .monospaced))
          .foregroundStyle(isSelected ? .white : (isAuto ? .white.opacity(0.7) : .white.opacity(0.85)))
      }
      .frame(maxWidth: .infinity)
      .frame(minHeight: 44)
      .liquidGlass(in: RoundedRectangle(cornerRadius: 14, style: .continuous))
      .overlay {
        if isSelected {
          RoundedRectangle(cornerRadius: 14, style: .continuous)
            .strokeBorder(Color.white.opacity(0.8), lineWidth: 1.5)
        }
      }
      .background {
        if isSelected {
          RoundedRectangle(cornerRadius: 14, style: .continuous)
            .fill(Color.white.opacity(0.18))
        }
      }
    }
    .contentShape(RoundedRectangle(cornerRadius: 14, style: .continuous))
    .buttonStyle(.plain)
  }

  private var primaryCaptureControls: some View {
    Button {
      model.capturePrimary()
    } label: {
      ZStack {
        Color.clear
          .frame(width: 84, height: 84)
          .liquidGlassInteractive(in: Circle())

        Color.clear
          .frame(width: 58, height: 58)
          .liquidGlassWhiteRing(thickness: 8)
      }
      .contentShape(Circle())
    }
    .buttonStyle(.elasticGlassCapture)
    .disabled(model.isCapturingPhoto)
    .opacity(model.isCapturingPhoto ? 0.55 : 1)
    .accessibilityLabel("Capture primary photo")
    .padding(.bottom, 24)
  }

  private var manualAlignedCaptureControls: some View {
    Button {
      model.captureCurrentAlignedTarget()
    } label: {
      ZStack {
        Circle()
          .fill(.cyan.opacity(0.35))
          .frame(width: 80, height: 80)
          .liquidGlassInteractive(in: Circle())

        Image(systemName: "camera.fill")
          .font(.system(size: 28, weight: .bold))
          .foregroundStyle(.white)
      }
      .contentShape(Circle())
    }
    .buttonStyle(.elasticGlassCapture)
    .disabled(model.isCapturingPhoto)
    .transition(.scale.combined(with: .opacity))
    .accessibilityLabel("Capture photo")
    .padding(.bottom, 24)
  }

  private func formattedShutterSpeed(_ seconds: Double) -> String {
    if seconds <= 0 { return "Auto" }
    if seconds < 1 {
      let denom = Int(round(1.0 / seconds))
      return "1/\(denom)"
    }
    return String(format: "%.1fs", seconds)
  }

  private func formattedFocus(_ position: Float) -> String {
    if position <= 0.05 { return "Macro" }
    if position >= 0.95 { return "∞" }
    return "\(Int(position * 100))%"
  }
}

/// Native Apple Camera app dial with vertical tick markings and center yellow needle
private struct AppleCameraDialPanel: View {
  @ObservedObject var model: CaptureViewModel
  let parameter: ManualParameter

  private let needleAmber = Color(red: 1.0, green: 0.84, blue: 0.04)
  private let tickSpacing: CGFloat = 20

  @State private var dragOffset: CGFloat = 0
  @State private var isDragging: Bool = false
  @State private var dragStartStopIndex: Int = 0
  @State private var lastFeedbackIndex: Int = 0

  private static let shutterStops: [Double] = [
    1.0/8000, 1.0/4000, 1.0/2000, 1.0/1000, 1.0/500, 1.0/250,
    1.0/125, 1.0/60, 1.0/30, 1.0/15, 1.0/8, 1.0/4, 1.0/2, 1.0
  ]
  private static let isoStops: [Float] = [
    50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200
  ]
  private static let focusStops: [Float] = [
    0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0
  ]
  private static let wbStops: [Float] = [
    2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000
  ]

  var body: some View {
    VStack(spacing: 14) {
      // Header row: circular back button, centered tracked uppercase title, and auto pill
      HStack {
        Button {
          withAnimation(.spring(response: 0.28, dampingFraction: 0.82)) {
            model.selectedManualParameter = nil
          }
        } label: {
          Image(systemName: "chevron.left")
            .font(.system(size: 14, weight: .bold))
            .foregroundStyle(.white)
            .frame(width: 36, height: 36)
            .liquidGlass(in: Circle())
        }
        .contentShape(Circle())
        .buttonStyle(.plain)

        Spacer()

        Text(parameterTitle.uppercased())
          .font(.system(size: 13, weight: .semibold))
          .tracking(1.8)
          .foregroundStyle(.white)

        Spacer()

        Button {
          withAnimation(.easeInOut(duration: 0.18)) {
            toggleCurrentAuto()
          }
          UIImpactFeedbackGenerator(style: .light).impactOccurred()
        } label: {
          Text("AUTO")
            .font(.system(size: 11, weight: .bold, design: .monospaced))
            .foregroundStyle(isCurrentAuto ? .black : .white)
            .padding(.horizontal, 9)
            .padding(.vertical, 5)
            .background {
              Capsule()
                .fill(isCurrentAuto ? Color.white : Color.white.opacity(0.15))
            }
        }
        .contentShape(Capsule())
        .buttonStyle(.plain)
      }
      .padding(.horizontal, 4)

      // Prominent value readout
      Text(currentValueFormatted)
        .font(.system(size: 38, weight: .medium, design: .monospaced))
        .foregroundStyle(.white)
        .contentTransition(.numericText())

      // Native Apple camera graduated tick dial ruler
      dialRuler
    }
    .padding(.horizontal, 20)
    .padding(.vertical, 16)
    .liquidGlass(in: RoundedRectangle(cornerRadius: 32, style: .continuous))
    .padding(.horizontal, 16)
    .shadow(color: .black.opacity(0.55), radius: 18, y: 8)
    .onAppear {
      resetState()
    }
    .onChange(of: parameter) { _ in
      resetState()
    }
  }

  private func resetState() {
    isDragging = false
    dragOffset = 0
    let (idx, _) = currentStopIndexAndCount
    dragStartStopIndex = idx
    lastFeedbackIndex = idx
  }

  private var activeFractionalIndex: CGFloat {
    let (currentIndex, totalStops) = currentStopIndexAndCount
    guard totalStops > 0 else { return 0 }
    if isDragging {
      let raw = CGFloat(dragStartStopIndex) - (dragOffset / tickSpacing)
      if raw < 0 {
        return -log1p(max(0, -raw) * 0.5) * 0.6
      } else if raw > CGFloat(totalStops - 1) {
        let over = raw - CGFloat(totalStops - 1)
        return CGFloat(totalStops - 1) + log1p(max(0, over) * 0.5) * 0.6
      } else {
        return raw
      }
    } else {
      return CGFloat(min(max(currentIndex, 0), totalStops - 1))
    }
  }

  private var dialRuler: some View {
    GeometryReader { geo in
      let width = geo.size.width
      let center = width / 2
      let totalStops = currentStopIndexAndCount.count
      let activeIndex = activeFractionalIndex

      ZStack {
        // Graduated tick marks along the ruler (white)
        ForEach(0..<totalStops, id: \.self) { i in
          let tickX = center + (CGFloat(i) - activeIndex) * tickSpacing
          let distFromCenter = abs(tickX - center)

          if distFromCenter <= (width / 2 + 30) {
            let opacity = max(0.0, 1.0 - (distFromCenter / (width * 0.48)))
            let isMajor = (i % 3 == 0)

            Capsule()
              .fill(Color.white.opacity(0.85))
              .frame(
                width: 1.8,
                height: isMajor ? 18 : 12
              )
              .position(x: tickX, y: 16)
              .opacity(opacity)
          }
        }

        // Center yellow indicator needle
        Capsule()
          .fill(needleAmber)
          .frame(width: 3, height: 26)
          .position(x: center, y: 16)
      }
      .frame(height: 32)
      .contentShape(Rectangle())
      .gesture(
        DragGesture(minimumDistance: 1)
          .onChanged { value in
            if !isDragging {
              isDragging = true
              dragStartStopIndex = currentStopIndexAndCount.index
              lastFeedbackIndex = dragStartStopIndex
            }
            dragOffset = value.translation.width

            let rawIndex = CGFloat(dragStartStopIndex) - (dragOffset / tickSpacing)
            let clampedIndex = min(max(Int(round(rawIndex)), 0), totalStops - 1)

            if clampedIndex != lastFeedbackIndex {
              lastFeedbackIndex = clampedIndex
              applyStop(at: clampedIndex)
              UISelectionFeedbackGenerator().selectionChanged()
            }
          }
          .onEnded { value in
            let totalDrag = value.translation.width
            if abs(totalDrag) < 3 {
              let tapOffset = value.location.x - center
              let tapDelta = Int(round(tapOffset / tickSpacing))
              let targetIndex = min(max(currentStopIndexAndCount.index + tapDelta, 0), totalStops - 1)
              withAnimation(.spring(response: 0.28, dampingFraction: 0.82)) {
                applyStop(at: targetIndex)
              }
            } else {
              let finalFractional = CGFloat(dragStartStopIndex) - (totalDrag / tickSpacing)
              let finalIndex = min(max(Int(round(finalFractional)), 0), totalStops - 1)
              withAnimation(.spring(response: 0.28, dampingFraction: 0.82)) {
                applyStop(at: finalIndex)
              }
            }
            dragOffset = 0
            isDragging = false
            lastFeedbackIndex = currentStopIndexAndCount.index
            UISelectionFeedbackGenerator().selectionChanged()
          }
      )
    }
    .frame(height: 34)
    .padding(.bottom, 2)
  }

  // MARK: - Helpers

  private var parameterTitle: String {
    switch parameter {
    case .shutter: return "Exposure"
    case .iso: return "ISO"
    case .focus: return "Focus"
    case .whiteBalance: return "White Balance"
    }
  }

  private var isCurrentAuto: Bool {
    switch parameter {
    case .shutter: return model.isShutterAuto
    case .iso: return model.isISOAuto
    case .focus: return model.isFocusAuto
    case .whiteBalance: return model.isWhiteBalanceAuto
    }
  }

  private func toggleCurrentAuto() {
    switch parameter {
    case .shutter:
      model.isShutterAuto.toggle()
    case .iso:
      model.isISOAuto.toggle()
    case .focus:
      model.isFocusAuto.toggle()
    case .whiteBalance:
      model.isWhiteBalanceAuto.toggle()
    }
  }

  private var currentStopIndexAndCount: (index: Int, count: Int) {
    switch parameter {
    case .shutter:
      let stops = Self.shutterStops
      let current = model.manualExposureDurationSeconds
      let closest = stops.enumerated().min(by: { abs($0.element - current) < abs($1.element - current) })?.offset ?? 0
      return (closest, stops.count)
    case .iso:
      let stops = Self.isoStops
      let current = model.manualISO
      let closest = stops.enumerated().min(by: { abs($0.element - current) < abs($1.element - current) })?.offset ?? 0
      return (closest, stops.count)
    case .focus:
      let stops = Self.focusStops
      let current = model.manualFocusLensPosition
      let closest = stops.enumerated().min(by: { abs($0.element - current) < abs($1.element - current) })?.offset ?? 0
      return (closest, stops.count)
    case .whiteBalance:
      let stops = Self.wbStops
      let current = model.manualTemperatureKelvin
      let closest = stops.enumerated().min(by: { abs($0.element - current) < abs($1.element - current) })?.offset ?? 0
      return (closest, stops.count)
    }
  }

  private func applyStop(at index: Int) {
    switch parameter {
    case .shutter:
      let stops = Self.shutterStops
      guard index >= 0 && index < stops.count else { return }
      model.isShutterAuto = false
      model.manualExposureDurationSeconds = stops[index]
    case .iso:
      let stops = Self.isoStops
      guard index >= 0 && index < stops.count else { return }
      model.isISOAuto = false
      model.manualISO = stops[index]
    case .focus:
      let stops = Self.focusStops
      guard index >= 0 && index < stops.count else { return }
      model.isFocusAuto = false
      model.manualFocusLensPosition = stops[index]
    case .whiteBalance:
      let stops = Self.wbStops
      guard index >= 0 && index < stops.count else { return }
      model.isWhiteBalanceAuto = false
      model.manualTemperatureKelvin = stops[index]
    }
  }

  private var currentValueFormatted: String {
    switch parameter {
    case .shutter:
      if model.isShutterAuto { return "AUTO" }
      let sec = model.manualExposureDurationSeconds
      if sec < 1 {
        let denom = Int(round(1.0 / sec))
        return "1/\(denom)"
      }
      return String(format: "%.1fs", sec)
    case .iso:
      return model.isISOAuto ? "AUTO" : "\(Int(model.manualISO))"
    case .focus:
      if model.isFocusAuto { return "AUTO" }
      let pos = model.manualFocusLensPosition
      if pos <= 0.05 { return "MACRO" }
      if pos >= 0.95 { return "∞" }
      return "\(Int(round(pos * 100)))%"
    case .whiteBalance:
      if model.isWhiteBalanceAuto { return "AUTO" }
      let k = Int(round(model.manualTemperatureKelvin))
      return "\(k)K"
    }
  }
}

private struct CaptureSegmentProgress: View {
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

private struct ElasticGlassCaptureButtonStyle: ButtonStyle {
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
  fileprivate static var elasticGlassCapture: ElasticGlassCaptureButtonStyle {
    ElasticGlassCaptureButtonStyle()
  }
}

private struct GlassStopButtonStyle: ButtonStyle {
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
  fileprivate static var glassStop: GlassStopButtonStyle { GlassStopButtonStyle() }
}

private struct SavedCaptureView: View {
  let package: CapturePackage
  @ObservedObject var model: CaptureViewModel
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

  var body: some View {
    NavigationStack {
      VStack(spacing: 20) {
        Spacer()
        Image(systemName: "checkmark.circle.fill")
          .font(.system(size: 52))
          .foregroundStyle(.green)
        Text("Capture saved")
          .font(.title2.bold())
        Text(
          "\(package.manifest.frames.count) frames with camera, pose, and gyroscope data are in the gallery. Compute on device when you want a panorama."
        )
        .font(.body)
        .foregroundStyle(.secondary)
        .multilineTextAlignment(.center)
        .padding(.horizontal)

        VStack(spacing: 12) {
          Button {
            Task {
              await model.computeOnDevice(package: package)
            }
          } label: {
            Label("Compute on device", systemImage: "cpu")
              .frame(maxWidth: .infinity)
          }
          .buttonStyle(.borderedProminent)

          Button {
            Task { await presentShare() }
          } label: {
            Label(
              isPreparingShare ? "Preparing archive…" : "Share capture archive",
              systemImage: "square.and.arrow.up"
            )
            .frame(maxWidth: .infinity)
          }
          .buttonStyle(.bordered)
          .disabled(isPreparingShare)

          Button("New capture") {
            model.returnToSetup()
          }
          .buttonStyle(.bordered)
          .frame(maxWidth: .infinity)
        }
        .padding(.horizontal, 24)

        Spacer()
      }
      .navigationTitle("Saved")
      .navigationBarTitleDisplayMode(.inline)
      .sheet(item: $sharePayload) { payload in
        ActivityView(activityItems: [payload.url])
      }
      .alert(
        "Share failed",
        isPresented: Binding(
          get: { shareErrorMessage != nil },
          set: { if !$0 { shareErrorMessage = nil } }
        )
      ) {
        Button("OK", role: .cancel) {}
      } message: {
        Text(shareErrorMessage ?? "")
      }
    }
  }

  @MainActor
  private func presentShare() async {
    isPreparingShare = true
    defer { isPreparingShare = false }
    do {
      let url = try await model.makeShareArchive(for: package)
      sharePayload = SharePayload(url: url)
    } catch {
      shareErrorMessage = error.localizedDescription
    }
  }
}

private struct StatusView: View {
  let title: String
  let detail: String

  var body: some View {
    VStack(spacing: 16) {
      ProgressView()
        .controlSize(.large)
      Text(title)
        .font(.title2.bold())
      Text(detail)
        .font(.body)
        .foregroundStyle(.primary)
        .multilineTextAlignment(.center)
        .contentTransition(.opacity)
        .animation(.easeInOut(duration: 0.2), value: detail)
    }
    .padding(30)
  }
}


private struct FailureView: View {
  let message: String
  let dismiss: () -> Void

  var body: some View {
    VStack(spacing: 16) {
      Image(systemName: "exclamationmark.triangle")
        .font(.system(size: 42))
        .foregroundStyle(.orange)
      Text("Capture unavailable")
        .font(.title2.bold())
      Text(message)
        .multilineTextAlignment(.center)
        .foregroundStyle(.secondary)
      Button("Try again", action: dismiss)
        .buttonStyle(.borderedProminent)
    }
    .padding(30)
  }
}

#Preview {
  ContentView()
}
