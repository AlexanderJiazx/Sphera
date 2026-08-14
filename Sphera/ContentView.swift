import SwiftUI

struct ContentView: View {
  @StateObject private var model = CaptureViewModel()
  @State private var selectedTab = 0
  @State private var handledDebugLaunchArguments = false

  var body: some View {
    TabView(selection: $selectedTab) {
      Tab("Capture", systemImage: "camera.fill", value: 0) {
        CaptureTabView(model: model, isSelected: selectedTab == 0)
      }

      Tab("Gallery", systemImage: "photo.on.rectangle.angled", value: 1) {
        NavigationStack {
          GalleryView(model: model)
        }
      }

      Tab("Settings", systemImage: "gearshape.fill", value: 2) {
        NavigationStack {
          SettingsView(model: model)
        }
      }
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
      case .completed(let completion):
        CaptureResultView(completion: completion) {
          model.returnToSetup()
        }
      case .failed(let message):
        FailureView(message: message) {
          model.returnToSetup()
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

private struct SettingsView: View {
  @ObservedObject var model: CaptureViewModel

  var body: some View {
    Form {
      Section {
        Stepper(value: $model.configuration.horizontalCount, in: 4...16) {
          countRow("Horizontal ring", model.configuration.horizontalCount)
        }
        Stepper(value: $model.configuration.downwardCount, in: 4...6) {
          countRow("Downward ring", model.configuration.downwardCount)
        }
        Stepper(value: $model.configuration.upwardCount, in: 4...6) {
          countRow("Upward ring", model.configuration.upwardCount)
        }
        countRow("Total frames", model.configuration.totalImageCount)
      } header: {
        Text("Capture preset")
      } footer: {
        Text("Changes apply the next time a capture session starts.")
      }

      Section {
        Toggle("Experimental Metal stitch", isOn: $model.useExperimentalMetalStitch)
      } header: {
        Text("On-device compute")
      } footer: {
        Text("Off by default. When off, Compute on device uses the stable OpenCV engine. When on, it uses the experimental Swift/Metal engine (no OpenCV).")
      }
    }
    .navigationTitle("Settings")
    .onChange(of: model.configuration) {
      model.rebuildPlan()
    }
  }

  private func countRow(_ title: String, _ count: Int) -> some View {
    HStack {
      Text(title)
      Spacer()
      Text(count, format: .number)
        .monospacedDigit()
        .foregroundStyle(.secondary)
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
      CameraPreviewView(session: model.camera.session)
        .ignoresSafeArea()

      if !isAwaitingPrimary {
        CapturePointGuideView(
          points: model.guidePoints,
          isCapturingPhoto: model.isCapturingPhoto
        )
        .ignoresSafeArea()
      }

      VStack(spacing: 0) {
        topPanel
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
        }
        if isAwaitingPrimary {
          primaryCaptureControls
        }
      }
      .padding(.vertical, 10)
    }
  }

  private var topPanel: some View {
    VStack(spacing: 8) {
      HStack {
        VStack(alignment: .leading, spacing: 2) {
          if isAwaitingPrimary {
            Text("PRIMARY CAPTURE")
              .font(.headline)
            Text("SETS EXPOSURE, FOCUS, AND ORIGIN")
              .font(.caption.weight(.semibold))
              .foregroundStyle(.secondary)
          } else {
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
        }
        Spacer()
        if !isAwaitingPrimary {
          Button("Reset") {
            model.resetCapture()
          }
          .buttonStyle(.glassStop)
        }
      }
      if !isAwaitingPrimary {
        CaptureSegmentProgress(
          total: model.totalFrameCount,
          capturedCount: model.capturedFrames.count
        )
      }
    }
    .padding(14)
    .liquidGlass(in: RoundedRectangle(cornerRadius: 18, style: .continuous))
    .padding(.horizontal, 16)
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
    .padding(.bottom, 22)
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

private struct CaptureResultView: View {
  let completion: CaptureCompletion
  let dismiss: () -> Void

  var body: some View {
    NavigationStack {
      Group {
        if let result = completion.stitchingResult {
          PanoramaViewer(imageURL: result.panoramaURL)
            .ignoresSafeArea(edges: .bottom)
            .safeAreaInset(edge: .bottom) {
              Button("Done", action: dismiss)
                .buttonStyle(.borderedProminent)
                .padding()
            }
        } else {
          VStack(spacing: 18) {
            Spacer()
            Image(systemName: "exclamationmark.triangle")
              .font(.system(size: 46))
              .foregroundStyle(.orange)
            Text("Compute unfinished")
              .font(.title2.bold())
            Text("The capture remains in the gallery with all frames and sensor data.")
              .foregroundStyle(.secondary)
              .multilineTextAlignment(.center)
            if let message = completion.stitchingMessage {
              Text(message)
                .font(.callout)
                .multilineTextAlignment(.center)
                .foregroundStyle(.orange)
            }
            Spacer()
            Button("Done", action: dismiss)
              .buttonStyle(.borderedProminent)
          }
          .padding()
        }
      }
      .navigationTitle("Panorama")
      .navigationBarTitleDisplayMode(.inline)
    }
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
