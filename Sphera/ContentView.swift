import SwiftUI

struct ContentView: View {
  @StateObject private var model = CaptureViewModel()
  @State private var handledDebugLaunchArguments = false

  var body: some View {
    Group {
      switch model.phase {
      case .setup:
        CaptureSetupView(model: model)
      case .preparing:
        StatusView(title: "Preparing capture", detail: model.statusMessage)
      case .capturing:
        CaptureScreen(model: model)
      case .stitching:
        StatusView(title: "Stitching panorama", detail: model.statusMessage)
      case .completed(let completion):
        CaptureResultView(completion: completion) {
          model.prepareNewCapture()
        }
      case .failed(let message):
        FailureView(message: message) {
          model.prepareNewCapture()
        }
      }
    }
    .preferredColorScheme(.dark)
    .onAppear {
      #if DEBUG
        guard !handledDebugLaunchArguments else { return }
        handledDebugLaunchArguments = true
        if ProcessInfo.processInfo.arguments.contains("--auto-start-capture") {
          model.startCapture()
        }
      #endif
    }
  }
}

private struct CaptureSetupView: View {
  @ObservedObject var model: CaptureViewModel

  var body: some View {
    NavigationStack {
      Form {
        Section("Capture preset") {
          Stepper(value: $model.configuration.horizontalCount, in: 4...16) {
            countRow("Horizontal ring", model.configuration.horizontalCount)
          }
          Stepper(value: $model.configuration.downwardCount, in: 4...6) {
            countRow("Downward ring", model.configuration.downwardCount)
          }
          Stepper(value: $model.configuration.upwardCount, in: 4...6) {
            countRow("Upward ring", model.configuration.upwardCount)
          }
          countRow("Total", model.configuration.totalImageCount)
        }

        Section {
          Button("Start capture") {
            model.startCapture()
          }
          .frame(maxWidth: .infinity)
          .buttonStyle(.borderedProminent)
        }
      }
      .navigationTitle("Sphera Capture")
      .onChange(of: model.configuration) {
        model.rebuildPlan()
      }
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

  var body: some View {
    ZStack {
      Color.black.ignoresSafeArea()
      CameraPreviewView(session: model.camera.session)
        .ignoresSafeArea()

      VStack(spacing: 0) {
        topPanel
        Spacer(minLength: 12)
        CaptureGuideView(
          instruction: model.navigationInstruction
        )
        .frame(maxWidth: 390)
        Spacer(minLength: 12)
        if let error = model.captureErrorMessage {
          Text(error)
            .font(.callout.weight(.semibold))
            .foregroundStyle(.white)
            .multilineTextAlignment(.center)
            .frame(maxWidth: .infinity)
            .padding(12)
            .background(.red.opacity(0.82), in: RoundedRectangle(cornerRadius: 12))
        }
      }
      .padding(.horizontal, 16)
      .padding(.vertical, 10)
    }
  }

  private var topPanel: some View {
    VStack(spacing: 8) {
      HStack {
        if let target = model.currentTarget {
          VStack(alignment: .leading, spacing: 2) {
            Text("PHOTO \(target.sequenceIndex + 1) OF \(model.totalFrameCount)")
              .font(.headline)
            Text(
              "\(target.ring.displayName.uppercased()) RING · \(target.ringIndex + 1) OF \(target.ringCount)"
            )
            .font(.caption.weight(.semibold))
            .foregroundStyle(.secondary)
          }
        }
        Spacer()
        Button("Stop") {
          model.stopCapture()
        }
        .buttonStyle(.bordered)
      }
      ProgressView(value: model.progressFraction)
    }
    .padding(12)
    .background(.black.opacity(0.62), in: RoundedRectangle(cornerRadius: 12))
  }
}

private struct StatusView: View {
  let title: String
  let detail: String

  var body: some View {
    VStack(spacing: 14) {
      ProgressView()
        .controlSize(.large)
      Text(title)
        .font(.title2.bold())
      Text(detail)
        .foregroundStyle(.secondary)
        .multilineTextAlignment(.center)
    }
    .padding(30)
  }
}

private struct CaptureResultView: View {
  let completion: CaptureCompletion
  let newCapture: () -> Void

  var body: some View {
    NavigationStack {
      Group {
        if let result = completion.stitchingResult {
          PanoramaViewer(imageURL: result.panoramaURL)
            .ignoresSafeArea(edges: .bottom)
            .safeAreaInset(edge: .bottom) {
              Button("New capture", action: newCapture)
                .buttonStyle(.borderedProminent)
                .padding()
            }
        } else {
          VStack(spacing: 18) {
            Spacer()
            Image(systemName: "checkmark.circle")
              .font(.system(size: 46))
              .foregroundStyle(.green)
            Text("Capture package complete")
              .font(.title2.bold())
            Text("\(completion.package.manifest.frames.count) full-resolution frames saved")
              .foregroundStyle(.secondary)
            if let message = completion.stitchingMessage {
              Text(message)
                .font(.callout)
                .multilineTextAlignment(.center)
                .foregroundStyle(.orange)
            }
            Text(completion.package.directoryURL.path)
              .font(.caption.monospaced())
              .foregroundStyle(.secondary)
              .textSelection(.enabled)
              .multilineTextAlignment(.center)
            Spacer()
            Button("New capture", action: newCapture)
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
      Button("Back", action: dismiss)
        .buttonStyle(.borderedProminent)
    }
    .padding(30)
  }
}

#Preview {
  ContentView()
}
