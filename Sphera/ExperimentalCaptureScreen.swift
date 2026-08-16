import SwiftUI

struct ExperimentalCaptureScreen: View {
  @ObservedObject var model: CaptureViewModel

  private var experimental: ExperimentalCaptureController { model.experimental }
  private var guidance: ExperimentalGuidanceSnapshot { experimental.guidance }

  var body: some View {
    ZStack {
      Color.black.ignoresSafeArea()

      ARKitCameraPreviewView(
        service: experimental.arkit,
        isSourceReady: experimental.arkit.isFeedActive
      )
        .ignoresSafeArea()
        .allowsHitTesting(false)

      if experimental.isARKitReady {
        ExperimentalCaptureGuideView(
          guidance: guidance,
          isCapturingPhoto: experimental.isCapturingPhoto
        )
        .ignoresSafeArea()
      }

      VStack(spacing: 0) {
        header
        Spacer()
        if let error = experimental.errorMessage ?? model.captureErrorMessage {
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
        if !experimental.isSweeping {
          shutterButton
        }
      }
      .padding(.vertical, 10)
    }
  }

  private var header: some View {
    VStack(spacing: 8) {
      HStack {
        VStack(alignment: .leading, spacing: 2) {
          Text(guidance.instruction.uppercased())
            .font(.headline)
            .foregroundStyle(guidance.guideAccentIsBlocking ? Color.red : .primary)
            .lineLimit(1)
          Text(guidance.chromeCaption.uppercased())
            .font(.caption.weight(.semibold))
            .foregroundStyle(.secondary)
        }
        Spacer()
        CaptureSessionModeSwitch(
          mode: model.displayedCaptureSessionMode,
          isEnabled: !experimental.isCapturingPhoto && !experimental.isSweeping
            && !model.isSwitchingCaptureSessionMode,
          onSelect: { model.setCaptureSessionMode($0) }
        )
        if experimental.isSweeping {
          Button("Cancel") {
            model.cancelExperimentalSweep()
          }
          .buttonStyle(.glassStop)
        }
      }
      if experimental.isSweeping {
        CaptureSegmentProgress(
          total: max(guidance.targetInLine, 1),
          capturedCount: guidance.capturedInLine
        )
      }
    }
    .padding(14)
    .liquidGlass(in: RoundedRectangle(cornerRadius: 18, style: .continuous))
    .padding(.horizontal, 16)
    .accessibilityElement(children: .combine)
    .accessibilityLabel(guidance.instruction)
    .accessibilityValue(guidance.chromeCaption)
  }

  private var shutterButton: some View {
    Button {
      model.beginExperimentalSweep()
    } label: {
      ZStack {
        Color.clear
          .frame(width: 84, height: 84)
          .liquidGlassInteractive(in: Circle())
        Circle()
          .strokeBorder(Color(red: 1, green: 0.72, blue: 0.12), lineWidth: 4)
          .frame(width: 70, height: 70)
        Circle()
          .fill(Color(red: 1, green: 0.72, blue: 0.12))
          .frame(width: 54, height: 54)
      }
      .contentShape(Circle())
    }
    .buttonStyle(.elasticGlassCapture)
    .disabled(
      experimental.isCapturingPhoto
        || !experimental.isARKitReady
        || !experimental.hasActivePackage
    )
    .opacity(experimental.isARKitReady && experimental.hasActivePackage ? 1 : 0.55)
    .accessibilityLabel("Start experimental ARKit panorama")
    .padding(.bottom, 24)
  }
}

struct ExperimentalSavedCaptureView: View {
  let package: ExperimentalCapturePackage
  @ObservedObject var model: CaptureViewModel
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

  var body: some View {
    NavigationStack {
      VStack(spacing: 20) {
        Spacer()
        Image(systemName: "move.3d")
          .font(.system(size: 52))
          .foregroundStyle(Color(red: 1, green: 0.72, blue: 0.12))
        Text("Experimental capture saved")
          .font(.title2.bold())
        Text(
          "\(package.manifest.frames.count) ARKit-guided frames with camera poses and gyroscope data are ready to share. The current stitching engine will not process this dataset."
        )
        .font(.body)
        .foregroundStyle(.secondary)
        .multilineTextAlignment(.center)
        .padding(.horizontal)

        VStack(spacing: 12) {
          Button {
            Task { await presentShare() }
          } label: {
            Label(
              isPreparingShare ? "Preparing archive…" : "Share ARKit capture archive",
              systemImage: "square.and.arrow.up"
            )
            .frame(maxWidth: .infinity)
          }
          .buttonStyle(.borderedProminent)
          .tint(Color(red: 1, green: 0.72, blue: 0.12))
          .foregroundStyle(.black)
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
