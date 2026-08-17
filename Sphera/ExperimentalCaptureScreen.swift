import SwiftUI

struct ExperimentalCaptureScreen: View {
  @ObservedObject var model: CaptureViewModel
  @ObservedObject var experimental: ExperimentalCaptureController

  init(model: CaptureViewModel) {
    self.model = model
    self.experimental = model.experimental
  }

  private var guidance: ExperimentalGuidanceSnapshot { experimental.guidance }
  private var stage: ExperimentalCaptureStage { experimental.stage }

  var body: some View {
    ZStack {
      Color.black.ignoresSafeArea()

      ARKitCameraPreviewView(
        service: experimental.arkit,
        isSourceReady: experimental.arkit.isFeedActive
      )
      .ignoresSafeArea()
      .allowsHitTesting(false)

      if stage.showsSweepGuides {
        ExperimentalLevelGuide(
          pitchErrorDegrees: guidance.pitchErrorDegrees,
          rollDegrees: guidance.rollDegrees,
          guideScaleDegrees: guidance.pitchGuideScaleDegrees,
          isAligned: guidance.isLevelForCapture,
          isBlocked: guidance.isBlocked,
          holdFraction: stage == .aligning ? guidance.alignmentHoldFraction : 0
        )
        .equatable()
        .allowsHitTesting(false)
      }

      VStack(spacing: 0) {
        topBar
        Spacer(minLength: 0)
        bottomStack
      }
      .padding(.horizontal, 16)
      .padding(.top, 4)
      .padding(.bottom, 8)

      if stage == .starting || stage == .finishing {
        busyOverlay
      }
      if stage == .paused, let reason = experimental.pauseReason {
        pausedOverlay(reason)
      }
    }
    .animation(.easeInOut(duration: 0.2), value: stage)
  }

  // MARK: - Top

  private var topBar: some View {
    HStack(alignment: .top) {
      if stage.showsSweepGuides {
        passPill
      }
      Spacer(minLength: 8)
      if stage.showsSweepGuides {
        Button("Cancel", role: .cancel) {
          model.cancelExperimentalSweep()
        }
        .buttonStyle(.glassStop)
      } else {
        CaptureSessionModeSwitch(
          mode: model.displayedCaptureSessionMode,
          isEnabled: stage == .ready && !model.isSwitchingCaptureSessionMode,
          onSelect: { model.setCaptureSessionMode($0) }
        )
      }
    }
  }

  private var passPill: some View {
    VStack(alignment: .leading, spacing: 2) {
      Text(guidance.passCaption)
        .font(.footnote.weight(.semibold))
        .foregroundStyle(.white)
      Text(guidance.photoCaption)
        .font(.caption)
        .foregroundStyle(.white.opacity(0.7))
    }
    .padding(.horizontal, 14)
    .padding(.vertical, 8)
    .liquidGlass(in: RoundedRectangle(cornerRadius: 14, style: .continuous))
    .accessibilityElement(children: .combine)
  }

  // MARK: - Bottom

  private var bottomStack: some View {
    VStack(spacing: 14) {
      if let alert = guidance.alert {
        ExperimentalAlertBadge(text: alert)
          .transition(.scale(scale: 0.92).combined(with: .opacity))
      }

      instructionBlock

      if let notice = experimental.noticeMessage ?? model.captureErrorMessage {
        Text(notice)
          .font(.footnote.weight(.medium))
          .foregroundStyle(.white)
          .multilineTextAlignment(.center)
          .padding(.horizontal, 12)
          .padding(.vertical, 8)
          .background(.orange.opacity(0.9), in: Capsule())
          .transition(.opacity)
      }

      if stage.showsSweepGuides {
        ExperimentalCoverageTrack(
          targetStates: guidance.targetStates,
          sweepFraction: guidance.sweepFraction,
          isDimmed: experimental.isCapturingPhoto,
          strip: experimental.arkit.livePanoPreview.image
        )
        totalProgress
      }

      if stage == .ready {
        shutterButton
      }
    }
    .animation(.easeInOut(duration: 0.18), value: guidance.alert)
    .animation(.easeInOut(duration: 0.18), value: experimental.noticeMessage)
  }

  private var instructionBlock: some View {
    VStack(spacing: 4) {
      Text(guidance.title)
        .font(.title3.weight(.semibold))
        .foregroundStyle(.white)
      if let subtitle = guidance.subtitle {
        Text(subtitle)
          .font(.subheadline)
          .foregroundStyle(.white.opacity(0.75))
      }
    }
    .multilineTextAlignment(.center)
    .shadow(color: .black.opacity(0.5), radius: 4, y: 1)
    .frame(maxWidth: .infinity)
    .padding(.horizontal, 8)
    .contentTransition(.opacity)
    .animation(.easeInOut(duration: 0.2), value: guidance.title)
    .accessibilityElement(children: .combine)
  }

  private var totalProgress: some View {
    HStack(spacing: 10) {
      ProgressView(value: guidance.totalProgress)
        .tint(.yellow)
      Text("\(guidance.capturedTotal)/\(guidance.targetTotal)")
        .font(.caption.weight(.semibold).monospacedDigit())
        .foregroundStyle(.white.opacity(0.8))
    }
    .accessibilityElement(children: .ignore)
    .accessibilityLabel("Sweep progress")
    .accessibilityValue("\(guidance.capturedTotal) of \(guidance.targetTotal) photos")
  }

  private var shutterButton: some View {
    Button {
      model.beginExperimentalSweep()
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
    .disabled(!experimental.canStartSweep)
    .opacity(experimental.canStartSweep ? 1 : 0.5)
    .accessibilityLabel("Start sweep")
    .accessibilityHint("Takes \(guidance.targetTotal) photos while you turn in a circle")
    .padding(.top, 4)
    .padding(.bottom, 16)
  }

  // MARK: - Overlays

  private var busyOverlay: some View {
    ZStack {
      Color.black.opacity(0.45).ignoresSafeArea()
      VStack(spacing: 14) {
        ProgressView()
          .controlSize(.large)
          .tint(.white)
        Text(guidance.title)
          .font(.headline)
          .foregroundStyle(.white)
        if let subtitle = guidance.subtitle {
          Text(subtitle)
            .font(.subheadline)
            .foregroundStyle(.white.opacity(0.75))
        }
      }
      .multilineTextAlignment(.center)
      .padding(28)
    }
    .transition(.opacity)
    .accessibilityElement(children: .combine)
  }

  private func pausedOverlay(_ reason: ExperimentalPauseReason) -> some View {
    ZStack {
      Rectangle()
        .fill(.ultraThinMaterial)
        .ignoresSafeArea()

      VStack(spacing: 16) {
        Image(systemName: "pause.circle.fill")
          .font(.system(size: 44))
          .foregroundStyle(.yellow)
        Text(reason.title)
          .font(.title3.bold())
        Text(reason.message)
          .font(.subheadline)
          .foregroundStyle(.secondary)
          .multilineTextAlignment(.center)
        Text(
          experimental.resumeRestartsRow
            ? "\(guidance.capturedTotal) of \(guidance.targetTotal) photos taken. The \(guidance.line.rowName.lowercased()) starts over when you resume."
            : "\(guidance.capturedTotal) of \(guidance.targetTotal) photos taken."
        )
        .font(.footnote)
        .foregroundStyle(.secondary)
        .multilineTextAlignment(.center)

        VStack(spacing: 10) {
          Button {
            experimental.resumeSweep()
          } label: {
            Text("Resume sweep")
              .frame(maxWidth: .infinity)
          }
          .buttonStyle(.borderedProminent)
          .controlSize(.large)

          Button(role: .destructive) {
            model.cancelExperimentalSweep()
          } label: {
            Text("Discard sweep")
              .frame(maxWidth: .infinity)
          }
          .buttonStyle(.bordered)
          .controlSize(.large)
        }
        .padding(.top, 4)
      }
      .padding(24)
      .frame(maxWidth: 340)
    }
    .transition(.opacity)
  }
}

struct ExperimentalSavedCaptureView: View {
  let package: ExperimentalCapturePackage
  @ObservedObject var model: CaptureViewModel
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

  private var skippedCount: Int { package.manifest.skippedTargets?.count ?? 0 }

  var body: some View {
    NavigationStack {
      VStack(spacing: 20) {
        Spacer()
        Image(systemName: "checkmark.circle.fill")
          .font(.system(size: 52))
          .foregroundStyle(.green)
        Text("Sweep saved")
          .font(.title2.bold())
        Text(detailText)
          .font(.body)
          .foregroundStyle(.secondary)
          .multilineTextAlignment(.center)
          .padding(.horizontal)

        VStack(spacing: 12) {
          Button {
            Task { await presentShare() }
          } label: {
            Label(
              isPreparingShare ? "Preparing archive…" : "Share sweep archive",
              systemImage: "square.and.arrow.up"
            )
            .frame(maxWidth: .infinity)
          }
          .buttonStyle(.borderedProminent)
          .controlSize(.large)
          .disabled(isPreparingShare)

          Button("New capture") {
            model.returnToSetup()
          }
          .buttonStyle(.bordered)
          .controlSize(.large)
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

  private var detailText: String {
    let count = package.manifest.frames.count
    let base =
      "\(count) photos with camera poses and motion data are in your gallery, ready to share."
    guard skippedCount > 0 else { return base }
    return base + " \(skippedCount) angles were skipped along the way."
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
