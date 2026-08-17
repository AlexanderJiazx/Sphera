import SwiftUI
import UIKit
import UniformTypeIdentifiers

struct GalleryView: View {
  @ObservedObject var model: CaptureViewModel
  @Namespace private var galleryZoom
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

  private let columns = [
    GridItem(.flexible(), spacing: 12),
    GridItem(.flexible(), spacing: 12),
  ]

  var body: some View {
    Group {
      if model.isRefreshingGallery && model.galleryItems.isEmpty {
        ProgressView("Loading gallery")
      } else if model.galleryItems.isEmpty {
        ContentUnavailableView(
          "No captures yet",
          systemImage: "photo.on.rectangle.angled",
          description: Text("Finished captures are saved here with images and motion data.")
        )
      } else {
        ScrollView {
          LazyVGrid(columns: columns, spacing: 12) {
            ForEach(model.galleryItems) { item in
              NavigationLink {
                galleryDestination(for: item)
              } label: {
                switch item {
                case .standard(let package):
                  GalleryGridCard(package: package, zoomNamespace: galleryZoom)
                case .experimental(let package):
                  ExperimentalGalleryGridCard(package: package)
                }
              }
              .buttonStyle(.plain)
              .contextMenu {
                Button(role: .destructive) {
                  Task {
                    switch item {
                    case .standard(let package):
                      await model.deleteFromGallery(package)
                    case .experimental(let package):
                      await model.deleteFromGallery(package)
                    }
                  }
                } label: {
                  Label("Delete", systemImage: "trash")
                }
              }
            }
          }
          .padding(16)
        }
      }
    }
    .navigationTitle("Gallery")
    .navigationBarTitleDisplayMode(.large)
    .toolbar {
      ToolbarItem(placement: .topBarTrailing) {
        Button {
          Task { await model.refreshGallery() }
        } label: {
          Image(systemName: "arrow.clockwise")
        }
        .disabled(model.isRefreshingGallery)
      }
    }
    .task {
      await model.refreshGallery()
    }
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
    .alert(
      "Gallery error",
      isPresented: Binding(
        get: { model.galleryErrorMessage != nil },
        set: { if !$0 { model.clearGalleryError() } }
      )
    ) {
      Button("OK", role: .cancel) {}
    } message: {
      Text(model.galleryErrorMessage ?? "")
    }
    .overlay {
      if isPreparingShare {
        ZStack {
          Color.black.opacity(0.35).ignoresSafeArea()
          ProgressView("Preparing share archive")
            .padding(20)
            .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 14))
        }
      }
    }
  }

  @ViewBuilder
  private func galleryDestination(for item: GalleryCaptureItem) -> some View {
    switch item {
    case .standard(let package):
      if package.hasPanorama {
        GalleryPanoramaView(
          model: model,
          package: package,
          onShare: { await presentShare(for: package) }
        )
        .navigationTransition(.zoom(sourceID: package.manifest.sessionID, in: galleryZoom))
      } else {
        GalleryDetailView(
          model: model,
          package: package,
          onShare: { await presentShare(for: package) }
        )
      }
    case .experimental(let package):
      ExperimentalGalleryDetailView(
        model: model,
        package: package,
        onShare: { await presentShare(for: package) }
      )
    }
  }

  @MainActor
  private func presentShare(for package: CapturePackage) async {
    isPreparingShare = true
    defer { isPreparingShare = false }
    do {
      let url = try await model.makeShareArchive(for: package)
      sharePayload = SharePayload(url: url)
    } catch {
      shareErrorMessage = error.localizedDescription
    }
  }

  @MainActor
  private func presentShare(for package: ExperimentalCapturePackage) async {
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

private struct GalleryGridCard: View {
  let package: CapturePackage
  var zoomNamespace: Namespace.ID

  var body: some View {
    VStack(alignment: .leading, spacing: 0) {
      thumbnail
      VStack(alignment: .leading, spacing: 2) {
        Text(package.manifest.completedAt ?? package.manifest.createdAt, format: .dateTime)
          .font(.subheadline.weight(.semibold))
          .lineLimit(1)
        Text(
          "\(package.manifest.frames.count) frames · \(package.hasPanorama ? "Ready" : "Raw")"
        )
        .font(.caption)
        .foregroundStyle(.secondary)
        .lineLimit(1)
      }
      .padding(.top, 8)
      .padding(.horizontal, 2)
    }
  }

  @ViewBuilder
  private var thumbnail: some View {
    let shape = RoundedRectangle(cornerRadius: 14, style: .continuous)
    preview
      .frame(maxWidth: .infinity)
      .aspectRatio(1, contentMode: .fit)
      .clipShape(shape)
      .overlay(alignment: .topTrailing) {
        if package.hasPanorama {
          Image(systemName: "pano.fill")
            .font(.caption.weight(.bold))
            .foregroundStyle(.white)
            .padding(6)
            .background(.black.opacity(0.55), in: Capsule())
            .padding(8)
        }
      }
      .overlay {
        shape.strokeBorder(.white.opacity(0.12), lineWidth: 1)
      }
      .shadow(color: .black.opacity(0.35), radius: 8, y: 4)
      .galleryZoomSource(
        id: package.manifest.sessionID,
        namespace: zoomNamespace,
        enabled: package.hasPanorama
      )
  }

  @ViewBuilder
  private var preview: some View {
    if let url = package.previewImageURL,
      let image = UIImage(contentsOfFile: url.path)
    {
      Image(uiImage: image)
        .resizable()
        .scaledToFill()
        .frame(minWidth: 0, maxWidth: .infinity, minHeight: 0, maxHeight: .infinity)
    } else {
      ZStack {
        Color.secondary.opacity(0.18)
        Image(systemName: "photo")
          .font(.title)
          .foregroundStyle(.secondary)
      }
    }
  }
}

private struct GalleryPanoramaView: View {
  @ObservedObject var model: CaptureViewModel
  let package: CapturePackage
  let onShare: () async -> Void

  @Environment(\.dismiss) private var dismiss
  @State private var showInfo = false
  @State private var revealPanorama = false

  private var livePackage: CapturePackage {
    model.galleryPackages.first {
      $0.manifest.sessionID == package.manifest.sessionID
    } ?? package
  }

  private var previewImage: UIImage? {
    guard let url = livePackage.previewImageURL else { return nil }
    return UIImage(contentsOfFile: url.path)
  }

  var body: some View {
    ZStack {
      Color.black
      PanoramaViewer(imageURL: livePackage.panoramaURL)
        .frame(maxWidth: .infinity, maxHeight: .infinity)
      if let previewImage {
        Image(uiImage: previewImage)
          .resizable()
          .scaledToFill()
          .frame(maxWidth: .infinity, maxHeight: .infinity)
          .clipped()
          .opacity(revealPanorama ? 0 : 1)
          .allowsHitTesting(false)
          .accessibilityHidden(true)
      }
    }
    .frame(maxWidth: .infinity, maxHeight: .infinity)
    .background(Color.black)
    .ignoresSafeArea()
    .navigationTitle("")
    .navigationBarTitleDisplayMode(.inline)
    .containerBackground(.black, for: .navigation)
    .toolbarBackground(.hidden, for: .navigationBar)
    .toolbarBackgroundVisibility(.hidden, for: .navigationBar)
    .toolbarColorScheme(.dark, for: .navigationBar)
    .toolbar {
      ToolbarItem(placement: .topBarTrailing) {
        Button {
          showInfo = true
        } label: {
          Image(systemName: "info.circle")
        }
        .accessibilityLabel("Capture details")
      }
    }
    .toolbar(.hidden, for: .tabBar)
    .toolbarVisibility(.hidden, for: .tabBar)
    .disableNavigationZoomDismissGestures()
    .task {
      try? await Task.sleep(for: .milliseconds(480))
      withAnimation(.easeOut(duration: 0.35)) {
        revealPanorama = true
      }
    }
    .onChange(of: model.phase) { _, phase in
      if phase == .stitching {
        showInfo = false
      }
    }
    .sheet(isPresented: $showInfo) {
      NavigationStack {
        GalleryDetailView(
          model: model,
          package: livePackage,
          onShare: onShare,
          showsPanoramaLink: false,
          onDeleted: {
            showInfo = false
            dismiss()
          }
        )
        .toolbar {
          ToolbarItem(placement: .confirmationAction) {
            Button("Done") { showInfo = false }
          }
        }
      }
    }
  }
}

struct GalleryDetailView: View {
  @ObservedObject var model: CaptureViewModel
  let package: CapturePackage
  let onShare: () async -> Void
  var showsPanoramaLink: Bool = true
  var onDeleted: (() -> Void)? = nil

  @Environment(\.dismiss) private var dismiss
  @State private var isSharing = false
  @State private var showRecomputeConfirmation = false
  @State private var showEngineImportPicker = false

  var body: some View {
    detailList
      .navigationTitle("Capture")
      .navigationBarTitleDisplayMode(.inline)
      .confirmationDialog(
        "Recompute panorama?",
        isPresented: $showRecomputeConfirmation,
        titleVisibility: .visible
      ) {
        Button("Recompute") {
          Task {
            await model.computeOnDevice(package: livePackage, replaceExisting: true)
          }
        }
        Button("Cancel", role: .cancel) {}
      } message: {
        Text("This replaces the existing on-device panorama with a fresh stitch of the saved frames.")
      }
      .fileImporter(
        isPresented: $showEngineImportPicker,
        allowedContentTypes: [.jpeg, .json],
        allowsMultipleSelection: true,
        onCompletion: handleEngineImport
      )
      .disabled(model.phase == .stitching)
  }

  private var livePackage: CapturePackage {
    model.galleryPackages.first {
      $0.manifest.sessionID == package.manifest.sessionID
    } ?? package
  }

  private var detailList: some View {
    List {
      previewSection
      captureMetadataSection
      framesSection
      actionsSection
    }
  }

  @ViewBuilder
  private var previewSection: some View {
    Section {
      if let url = livePackage.previewImageURL,
        let image = UIImage(contentsOfFile: url.path)
      {
        Image(uiImage: image)
          .resizable()
          .scaledToFit()
          .frame(maxWidth: .infinity)
          .listRowInsets(EdgeInsets())
      }
    }
  }

  private var captureMetadataSection: some View {
    Section("Capture") {
      LabeledContent("Frames", value: "\(package.manifest.frames.count)")
      LabeledContent(
        "Saved",
        value: (package.manifest.completedAt ?? package.manifest.createdAt)
          .formatted(date: .abbreviated, time: .shortened)
      )
      LabeledContent(
        "On-device panorama",
        value: livePackage.hasPanorama ? "Ready" : "Not computed"
      )
      LabeledContent("Motion frame", value: package.manifest.coreMotionReferenceFrame)
    }
  }

  private var framesSection: some View {
    Section("Frames") {
      ForEach(package.manifest.frames) { frame in
        VStack(alignment: .leading, spacing: 4) {
          Text("\(frame.sequenceIndex + 1). \(frame.target.ring.displayName) \(frame.target.ringIndex + 1)")
            .font(.headline)
          Text(
            "Yaw \(frame.target.yawDegrees, format: .number.precision(.fractionLength(1)))° · Pitch \(frame.target.pitchDegrees, format: .number.precision(.fractionLength(1)))°"
          )
          .font(.caption)
          .foregroundStyle(.secondary)
          Text(
            "Alignment error \(frame.alignment.directionErrorDegrees, format: .number.precision(.fractionLength(2)))°"
          )
          .font(.caption2)
          .foregroundStyle(.secondary)
        }
        .padding(.vertical, 2)
      }
    }
  }

  private var actionsSection: some View {
    Section {
      Button {
        Task {
          isSharing = true
          await onShare()
          isSharing = false
        }
      } label: {
        Label(
          isSharing ? "Preparing archive…" : "Share capture archive",
          systemImage: "square.and.arrow.up"
        )
      }
      .disabled(isSharing)

      Button {
        showEngineImportPicker = true
      } label: {
        Label("Import Engine panorama", systemImage: "square.and.arrow.down.on.square")
      }

      panoramaActions

      Button(role: .destructive) {
        Task {
          await model.deleteFromGallery(package)
          if let onDeleted {
            onDeleted()
          } else {
            dismiss()
          }
        }
      } label: {
        Label {
          Text("Delete capture")
        } icon: {
          Image(systemName: "trash")
            .symbolRenderingMode(.monochrome)
            .foregroundStyle(.red)
        }
      }
      .tint(.red)
      .foregroundStyle(.red)
    } footer: {
      Text(
        "On-device stitching uses sensor-first SIFT (no ML models). LoFTR remains an optional offline diagnostic only—share the archive if you need to compare Engine oracle outputs."
      )
    }
  }

  @ViewBuilder
  private var panoramaActions: some View {
    if livePackage.hasPanorama {
      if showsPanoramaLink {
        NavigationLink {
          GalleryPanoramaView(
            model: model,
            package: livePackage,
            onShare: onShare
          )
        } label: {
          Label("View panorama", systemImage: "pano")
        }
      }

      Button {
        showRecomputeConfirmation = true
      } label: {
        Label("Recompute panorama", systemImage: "arrow.triangle.2.circlepath")
      }
    } else {
      Button {
        Task {
          await model.computeOnDevice(package: livePackage)
        }
      } label: {
        Label("Compute on device", systemImage: "cpu")
      }
    }
  }

  private func handleEngineImport(_ result: Result<[URL], Error>) {
    switch result {
    case .success(let urls):
      let panorama = urls.first(where: isJPEGURL)
      let report = urls.first(where: isJSONURL)
      guard let panorama else {
        model.reportGalleryError(
          "Select panorama_equirectangular.jpg (optional report.json)."
        )
        return
      }
      Task {
        await model.importEnginePanorama(
          into: package,
          panoramaURL: panorama,
          reportURL: report
        )
      }
    case .failure(let error):
      model.reportGalleryError(error.localizedDescription)
    }
  }

  private func isJPEGURL(_ url: URL) -> Bool {
    let ext = url.pathExtension.lowercased()
    return ext == "jpg" || ext == "jpeg"
  }

  private func isJSONURL(_ url: URL) -> Bool {
    url.pathExtension.lowercased() == "json"
  }
}

private struct ExperimentalGalleryGridCard: View {
  let package: ExperimentalCapturePackage

  var body: some View {
    VStack(alignment: .leading, spacing: 0) {
      thumbnail
      VStack(alignment: .leading, spacing: 2) {
        Text(package.manifest.completedAt ?? package.manifest.createdAt, format: .dateTime)
          .font(.subheadline.weight(.semibold))
          .lineLimit(1)
        Text("\(package.manifest.frames.count) photos · Sweep")
          .font(.caption)
          .foregroundStyle(.secondary)
          .lineLimit(1)
      }
      .padding(.top, 8)
      .padding(.horizontal, 2)
    }
  }

  @ViewBuilder
  private var thumbnail: some View {
    let shape = RoundedRectangle(cornerRadius: 14, style: .continuous)
    preview
      .frame(maxWidth: .infinity)
      .aspectRatio(1, contentMode: .fit)
      .clipShape(shape)
      .overlay(alignment: .topTrailing) {
        Label("Sweep", systemImage: CaptureSessionMode.experimentalARKit.symbolName)
          .labelStyle(.titleAndIcon)
          .font(.caption2.weight(.bold))
          .foregroundStyle(.black)
          .padding(.horizontal, 7)
          .padding(.vertical, 4)
          .background(.yellow, in: Capsule())
          .padding(8)
      }
      .overlay {
        shape.strokeBorder(.white.opacity(0.12), lineWidth: 1)
      }
      .shadow(color: .black.opacity(0.35), radius: 8, y: 4)
  }

  @ViewBuilder
  private var preview: some View {
    if let url = package.previewImageURL,
      let image = UIImage(contentsOfFile: url.path)
    {
      Image(uiImage: image)
        .resizable()
        .scaledToFill()
        .frame(minWidth: 0, maxWidth: .infinity, minHeight: 0, maxHeight: .infinity)
    } else {
      ZStack {
        Color.secondary.opacity(0.18)
        Image(systemName: CaptureSessionMode.experimentalARKit.symbolName)
          .font(.title)
          .foregroundStyle(.secondary)
      }
    }
  }
}

struct ExperimentalGalleryDetailView: View {
  @ObservedObject var model: CaptureViewModel
  let package: ExperimentalCapturePackage
  let onShare: () async -> Void

  @Environment(\.dismiss) private var dismiss
  @State private var isSharing = false

  private var livePackage: ExperimentalCapturePackage {
    model.experimentalGalleryPackages.first {
      $0.manifest.sessionID == package.manifest.sessionID
    } ?? package
  }

  var body: some View {
    List {
      Section {
        if let url = livePackage.previewImageURL,
          let image = UIImage(contentsOfFile: url.path)
        {
          Image(uiImage: image)
            .resizable()
            .scaledToFit()
            .frame(maxWidth: .infinity)
            .listRowInsets(EdgeInsets())
        }
      }

      Section("Sweep capture") {
        LabeledContent("Photos", value: "\(package.manifest.frames.count)")
        LabeledContent(
          "Saved",
          value: (package.manifest.completedAt ?? package.manifest.createdAt)
            .formatted(date: .abbreviated, time: .shortened)
        )
        if let skipped = package.manifest.skippedTargets, !skipped.isEmpty {
          LabeledContent("Skipped angles", value: "\(skipped.count)")
        }
        ForEach(package.manifest.lineSummaries, id: \.scanLine) { summary in
          LabeledContent(
            summary.scanLine.rowName,
            value: "\(summary.capturedCount)/\(summary.imageCount)"
          )
        }
      }

      Section("Frames") {
        ForEach(package.manifest.frames) { frame in
          VStack(alignment: .leading, spacing: 4) {
            Text(
              "\(frame.sequenceIndex + 1). \(frame.scanLine.rowName) \(frame.indexInLine + 1)"
            )
            .font(.headline)
            Text(frame.imageFilename)
              .font(.caption2)
              .foregroundStyle(.secondary)
            Text(
              "Yaw offset \(frame.actualYawOffsetDegrees, format: .number.precision(.fractionLength(1)))° · Pitch \(frame.actualPitchDegrees, format: .number.precision(.fractionLength(1)))°"
            )
            .font(.caption)
            .foregroundStyle(.secondary)
            Text("Tracking \(frame.arkit.trackingState.displayName)")
              .font(.caption2)
              .foregroundStyle(.secondary)
          }
          .padding(.vertical, 2)
        }
      }

      Section {
        Button {
          Task {
            isSharing = true
            await onShare()
            isSharing = false
          }
        } label: {
          Label(
            isSharing ? "Preparing archive…" : "Share sweep archive",
            systemImage: "square.and.arrow.up"
          )
        }
        .disabled(isSharing)

        Button(role: .destructive) {
          Task {
            await model.deleteFromGallery(package)
            dismiss()
          }
        } label: {
          Label("Delete capture", systemImage: "trash")
        }
        .tint(.red)
        .foregroundStyle(.red)
      } footer: {
        Text(
          "Sweep captures are photo sets with camera poses, meant for a computer-side stitcher. Sphera's on-device stitcher does not process them yet."
        )
      }
    }
    .navigationTitle("Sweep Capture")
    .navigationBarTitleDisplayMode(.inline)
  }
}

struct SharePayload: Identifiable {
  let id = UUID()
  let url: URL
}

struct ActivityView: UIViewControllerRepresentable {
  let activityItems: [Any]
  var applicationActivities: [UIActivity]? = nil

  func makeUIViewController(context: Context) -> UIActivityViewController {
    UIActivityViewController(
      activityItems: activityItems,
      applicationActivities: applicationActivities
    )
  }

  func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}

private extension View {
  @ViewBuilder
  func galleryZoomSource(id: UUID, namespace: Namespace.ID, enabled: Bool) -> some View {
    if enabled {
      matchedTransitionSource(id: id, in: namespace)
    } else {
      self
    }
  }

  func disableNavigationZoomDismissGestures() -> some View {
    modifier(NavigationZoomDismissGestureDisablerModifier())
  }
}

private struct NavigationZoomDismissGestureDisablerModifier: ViewModifier {
  func body(content: Content) -> some View {
    content
      .background(NavigationZoomDismissGestureDisabler())
  }
}

private struct NavigationZoomDismissGestureDisabler: UIViewControllerRepresentable {
  func makeUIViewController(context: Context) -> UIViewController {
    let vc = UIViewController()
    vc.view.backgroundColor = .clear
    vc.view.isUserInteractionEnabled = false
    return vc
  }

  func updateUIViewController(_ viewController: UIViewController, context: Context) {
    Task { @MainActor in
      Self.applyGestureStates(viewController: viewController, isEnabled: false)
    }
  }

  static func dismantleUIViewController(_ viewController: UIViewController, coordinator: ()) {
    applyGestureStates(viewController: viewController, isEnabled: true)
  }

  @MainActor
  private static func applyGestureStates(viewController: UIViewController, isEnabled: Bool) {
    var current: UIViewController? = viewController
    while let vc = current {
      if let gestures = vc.view.gestureRecognizers {
        for gesture in gestures {
          let name = String(describing: type(of: gesture))
          if name.contains("Parallax") || name.contains("SwipeDismiss") || name.contains("SwipeDown") || name.contains("Transform") || name.contains("ZoomTransition") {
            gesture.isEnabled = isEnabled
          }
        }
      }
      if let nav = vc as? UINavigationController ?? vc.navigationController {
        if let navGestures = nav.view.gestureRecognizers {
          for gesture in navGestures {
            let name = String(describing: type(of: gesture))
            if name.contains("Parallax") || name.contains("SwipeDismiss") || name.contains("SwipeDown") || name.contains("Transform") || name.contains("ZoomTransition") {
              gesture.isEnabled = isEnabled
            }
          }
        }
      }
      current = vc.parent
    }
  }
}
