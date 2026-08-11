import SwiftUI
import UIKit
import UniformTypeIdentifiers

struct GalleryView: View {
  @ObservedObject var model: CaptureViewModel
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

  private let columns = [
    GridItem(.flexible(), spacing: 12),
    GridItem(.flexible(), spacing: 12),
  ]

  var body: some View {
    Group {
      if model.isRefreshingGallery && model.galleryPackages.isEmpty {
        ProgressView("Loading gallery")
      } else if model.galleryPackages.isEmpty {
        ContentUnavailableView(
          "No captures yet",
          systemImage: "photo.on.rectangle.angled",
          description: Text("Finished captures are saved here with images and motion data.")
        )
      } else {
        ScrollView {
          LazyVGrid(columns: columns, spacing: 12) {
            ForEach(model.galleryPackages, id: \.manifest.sessionID) { package in
              NavigationLink {
                GalleryDetailView(
                  model: model,
                  package: package,
                  onShare: { await presentShare(for: package) }
                )
              } label: {
                GalleryGridCard(package: package)
              }
              .buttonStyle(.plain)
              .contextMenu {
                Button(role: .destructive) {
                  Task { await model.deleteFromGallery(package) }
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
}

private struct GalleryGridCard: View {
  let package: CapturePackage

  var body: some View {
    VStack(alignment: .leading, spacing: 0) {
      preview
        .frame(maxWidth: .infinity)
        .aspectRatio(1, contentMode: .fit)
        .clipShape(RoundedRectangle(cornerRadius: 14, style: .continuous))
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
          RoundedRectangle(cornerRadius: 14, style: .continuous)
            .strokeBorder(.white.opacity(0.12), lineWidth: 1)
        }
        .shadow(color: .black.opacity(0.35), radius: 8, y: 4)

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

struct GalleryDetailView: View {
  @ObservedObject var model: CaptureViewModel
  let package: CapturePackage
  let onShare: () async -> Void

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
            await model.computeOnDevice(package: package, replaceExisting: true)
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
      if let url = package.previewImageURL,
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
        value: package.hasPanorama ? "Ready" : "Not computed"
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
            "Pose + gyro recorded · alignment \(frame.alignment.directionErrorDegrees, format: .number.precision(.fractionLength(2)))°"
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
          dismiss()
        }
      } label: {
        Label("Delete capture", systemImage: "trash")
      }
    } footer: {
      Text(
        "On-device stitching uses sensor-first SIFT (no ML models). LoFTR remains an optional offline diagnostic only—share the archive if you need to compare Engine oracle outputs."
      )
    }
  }

  @ViewBuilder
  private var panoramaActions: some View {
    if package.hasPanorama {
      NavigationLink {
        PanoramaViewer(imageURL: package.panoramaURL)
          .navigationTitle("Panorama")
          .navigationBarTitleDisplayMode(.inline)
      } label: {
        Label("View panorama", systemImage: "pano")
      }

      Button {
        showRecomputeConfirmation = true
      } label: {
        Label("Recompute panorama", systemImage: "arrow.triangle.2.circlepath")
      }
    } else {
      Button {
        Task {
          await model.computeOnDevice(package: package)
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
