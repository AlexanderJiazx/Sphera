import SwiftUI
import UIKit
import UniformTypeIdentifiers

struct GalleryView: View {
  @ObservedObject var model: CaptureViewModel
  @State private var sharePayload: SharePayload?
  @State private var shareErrorMessage: String?
  @State private var isPreparingShare = false

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
        List {
          ForEach(model.galleryPackages, id: \.manifest.sessionID) { package in
            NavigationLink {
              GalleryDetailView(
                model: model,
                package: package,
                onShare: { await presentShare(for: package) }
              )
            } label: {
              GalleryRowView(package: package)
            }
          }
          .onDelete { indexSet in
            let packages = indexSet.map { model.galleryPackages[$0] }
            Task {
              for package in packages {
                await model.deleteFromGallery(package)
              }
            }
          }
        }
        .listStyle(.insetGrouped)
      }
    }
    .navigationTitle("Gallery")
    .navigationBarTitleDisplayMode(.inline)
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

private struct GalleryRowView: View {
  let package: CapturePackage

  var body: some View {
    HStack(spacing: 14) {
      thumbnail
      VStack(alignment: .leading, spacing: 4) {
        Text(package.manifest.completedAt ?? package.manifest.createdAt, format: .dateTime)
          .font(.headline)
        Text(
          "\(package.manifest.frames.count) frames · \(package.hasPanorama ? "Panorama ready" : "Not computed")"
        )
        .font(.caption)
        .foregroundStyle(.secondary)
      }
    }
    .padding(.vertical, 4)
  }

  @ViewBuilder
  private var thumbnail: some View {
    if let url = package.firstImageURL,
      let image = UIImage(contentsOfFile: url.path)
    {
      Image(uiImage: image)
        .resizable()
        .scaledToFill()
        .frame(width: 56, height: 56)
        .clipShape(RoundedRectangle(cornerRadius: 8))
    } else {
      RoundedRectangle(cornerRadius: 8)
        .fill(.secondary.opacity(0.2))
        .frame(width: 56, height: 56)
        .overlay {
          Image(systemName: "photo")
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
      if let url = package.firstImageURL,
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
        "For highest quality, share the archive to a Mac and run Engine scripts/run_hierarchical_loftr.py (compact outdoor LoFTR, ~44 MB), then import panorama_equirectangular.jpg here."
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
