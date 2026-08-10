import SwiftUI
import UIKit

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

  var body: some View {
    List {
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

        Button(role: .destructive) {
          Task {
            await model.deleteFromGallery(package)
            dismiss()
          }
        } label: {
          Label("Delete capture", systemImage: "trash")
        }
      }
    }
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
