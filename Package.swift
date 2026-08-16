// swift-tools-version: 6.0

import PackageDescription

let package = Package(
  name: "SpheraCoreVerification",
  defaultLocalization: "en",
  platforms: [.macOS(.v15)],
  products: [
    .library(name: "SpheraCore", targets: ["SpheraCore"])
  ],
  targets: [
    .target(
      name: "SpheraCore",
      path: "Sphera",
      exclude: [
        "EngineBridge",
        "ExperimentalSpheraEngine.swift",
        "Resources",
        "Settings.bundle",
        "CameraCaptureService.swift",
        "CameraPreviewView.swift",
        "CaptureGuideView.swift",
        "CaptureViewModel.swift",
        "ContentView.swift",
        "GalleryView.swift",
        "LoFTRMatcher.swift",
        "LoFTRSmokeTest.swift",
        "MotionTrackingService.swift",
        "OpenCVSpheraEngine.swift",
        "PanoramaViewer.swift",
        "Sphera-Bridging-Header.h",
        "SpheraApp.swift",
        "ARKitTrackingService.swift",
        "ARKitCameraPreviewView.swift",
        "ExperimentalCaptureController.swift",
        "ExperimentalCaptureGuideView.swift",
        "ExperimentalCaptureScreen.swift",
      ],
      sources: [
        "AlignmentHoldTracker.swift",
        "CaptureNavigation.swift",
        "CaptureModels.swift",
        "CapturePackageStore.swift",
        "CaptureShareArchive.swift",
        "ExperimentalCaptureModels.swift",
        "ExperimentalCapturePackageStore.swift",
        "ExperimentalPanoramaConfiguration.swift",
        "ExperimentalPoseMath.swift",
        "ExperimentalScanProgressor.swift",
        "OrientationMath.swift",
        "PanoramaStitching.swift",
      ]
    ),
    .testTarget(
      name: "SpheraCoreTests",
      dependencies: ["SpheraCore"],
      path: "SpheraCoreTests"
    ),
  ]
)
