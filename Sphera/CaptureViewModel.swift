import Combine
import Foundation
import UIKit

struct CaptureCompletion: Equatable, Sendable {
  let package: CapturePackage
  let stitchingResult: StitchingResult?
  let stitchingMessage: String?
}

enum CaptureWorkflowPhase: Equatable {
  case setup
  case preparing
  case awaitingPrimary
  case capturingPoints
  case saved(CapturePackage)
  case stitching
  case completed(CaptureCompletion)
  case failed(String)
}

enum CameraMode: String, CaseIterable, Codable, Sendable {
  case auto = "Auto"
  case manual = "Manual"
}

enum ManualParameter: String, CaseIterable, Identifiable, Sendable {
  case shutter = "Shutter"
  case iso = "ISO"
  case focus = "Focus"
  case whiteBalance = "WB"

  var id: String { rawValue }
}

@MainActor
final class CaptureViewModel: ObservableObject {
  private static let configurationKey = "sphera.captureConfiguration"
  private static let autoFireCaptureKey = "sphera.autoFireCapture"
  private static let alignmentToleranceKey = "sphera.alignmentToleranceDegrees"
  private static let stableHoldDurationKey = "sphera.stableHoldDurationSeconds"
  private static let horizontalCountKey = "sphera.horizontalCount"
  private static let downwardCountKey = "sphera.downwardCount"
  private static let upwardCountKey = "sphera.upwardCount"
  private static let cameraModeKey = "sphera.cameraMode"
  private static let isShutterAutoKey = "sphera.isShutterAuto"
  private static let isISOAutoKey = "sphera.isISOAuto"
  private static let isFocusAutoKey = "sphera.isFocusAuto"
  private static let isWhiteBalanceAutoKey = "sphera.isWhiteBalanceAuto"
  private static let manualISOKey = "sphera.manualISO"
  private static let manualExposureDurationKey = "sphera.manualExposureDuration"
  private static let manualFocusLensPositionKey = "sphera.manualFocusLensPosition"
  private static let manualTemperatureKelvinKey = "sphera.manualTemperatureKelvin"
  private static let experimentalMetalStitchKey = "useExperimentalMetalStitch"
  public static let viewerScrollModeKey = "sphera.viewerScrollMode"

  private var isUpdatingFromDefaults = false

  static func registerDefaults() {
    UserDefaults.standard.register(defaults: [
      autoFireCaptureKey: true,
      alignmentToleranceKey: "6",
      stableHoldDurationKey: "0.3",
      horizontalCountKey: "\(CaptureConfiguration.debugPreset.horizontalCount)",
      downwardCountKey: "\(CaptureConfiguration.debugPreset.downwardCount)",
      upwardCountKey: "\(CaptureConfiguration.debugPreset.upwardCount)",
      experimentalMetalStitchKey: false,
      viewerScrollModeKey: "screenRelative",
    ])
  }

  @Published var configuration: CaptureConfiguration {
    didSet {
      guard !isUpdatingFromDefaults else { return }
      Self.saveConfiguration(configuration)
    }
  }
  @Published var autoFireCapture: Bool {
    didSet {
      guard !isUpdatingFromDefaults else { return }
      UserDefaults.standard.set(autoFireCapture, forKey: Self.autoFireCaptureKey)
    }
  }
  @Published var cameraMode: CameraMode {
    didSet {
      UserDefaults.standard.set(cameraMode.rawValue, forKey: Self.cameraModeKey)
      Task { await applyActiveCameraMode() }
    }
  }
  @Published var isShutterAuto: Bool {
    didSet {
      UserDefaults.standard.set(isShutterAuto, forKey: Self.isShutterAutoKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var isISOAuto: Bool {
    didSet {
      UserDefaults.standard.set(isISOAuto, forKey: Self.isISOAutoKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var isFocusAuto: Bool {
    didSet {
      UserDefaults.standard.set(isFocusAuto, forKey: Self.isFocusAutoKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var isWhiteBalanceAuto: Bool {
    didSet {
      UserDefaults.standard.set(isWhiteBalanceAuto, forKey: Self.isWhiteBalanceAutoKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var selectedManualParameter: ManualParameter? = nil
  @Published var manualISO: Float {
    didSet {
      UserDefaults.standard.set(manualISO, forKey: Self.manualISOKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var manualExposureDurationSeconds: Double {
    didSet {
      UserDefaults.standard.set(manualExposureDurationSeconds, forKey: Self.manualExposureDurationKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var manualFocusLensPosition: Float {
    didSet {
      UserDefaults.standard.set(manualFocusLensPosition, forKey: Self.manualFocusLensPositionKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var manualTemperatureKelvin: Float {
    didSet {
      UserDefaults.standard.set(manualTemperatureKelvin, forKey: Self.manualTemperatureKelvinKey)
      if cameraMode == .manual { Task { await applyManualControls() } }
    }
  }
  @Published var isManualControlPanelExpanded = false
  @Published private(set) var manualCapabilities: CameraManualControlCapabilities = .default
  @Published private(set) var isCameraSourceReady = false

  @Published private(set) var phase: CaptureWorkflowPhase = .setup
  @Published private(set) var plan = CapturePlan(configuration: .debugPreset)
  @Published private(set) var capturedFrames: [CapturedFrameRecord] = []
  private(set) var currentMotionSample: MotionSample?
  @Published private(set) var navigationReading = CaptureNavigationReading.unavailable
  @Published private(set) var guidePoints: [CapturePointProjection] = []
  @Published private(set) var activeTarget: CaptureTarget?
  @Published private(set) var stableHoldProgress = 0.0
  @Published private(set) var isCapturingPhoto = false
  @Published private(set) var statusMessage = "Ready"
  @Published private(set) var stitchProgress: Double?
  @Published private(set) var captureErrorMessage: String?
  @Published private(set) var galleryPackages: [CapturePackage] = []
  @Published private(set) var galleryErrorMessage: String?
  @Published private(set) var isRefreshingGallery = false

  let motion: MotionTrackingService
  let camera: CameraCaptureService

  private let packageStore: CapturePackageStore
  /// When set (tests), compute always uses this stitcher and ignores the toggle.
  private let stitcherOverride: (any PanoramaStitching)?
  /// Settings toggle, off by default. Does not change the stable OpenCV path
  /// unless the user turns this on.
  @Published var useExperimentalMetalStitch: Bool {
    didSet {
      guard !isUpdatingFromDefaults else { return }
      UserDefaults.standard.set(
        useExperimentalMetalStitch,
        forKey: Self.experimentalMetalStitchKey
      )
    }
  }
  private var subscriptions = Set<AnyCancellable>()
  private var captureReference: CaptureReferenceFrame?
  private var alignmentHoldTracker = AlignmentHoldTracker()
  private var autoCaptureBlockedUntil = 0.0
  private var lastNavigationTraceTimestamp = -Double.infinity
  private let navigationTraceEnabled =
    ProcessInfo.processInfo.arguments.contains("--navigation-trace")
  private var isCaptureTabActive = false

  init(
    packageStore: CapturePackageStore = CapturePackageStore(),
    stitcher: (any PanoramaStitching)? = nil
  ) {
    Self.registerDefaults()
    let motion = MotionTrackingService()
    self.motion = motion
    let camera = CameraCaptureService(motionStore: motion.sampleStore)
    self.camera = camera
    self.packageStore = packageStore
    self.stitcherOverride = stitcher
    useExperimentalMetalStitch = UserDefaults.standard.bool(
      forKey: Self.experimentalMetalStitchKey
    )

    let loadedConfig = Self.loadSavedConfiguration()
    self.configuration = loadedConfig
    self.plan = CapturePlan(configuration: loadedConfig)
    if UserDefaults.standard.object(forKey: Self.autoFireCaptureKey) != nil {
      self.autoFireCapture = UserDefaults.standard.bool(forKey: Self.autoFireCaptureKey)
    } else {
      self.autoFireCapture = true
    }
    if let savedMode = UserDefaults.standard.string(forKey: Self.cameraModeKey),
       let mode = CameraMode(rawValue: savedMode) {
      self.cameraMode = mode
    } else {
      self.cameraMode = .auto
    }
    if UserDefaults.standard.object(forKey: Self.isShutterAutoKey) != nil {
      self.isShutterAuto = UserDefaults.standard.bool(forKey: Self.isShutterAutoKey)
    } else {
      self.isShutterAuto = true
    }
    if UserDefaults.standard.object(forKey: Self.isISOAutoKey) != nil {
      self.isISOAuto = UserDefaults.standard.bool(forKey: Self.isISOAutoKey)
    } else {
      self.isISOAuto = true
    }
    if UserDefaults.standard.object(forKey: Self.isFocusAutoKey) != nil {
      self.isFocusAuto = UserDefaults.standard.bool(forKey: Self.isFocusAutoKey)
    } else {
      self.isFocusAuto = true
    }
    if UserDefaults.standard.object(forKey: Self.isWhiteBalanceAutoKey) != nil {
      self.isWhiteBalanceAuto = UserDefaults.standard.bool(forKey: Self.isWhiteBalanceAutoKey)
    } else {
      self.isWhiteBalanceAuto = true
    }
    let savedISO = UserDefaults.standard.float(forKey: Self.manualISOKey)
    self.manualISO = savedISO > 0 ? savedISO : 100
    let savedDuration = UserDefaults.standard.double(forKey: Self.manualExposureDurationKey)
    self.manualExposureDurationSeconds = savedDuration > 0 ? savedDuration : 1.0 / 125.0
    let savedFocus = UserDefaults.standard.float(forKey: Self.manualFocusLensPositionKey)
    self.manualFocusLensPosition = savedFocus > 0 ? savedFocus : 0.5
    let savedTemp = UserDefaults.standard.float(forKey: Self.manualTemperatureKelvinKey)
    self.manualTemperatureKelvin = savedTemp > 0 ? savedTemp : 5500

    motion.$currentSample
      .sink { [weak self] sample in
        self?.handleMotionSample(sample)
      }
      .store(in: &subscriptions)

    NotificationCenter.default.publisher(for: UserDefaults.didChangeNotification)
      .receive(on: DispatchQueue.main)
      .sink { [weak self] _ in
        self?.reloadSettingsFromDefaults()
      }
      .store(in: &subscriptions)

    Publishers.CombineLatest(camera.$isFeedActive, camera.$state)
      .receive(on: DispatchQueue.main)
      .sink { [weak self] isFeedActive, state in
        guard let self else { return }
        if state == .running && isFeedActive && self.isCaptureTabActive {
          if !self.isCameraSourceReady {
            self.isCameraSourceReady = true
          }
          self.refreshManualCapabilities()
          if self.cameraMode == .manual {
            Task { await self.applyManualControls() }
          }
        }
      }
      .store(in: &subscriptions)
  }

  func reloadSettingsFromDefaults() {
    guard !isUpdatingFromDefaults else { return }
    isUpdatingFromDefaults = true
    defer { isUpdatingFromDefaults = false }

    let newAutoFire = UserDefaults.standard.object(forKey: Self.autoFireCaptureKey) != nil
      ? UserDefaults.standard.bool(forKey: Self.autoFireCaptureKey)
      : true
    if autoFireCapture != newAutoFire {
      autoFireCapture = newAutoFire
    }

    let newMetal = UserDefaults.standard.bool(forKey: Self.experimentalMetalStitchKey)
    if useExperimentalMetalStitch != newMetal {
      useExperimentalMetalStitch = newMetal
    }

    let newConfig = Self.loadSavedConfiguration()
    if configuration != newConfig {
      configuration = newConfig
      rebuildPlan()
    }
  }

  private static func parseDouble(
    forKey key: String,
    fallback: Double,
    min: Double? = nil,
    max: Double? = nil
  ) -> Double {
    guard let raw = UserDefaults.standard.object(forKey: key) else { return fallback }
    let value: Double
    if let num = raw as? NSNumber {
      value = num.doubleValue
    } else if let str = raw as? String,
              let parsed = Double(str.trimmingCharacters(in: .whitespacesAndNewlines)) {
      value = parsed
    } else {
      value = fallback
    }
    var clamped = value
    if let min { clamped = Swift.max(min, clamped) }
    if let max { clamped = Swift.min(max, clamped) }
    return clamped
  }

  private static func parseInt(
    forKey key: String,
    fallback: Int,
    min: Int? = nil,
    max: Int? = nil
  ) -> Int {
    guard let raw = UserDefaults.standard.object(forKey: key) else { return fallback }
    let value: Int
    if let num = raw as? NSNumber {
      value = Int(round(num.doubleValue))
    } else if let str = raw as? String,
              let parsed = Double(str.trimmingCharacters(in: .whitespacesAndNewlines)) {
      value = Int(round(parsed))
    } else {
      value = fallback
    }
    var clamped = value
    if let min { clamped = Swift.max(min, clamped) }
    if let max { clamped = Swift.min(max, clamped) }
    return clamped
  }

  private static func loadSavedConfiguration() -> CaptureConfiguration {
    var config = CaptureConfiguration.debugPreset

    if let data = UserDefaults.standard.data(forKey: configurationKey),
       let decoded = try? JSONDecoder().decode(CaptureConfiguration.self, from: data) {
      config = decoded
    }

    if UserDefaults.standard.object(forKey: alignmentToleranceKey) != nil {
      config.alignmentToleranceDegrees = parseDouble(
        forKey: alignmentToleranceKey,
        fallback: config.alignmentToleranceDegrees,
        min: 3,
        max: 15
      )
    }
    if UserDefaults.standard.object(forKey: stableHoldDurationKey) != nil {
      config.stableHoldDurationSeconds = parseDouble(
        forKey: stableHoldDurationKey,
        fallback: config.stableHoldDurationSeconds,
        min: 0.1,
        max: 1.5
      )
    }
    if UserDefaults.standard.object(forKey: horizontalCountKey) != nil {
      config.horizontalCount = parseInt(
        forKey: horizontalCountKey,
        fallback: config.horizontalCount,
        min: 4,
        max: 16
      )
    }
    if UserDefaults.standard.object(forKey: downwardCountKey) != nil {
      config.downwardCount = parseInt(
        forKey: downwardCountKey,
        fallback: config.downwardCount,
        min: 4,
        max: 8
      )
    }
    if UserDefaults.standard.object(forKey: upwardCountKey) != nil {
      config.upwardCount = parseInt(
        forKey: upwardCountKey,
        fallback: config.upwardCount,
        min: 4,
        max: 8
      )
    }

    return config
  }

  private static func saveConfiguration(_ config: CaptureConfiguration) {
    let toleranceString = config.alignmentToleranceDegrees.truncatingRemainder(dividingBy: 1) == 0
      ? "\(Int(config.alignmentToleranceDegrees))"
      : String(format: "%.1f", config.alignmentToleranceDegrees)
    UserDefaults.standard.set(toleranceString, forKey: alignmentToleranceKey)
    UserDefaults.standard.set(String(format: "%.1f", config.stableHoldDurationSeconds), forKey: stableHoldDurationKey)
    UserDefaults.standard.set("\(config.horizontalCount)", forKey: horizontalCountKey)
    UserDefaults.standard.set("\(config.downwardCount)", forKey: downwardCountKey)
    UserDefaults.standard.set("\(config.upwardCount)", forKey: upwardCountKey)
    if let encoded = try? JSONEncoder().encode(config) {
      UserDefaults.standard.set(encoded, forKey: configurationKey)
    }
  }

  /// Default remains the stable OpenCV engine. The experimental 3-second
  /// Swift/Metal engine is selected only when the Settings toggle is on and
  /// no test override is set.
  private func activeStitcher() -> any PanoramaStitching {
    if let stitcherOverride {
      return stitcherOverride
    }
    if useExperimentalMetalStitch {
      return SpheraEngineAdapter(nativeEngine: ExperimentalSpheraEngine())
    }
    return SpheraEngineAdapter(nativeEngine: OpenCVSpheraEngine())
  }

  var totalFrameCount: Int { plan.targets.count }

  var remainingTargets: [CaptureTarget] {
    let capturedIDs = Set(capturedFrames.map(\.target.id))
    return plan.targets.filter { !capturedIDs.contains($0.id) }
  }

  var progressFraction: Double {
    guard totalFrameCount > 0 else { return 0 }
    return Double(capturedFrames.count) / Double(totalFrameCount)
  }

  func rebuildPlan() {
    configuration.horizontalCount = min(16, max(4, configuration.horizontalCount))
    configuration.downwardCount = min(8, max(4, configuration.downwardCount))
    configuration.upwardCount = min(8, max(4, configuration.upwardCount))
    switch phase {
    case .setup, .saved, .completed, .failed:
      plan = CapturePlan(configuration: configuration)
    default:
      break
    }
  }

  func startCapture() {
    guard phase == .setup || phase.isTerminal else { return }
    rebuildPlan()
    isCameraSourceReady = false
    phase = .preparing
    statusMessage = "Starting camera and motion"
    captureErrorMessage = nil
    capturedFrames = []
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    autoCaptureBlockedUntil = 0
    captureReference = nil

    Task {
      do {
        try motion.start()
        try await startCameraIfCaptureTabActive(restoreAuto: true)
        _ = try await motion.waitForFirstSample()
        plan = CapturePlan(configuration: configuration)
        _ = try await packageStore.begin(
          plan: plan,
          coreMotionReferenceFrame: motion.referenceFrameName
        )
        phase = .awaitingPrimary
        statusMessage = "Frame the first shot, then capture"
        stopCameraIfCaptureTabInactive()
      } catch {
        camera.stop()
        motion.stop()
        await packageStore.abandon()
        phase = .failed(error.localizedDescription)
        statusMessage = "Capture unavailable"
      }
    }
  }

  func capturePrimary() {
    guard phase == .awaitingPrimary, !isCapturingPhoto else { return }
    isCapturingPhoto = true
    captureErrorMessage = nil
    statusMessage = "Capturing primary"
    Task {
      await performPrimaryCapture()
    }
  }

  /// Abandons the in-progress session and returns to setup so the Capture tab
  /// can start a fresh session.
  func resetCapture() {
    guard phase == .capturingPoints else { return }
    isCameraSourceReady = false
    motion.stop()
    isCapturingPhoto = false
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    statusMessage = "Resetting capture"
    Task {
      await camera.unlockExposureFocusWhiteBalance()
      camera.stop()
      await packageStore.abandon()
      phase = .setup
      statusMessage = "Ready"
      plan = CapturePlan(configuration: configuration)
    }
  }

  func stopCapture() {
    isCameraSourceReady = false
    camera.stop()
    motion.stop()
    isCapturingPhoto = false
    Task {
      await packageStore.abandon()
    }
    phase = .setup
    statusMessage = "Ready"
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    plan = CapturePlan(configuration: configuration)
  }

  func prepareNewCapture() {
    stopCapture()
  }

  func returnToSetup() {
    isCameraSourceReady = false
    camera.stop()
    motion.stop()
    isCapturingPhoto = false
    phase = .setup
    statusMessage = "Ready"
    stitchProgress = nil
    captureErrorMessage = nil
    navigationReading = .unavailable
    guidePoints = []
    activeTarget = nil
    stableHoldProgress = 0
    alignmentHoldTracker.reset()
    captureReference = nil
    capturedFrames = []
    plan = CapturePlan(configuration: configuration)
  }

  func setCameraMode(_ mode: CameraMode) {
    cameraMode = mode
    if mode != .manual {
      selectedManualParameter = nil
    }
  }

  func selectManualParameter(_ param: ManualParameter) {
    if selectedManualParameter == param {
      selectedManualParameter = nil
    } else {
      selectedManualParameter = param
    }
  }

  func applyActiveCameraMode() async {
    guard camera.state == .running else { return }
    if cameraMode == .manual {
      await applyManualControls()
    } else {
      try? await camera.setAutoExposureFocusWhiteBalance()
    }
  }

  func applyManualControls() async {
    guard camera.state == .running else { return }
    try? await camera.applyCameraSettings(
      isShutterAuto: isShutterAuto,
      shutterDuration: manualExposureDurationSeconds,
      isISOAuto: isISOAuto,
      iso: manualISO,
      isFocusAuto: isFocusAuto,
      focusLensPosition: manualFocusLensPosition,
      isWhiteBalanceAuto: isWhiteBalanceAuto,
      temperatureKelvin: manualTemperatureKelvin
    )
  }

  func refreshManualCapabilities() {
    if let caps = camera.queryCapabilities() {
      manualCapabilities = caps
    }
  }

  func captureCurrentAlignedTarget() {
    guard phase == .capturingPoints,
      !isCapturingPhoto,
      let target = activeTarget,
      let reference = captureReference,
      navigationReading.isAligned
    else { return }
    isCapturingPhoto = true
    Task {
      await capture(target: target, reference: reference)
    }
  }

  func refreshGallery() async {
    isRefreshingGallery = true
    defer { isRefreshingGallery = false }
    do {
      galleryPackages = try await packageStore.listCompletedPackages()
      galleryErrorMessage = nil
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func computeOnDevice(package: CapturePackage, replaceExisting: Bool = false) async {
    phase = .stitching
    stitchProgress = nil
    let usingExperimental =
      stitcherOverride == nil && useExperimentalMetalStitch
    statusMessage = replaceExisting
      ? (usingExperimental
        ? "Recomputing with experimental 3s Metal stitch"
        : "Recomputing panorama on device")
      : (usingExperimental
        ? "Starting experimental 3s Metal stitch"
        : "Starting native stitch")
    camera.stop()
    motion.stop()

    do {
      if replaceExisting {
        try await packageStore.clearEngineOutput(for: package)
      }
      let stitcher = activeStitcher()
      let result = try await stitcher.stitch(package: package) { [weak self] update in
        Task { @MainActor in
          guard let self else { return }
          // Live pipeline stage text; fraction is kept only for diagnostics.
          self.stitchProgress = update.fraction
          self.statusMessage = update.message
        }
      }
      stitchProgress = nil
      phase = .completed(
        CaptureCompletion(
          package: package,
          stitchingResult: result,
          stitchingMessage: nil
        )
      )
      if usingExperimental, let elapsed = result.elapsedSeconds {
        statusMessage = String(
          format: "Experimental Metal stitch complete in %.2fs",
          elapsed
        )
      } else {
        statusMessage = "Panorama complete"
      }
      await refreshGallery()
    } catch {
      stitchProgress = nil
      phase = .completed(
        CaptureCompletion(
          package: package,
          stitchingResult: nil,
          stitchingMessage: error.localizedDescription
        )
      )
      statusMessage = replaceExisting
        ? "Panorama recompute failed"
        : "On-device compute failed"
      galleryErrorMessage = error.localizedDescription
    }
  }

  func deleteFromGallery(_ package: CapturePackage) async {
    do {
      try await packageStore.deletePackage(package)
      await refreshGallery()
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func importEnginePanorama(
    into package: CapturePackage,
    panoramaURL: URL,
    reportURL: URL? = nil
  ) async {
    do {
      let accessing = panoramaURL.startAccessingSecurityScopedResource()
      defer {
        if accessing {
          panoramaURL.stopAccessingSecurityScopedResource()
        }
      }
      var reportAccessing = false
      if let reportURL {
        reportAccessing = reportURL.startAccessingSecurityScopedResource()
      }
      defer {
        if reportAccessing, let reportURL {
          reportURL.stopAccessingSecurityScopedResource()
        }
      }
      try await packageStore.importEnginePanorama(
        into: package,
        panoramaURL: panoramaURL,
        reportURL: reportURL
      )
      galleryErrorMessage = nil
      await refreshGallery()
    } catch {
      galleryErrorMessage = error.localizedDescription
    }
  }

  func makeShareArchive(for package: CapturePackage) async throws -> URL {
    try await packageStore.makeShareArchive(for: package)
  }

  func reportGalleryError(_ message: String) {
    galleryErrorMessage = message
  }

  func clearGalleryError() {
    galleryErrorMessage = nil
  }

  /// Starts or stops the camera when the Capture tab becomes visible or hidden.
  /// Motion keeps running so the capture-reference frame stays valid.
  func setCaptureTabActive(_ active: Bool) {
    let wasActive = isCaptureTabActive
    isCaptureTabActive = active
    guard wasActive != active else { return }

    if active {
      switch phase {
      case .setup:
        startCapture()
      case .awaitingPrimary, .capturingPoints:
        Task { await resumeCameraIfCaptureTabActive() }
      default:
        break
      }
    } else {
      pauseCameraForHiddenCaptureTab()
    }
  }

  private func startCameraIfCaptureTabActive(restoreAuto: Bool) async throws {
    guard isCaptureTabActive else {
      camera.stop()
      return
    }
    let shouldRestoreAuto = cameraMode == .auto && restoreAuto
    try await camera.start(restoreAuto: shouldRestoreAuto)
    guard isCaptureTabActive else {
      camera.stop()
      return
    }
    if cameraMode == .manual {
      await applyManualControls()
    }
    refreshManualCapabilities()
    switch camera.state {
    case .running, .configuring:
      break
    default:
      try await camera.start(restoreAuto: shouldRestoreAuto)
      if !isCaptureTabActive {
        camera.stop()
      } else {
        if cameraMode == .manual {
          await applyManualControls()
        }
        refreshManualCapabilities()
      }
    }
  }

  private func resumeCameraIfCaptureTabActive() async {
    guard isCaptureTabActive else { return }
    do {
      try await startCameraIfCaptureTabActive(restoreAuto: false)
    } catch {
      captureErrorMessage = error.localizedDescription
    }
  }

  private func stopCameraIfCaptureTabInactive() {
    if !isCaptureTabActive {
      camera.stop()
    }
  }

  private func pauseCameraForHiddenCaptureTab() {
    switch phase {
    case .preparing, .awaitingPrimary, .capturingPoints:
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      if !isCapturingPhoto {
        camera.stop()
      }
    default:
      break
    }
  }

  private func handleMotionSample(_ sample: MotionSample?) {
    currentMotionSample = sample
    guard phase == .capturingPoints,
      isCaptureTabActive,
      !isCapturingPhoto,
      let sample,
      let captureReference
    else {
      return
    }

    let remaining = remainingTargets
    let projections = remaining.map { target in
      OrientationMath.projectTargetToScreen(
        sample: sample,
        captureReference: captureReference,
        target: target
      )
    }

    guard
      let nearest = OrientationMath.nearestTarget(
        among: remaining,
        sample: sample,
        captureReference: captureReference,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
    else {
      guidePoints = projections
      activeTarget = nil
      navigationReading = .unavailable
      stableHoldProgress = 0
      alignmentHoldTracker.reset()
      return
    }

    let reading = nearest.reading
    navigationReading = reading
    activeTarget = nearest.target
    guidePoints = projections.map { point in
      var updated = point
      if point.targetID == nearest.target.id {
        updated = CapturePointProjection(
          targetID: point.targetID,
          ring: point.ring,
          offsetX: point.offsetX,
          offsetY: point.offsetY,
          aimingAngleRadians: point.aimingAngleRadians,
          directionErrorDegrees: point.directionErrorDegrees,
          isInFront: point.isInFront,
          isAligned: reading.isAligned
        )
      }
      return updated
    }

    let holdUpdate = alignmentHoldTracker.update(
      isAligned: reading.isAligned,
      timestamp: sample.monotonicTimestampSeconds,
      requiredDuration: configuration.stableHoldDurationSeconds,
      blockedUntilTimestamp: autoCaptureBlockedUntil
    )
    stableHoldProgress = holdUpdate.progress
    traceNavigationIfEnabled(
      sample: sample,
      target: nearest.target,
      reading: reading
    )

    if reading.isAligned {
      if autoFireCapture {
        statusMessage =
          stableHoldProgress < 1
          ? "Hold on point"
          : "Capturing"
      } else {
        statusMessage = "Aligned · Tap shutter to capture"
      }
    } else {
      statusMessage =
        "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
    }

    guard autoFireCapture, reading.isAligned, holdUpdate.shouldCapture else { return }
    isCapturingPhoto = true
    Task {
      await capture(target: nearest.target, reference: captureReference)
    }
  }

  private func performPrimaryCapture() async {
    do {
      let photo = try await camera.capturePhoto()
      let reference = OrientationMath.makeCaptureReference(from: photo.motionSample)
      captureReference = reference
      traceCaptureReferenceIfEnabled()

      let pitch = OrientationMath.currentPitchDegrees(
        sample: photo.motionSample,
        captureReference: reference
      )
      let ring = OrientationMath.classifyCaptureRing(
        pitchDegrees: pitch,
        configuration: configuration
      )
      guard
        let target = OrientationMath.closestTarget(
          in: ring,
          targets: plan.targets,
          sample: photo.motionSample,
          captureReference: reference
        )
      else {
        throw CameraCaptureError.sessionNotConfigured
      }

      let exposureAlignment = OrientationMath.navigationReading(
        sample: photo.motionSample,
        captureReference: reference,
        target: target,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
      let pose = OrientationMath.poseMetadata(
        sample: photo.motionSample,
        captureReference: reference,
        referenceFrameName: motion.referenceFrameName
      )
      let record = try await packageStore.append(
        photo: photo,
        target: target,
        pose: pose,
        alignment: AlignmentMetadata(
          directionErrorDegrees: exposureAlignment.directionErrorDegrees,
          yawErrorDegrees: exposureAlignment.yawErrorDegrees,
          pitchErrorDegrees: exposureAlignment.pitchErrorDegrees,
          requiredToleranceDegrees: configuration.alignmentToleranceDegrees,
          requiredStableDurationSeconds: configuration.stableHoldDurationSeconds
        ),
        primaryCapture: PrimaryCaptureMetadata(
          imageFilename: "",
          targetId: target.id,
          classifiedRing: ring
        )
      )

      capturedFrames.append(record)
      if cameraMode == .auto {
        try await camera.lockExposureFocusWhiteBalance()
      } else {
        await applyManualControls()
      }
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationReading = .unavailable
      isCapturingPhoto = false

      if capturedFrames.count == totalFrameCount {
        await finalizeCapture()
      } else {
        phase = .capturingPoints
        statusMessage =
          "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
        stopCameraIfCaptureTabInactive()
      }
    } catch {
      isCapturingPhoto = false
      captureReference = nil
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      captureErrorMessage = error.localizedDescription
      statusMessage = "Primary capture failed; try again"
      stopCameraIfCaptureTabInactive()
    }
  }

  private func capture(
    target: CaptureTarget,
    reference: CaptureReferenceFrame
  ) async {
    captureErrorMessage = nil
    do {
      let photo = try await camera.capturePhoto()
      let exposureAlignment = OrientationMath.navigationReading(
        sample: photo.motionSample,
        captureReference: reference,
        target: target,
        toleranceDegrees: configuration.alignmentToleranceDegrees
      )
      let pose = OrientationMath.poseMetadata(
        sample: photo.motionSample,
        captureReference: reference,
        referenceFrameName: motion.referenceFrameName
      )
      let record = try await packageStore.append(
        photo: photo,
        target: target,
        pose: pose,
        alignment: AlignmentMetadata(
          directionErrorDegrees: exposureAlignment.directionErrorDegrees,
          yawErrorDegrees: exposureAlignment.yawErrorDegrees,
          pitchErrorDegrees: exposureAlignment.pitchErrorDegrees,
          requiredToleranceDegrees: configuration.alignmentToleranceDegrees,
          requiredStableDurationSeconds: configuration.stableHoldDurationSeconds
        )
      )
      capturedFrames.append(record)
      UINotificationFeedbackGenerator().notificationOccurred(.success)
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      navigationReading = .unavailable
      activeTarget = nil
      isCapturingPhoto = false

      if capturedFrames.count == totalFrameCount {
        await finalizeCapture()
      } else {
        statusMessage =
          "Align center to a point · \(capturedFrames.count) of \(totalFrameCount)"
        stopCameraIfCaptureTabInactive()
      }
    } catch {
      isCapturingPhoto = false
      alignmentHoldTracker.reset()
      stableHoldProgress = 0
      autoCaptureBlockedUntil = ProcessInfo.processInfo.systemUptime + 1
      captureErrorMessage = error.localizedDescription
      statusMessage = "Capture failed; realign to retry"
      stopCameraIfCaptureTabInactive()
    }
  }

  private func finalizeCapture() async {
    statusMessage = "Saving capture to gallery"
    await camera.unlockExposureFocusWhiteBalance()
    camera.stop()
    motion.stop()

    do {
      let package = try await packageStore.finalize()
      phase = .saved(package)
      statusMessage = "Capture saved"
      await refreshGallery()
    } catch {
      phase = .failed(error.localizedDescription)
      statusMessage = "Could not save capture package"
    }
  }

  private func traceCaptureReferenceIfEnabled() {
    guard navigationTraceEnabled, let captureReference else { return }
    print(
      "SPHERA_NAV reference quaternion=\(captureReference.motionQuaternionInterpretation.rawValue)"
    )
  }

  private func traceNavigationIfEnabled(
    sample: MotionSample,
    target: CaptureTarget,
    reading: CaptureNavigationReading
  ) {
    guard navigationTraceEnabled else { return }
    guard sample.monotonicTimestampSeconds - lastNavigationTraceTimestamp >= 0.25 else {
      return
    }
    lastNavigationTraceTimestamp = sample.monotonicTimestampSeconds
    print(
      "SPHERA_NAV target=\(target.id) yawError=\(reading.yawErrorDegrees.formatted(.number.precision(.fractionLength(1)))) pitchError=\(reading.pitchErrorDegrees.formatted(.number.precision(.fractionLength(1)))) directionError=\(reading.directionErrorDegrees.formatted(.number.precision(.fractionLength(1)))) aligned=\(reading.isAligned)"
    )
  }
}

extension CaptureWorkflowPhase {
  fileprivate var isTerminal: Bool {
    switch self {
    case .saved, .completed, .failed:
      true
    default:
      false
    }
  }
}
