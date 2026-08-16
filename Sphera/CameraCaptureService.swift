@preconcurrency import AVFoundation
import Combine
import CoreMedia
import Foundation
import ImageIO
import simd

final class LatestCameraIntrinsicsStore: @unchecked Sendable {
  private let lock = NSLock()
  private var storedSample: CameraIntrinsicsSample?

  var latest: CameraIntrinsicsSample? {
    lock.lock()
    defer { lock.unlock() }
    return storedSample
  }

  func update(_ sample: CameraIntrinsicsSample) {
    lock.lock()
    storedSample = sample
    lock.unlock()
  }

  func clear() {
    lock.lock()
    storedSample = nil
    lock.unlock()
  }
}

struct CameraManualControlCapabilities: Equatable, Sendable {
  let minISO: Float
  let maxISO: Float
  let currentISO: Float
  let minExposureDurationSeconds: Double
  let maxExposureDurationSeconds: Double
  let currentExposureDurationSeconds: Double
  let currentLensPosition: Float
  let minTemperatureKelvin: Float
  let maxTemperatureKelvin: Float
  let currentTemperatureKelvin: Float

  static let `default` = CameraManualControlCapabilities(
    minISO: 50,
    maxISO: 2000,
    currentISO: 100,
    minExposureDurationSeconds: 1.0 / 8000.0,
    maxExposureDurationSeconds: 1.0 / 2.0,
    currentExposureDurationSeconds: 1.0 / 125.0,
    currentLensPosition: 0.5,
    minTemperatureKelvin: 2500,
    maxTemperatureKelvin: 9000,
    currentTemperatureKelvin: 5500
  )
}

@MainActor
final class CameraCaptureService: ObservableObject {
  enum State: Equatable {
    case idle
    case configuring
    case running
    case failed(String)
  }

  @Published private(set) var state: State = .idle
  @Published private(set) var isFeedActive = false

  nonisolated(unsafe) let session = AVCaptureSession()

  private nonisolated(unsafe) let photoOutput = AVCapturePhotoOutput()
  private nonisolated(unsafe) let videoOutput = AVCaptureVideoDataOutput()
  private nonisolated let sessionQueue = DispatchQueue(
    label: "com.sphera.capture.session", qos: .userInitiated)
  private nonisolated let videoOutputQueue = DispatchQueue(
    label: "com.sphera.capture.video-metadata", qos: .userInitiated)
  private nonisolated let motionStore: LatestMotionSampleStore
  private nonisolated let intrinsicsStore = LatestCameraIntrinsicsStore()
  private nonisolated let videoMetadataDelegate: VideoFrameMetadataDelegate
  private nonisolated let photoRegistry = PhotoProcessorRegistry()

  private nonisolated(unsafe) var isConfigured = false
  private nonisolated(unsafe) var captureDevice: AVCaptureDevice?
  private nonisolated(unsafe) var maximumPhotoDimensions = CMVideoDimensions(width: 0, height: 0)
  private nonisolated(unsafe) var sessionEpoch = 0
  private nonisolated(unsafe) var areExposureParametersLocked = false
  private nonisolated let captureRotationDegrees = 90.0

  init(motionStore: LatestMotionSampleStore) {
    self.motionStore = motionStore
    let intrinsics = intrinsicsStore
    let rotationDegrees = captureRotationDegrees
    let delegate = VideoFrameMetadataDelegate(
      store: intrinsics,
      captureRotationDegrees: rotationDegrees,
      onFrameReceived: nil
    )
    self.videoMetadataDelegate = delegate
    delegate.setOnFrameReceived { [weak self] in
      Task { @MainActor [weak self] in
        guard let self, !self.isFeedActive else { return }
        self.isFeedActive = true
      }
    }
  }

  func configureDelegate(onFirstFrame: @escaping @MainActor () -> Void) {
    // Convenience if needed
  }

  func start(restoreAuto: Bool = true) async throws {
    guard state != .running else { return }
    sessionEpoch += 1
    let epoch = sessionEpoch
    state = .configuring
    isFeedActive = false

    guard await requestCameraAuthorization() else {
      guard epoch == sessionEpoch else { return }
      state = .failed(CameraCaptureError.permissionDenied.localizedDescription)
      throw CameraCaptureError.permissionDenied
    }
    guard epoch == sessionEpoch else { return }

    do {
      try await performOnSessionQueue { [self] in
        guard epoch == sessionEpoch else { return }
        if !isConfigured {
          try configureSession()
          isConfigured = true
        } else if restoreAuto {
          areExposureParametersLocked = false
          if let captureDevice {
            try Self.applyContinuousAutoExposureFocusWhiteBalance(on: captureDevice)
          }
        } else if areExposureParametersLocked, let captureDevice {
          try Self.lockExposureFocusWhiteBalance(on: captureDevice)
        }
        guard epoch == sessionEpoch else { return }
        guard !session.isRunning else { return }
        intrinsicsStore.clear()
        session.startRunning()
      }
      guard epoch == sessionEpoch else { return }
      try await waitForIntrinsics()
      guard epoch == sessionEpoch else { return }
      state = .running
      isFeedActive = true
    } catch {
      guard epoch == sessionEpoch else { return }
      state = .failed(error.localizedDescription)
      throw error
    }
  }

  func stop() {
    sessionEpoch += 1
    state = .idle
    isFeedActive = false
    stopSessionIfRunning()
  }

  func stopAndWait() async {
    sessionEpoch += 1
    state = .idle
    isFeedActive = false
    await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
      sessionQueue.async { [session] in
        if session.isRunning {
          session.stopRunning()
        }
        continuation.resume()
      }
    }
  }

  func queryCapabilities() -> CameraManualControlCapabilities? {
    guard let device = captureDevice else { return nil }
    let format = device.activeFormat
    let duration = CMTimeGetSeconds(device.exposureDuration)
    let safeDuration = duration.isFinite && duration > 0 ? duration : 1.0 / 125.0
    let tempAndTint = device.temperatureAndTintValues(for: device.deviceWhiteBalanceGains)
    let temp = tempAndTint.temperature
    let safeTemp = temp.isFinite && temp > 1000 ? temp : 5500.0

    return CameraManualControlCapabilities(
      minISO: format.minISO,
      maxISO: format.maxISO,
      currentISO: device.iso,
      minExposureDurationSeconds: CMTimeGetSeconds(format.minExposureDuration),
      maxExposureDurationSeconds: CMTimeGetSeconds(format.maxExposureDuration),
      currentExposureDurationSeconds: safeDuration,
      currentLensPosition: device.lensPosition,
      minTemperatureKelvin: 2500,
      maxTemperatureKelvin: 9000,
      currentTemperatureKelvin: safeTemp
    )
  }

  func applyCameraSettings(
    isShutterAuto: Bool,
    shutterDuration: Double,
    isISOAuto: Bool,
    iso: Float,
    isFocusAuto: Bool,
    focusLensPosition: Float,
    isWhiteBalanceAuto: Bool,
    temperatureKelvin: Float
  ) async throws {
    guard state == .running else { return }
    try await performOnSessionQueue { [self] in
      guard let device = captureDevice else { return }
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }

      let format = device.activeFormat

      // 1. Exposure (Shutter Speed & ISO)
      if isShutterAuto && isISOAuto {
        if device.isExposureModeSupported(.continuousAutoExposure) {
          device.exposureMode = .continuousAutoExposure
        }
      } else if device.isExposureModeSupported(.custom) {
        let targetISO: Float
        if isISOAuto {
          targetISO = min(max(device.iso, format.minISO), format.maxISO)
        } else {
          targetISO = min(max(iso, format.minISO), format.maxISO)
        }

        let minDur = CMTimeGetSeconds(format.minExposureDuration)
        let maxDur = CMTimeGetSeconds(format.maxExposureDuration)
        let targetDuration: Double
        if isShutterAuto {
          let currentDur = CMTimeGetSeconds(device.exposureDuration)
          targetDuration = currentDur.isFinite && currentDur > 0 ? min(max(currentDur, minDur), maxDur) : 1.0 / 125.0
        } else {
          targetDuration = min(max(shutterDuration, minDur), maxDur)
        }
        let time = CMTime(seconds: targetDuration, preferredTimescale: 1_000_000)
        device.setExposureModeCustom(duration: time, iso: targetISO, completionHandler: nil)
      }

      // 2. Focus
      if isFocusAuto {
        if device.isFocusModeSupported(.continuousAutoFocus) {
          device.focusMode = .continuousAutoFocus
        } else if device.isFocusModeSupported(.autoFocus) {
          device.focusMode = .autoFocus
        }
      } else if device.isLockingFocusWithCustomLensPositionSupported {
        let clampedFocus = min(max(focusLensPosition, 0), 1)
        device.setFocusModeLocked(lensPosition: clampedFocus, completionHandler: nil)
      }

      // 3. White Balance
      if isWhiteBalanceAuto {
        if device.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) {
          device.whiteBalanceMode = .continuousAutoWhiteBalance
        }
      } else if device.isLockingWhiteBalanceWithCustomDeviceGainsSupported {
        let clampedTemp = min(max(temperatureKelvin, 2500), 9000)
        let tempAndTint = AVCaptureDevice.WhiteBalanceTemperatureAndTintValues(
          temperature: clampedTemp,
          tint: 0
        )
        var gains = device.deviceWhiteBalanceGains(for: tempAndTint)
        let maxGain = device.maxWhiteBalanceGain
        gains.redGain = min(max(gains.redGain, 1), maxGain)
        gains.greenGain = min(max(gains.greenGain, 1), maxGain)
        gains.blueGain = min(max(gains.blueGain, 1), maxGain)
        device.setWhiteBalanceModeLocked(with: gains, completionHandler: nil)
      }

      areExposureParametersLocked = (!isShutterAuto || !isISOAuto || !isFocusAuto || !isWhiteBalanceAuto)
    }
  }

  func setManualExposure(iso: Float, durationSeconds: Double) async throws {
    guard state == .running else { return }
    try await performOnSessionQueue { [self] in
      guard let device = captureDevice else { return }
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }
      if device.isExposureModeSupported(.custom) {
        let format = device.activeFormat
        let clampedISO = min(max(iso, format.minISO), format.maxISO)
        let minDuration = CMTimeGetSeconds(format.minExposureDuration)
        let maxDuration = CMTimeGetSeconds(format.maxExposureDuration)
        let clampedDuration = min(max(durationSeconds, minDuration), maxDuration)
        let time = CMTime(seconds: clampedDuration, preferredTimescale: 1_000_000)
        device.setExposureModeCustom(duration: time, iso: clampedISO, completionHandler: nil)
        areExposureParametersLocked = true
      }
    }
  }

  func setManualFocus(lensPosition: Float) async throws {
    guard state == .running else { return }
    try await performOnSessionQueue { [self] in
      guard let device = captureDevice else { return }
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }
      let clamped = min(max(lensPosition, 0), 1)
      if device.isLockingFocusWithCustomLensPositionSupported {
        device.setFocusModeLocked(lensPosition: clamped, completionHandler: nil)
        areExposureParametersLocked = true
      }
    }
  }

  func setManualWhiteBalance(temperatureKelvin: Float) async throws {
    guard state == .running else { return }
    try await performOnSessionQueue { [self] in
      guard let device = captureDevice else { return }
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }
      let clampedTemp = min(max(temperatureKelvin, 2500), 9000)
      let tempAndTint = AVCaptureDevice.WhiteBalanceTemperatureAndTintValues(
        temperature: clampedTemp,
        tint: 0
      )
      var gains = device.deviceWhiteBalanceGains(for: tempAndTint)
      let maxGain = device.maxWhiteBalanceGain
      gains.redGain = min(max(gains.redGain, 1), maxGain)
      gains.greenGain = min(max(gains.greenGain, 1), maxGain)
      gains.blueGain = min(max(gains.blueGain, 1), maxGain)
      if device.isLockingWhiteBalanceWithCustomDeviceGainsSupported {
        device.setWhiteBalanceModeLocked(with: gains, completionHandler: nil)
        areExposureParametersLocked = true
      }
    }
  }

  func setAutoExposureFocusWhiteBalance() async throws {
    areExposureParametersLocked = false
    guard state == .running else { return }
    try await performOnSessionQueue { [self] in
      guard let device = captureDevice else { return }
      try Self.applyContinuousAutoExposureFocusWhiteBalance(on: device)
    }
  }

  private func stopSessionIfRunning() {
    sessionQueue.async { [session] in
      if session.isRunning {
        session.stopRunning()
      }
    }
  }

  func capturePhoto() async throws -> CapturedPhoto {
    guard state == .running else {
      throw CameraCaptureError.sessionNotRunning
    }

    return try await withCheckedThrowingContinuation { continuation in
      sessionQueue.async { [self] in
        guard let captureDevice else {
          continuation.resume(throwing: CameraCaptureError.sessionNotConfigured)
          return
        }
        guard intrinsicsStore.latest != nil else {
          continuation.resume(throwing: CameraCaptureError.intrinsicsUnavailable)
          return
        }

        let settings = AVCapturePhotoSettings(
          format: [AVVideoCodecKey: AVVideoCodecType.jpeg]
        )
        settings.photoQualityPrioritization = .quality
        settings.flashMode = .off
        settings.maxPhotoDimensions = maximumPhotoDimensions

        let processorID = UUID()
        let processor = PhotoCaptureProcessor(
          motionStore: motionStore,
          intrinsicsStore: intrinsicsStore,
          lensMetadata: Self.makeLensMetadata(
            device: captureDevice,
            maximumPhotoDimensions: maximumPhotoDimensions
          )
        ) { [photoRegistry] result in
          photoRegistry.remove(processorID)
          continuation.resume(with: result)
        }
        photoRegistry.insert(processor, id: processorID)
        photoOutput.capturePhoto(with: settings, delegate: processor)
      }
    }
  }

  /// Freezes AE/AF/AWB at the current device settings after the primary capture.
  func lockExposureFocusWhiteBalance() async throws {
    guard state == .running else {
      throw CameraCaptureError.sessionNotRunning
    }
    try await performOnSessionQueue { [self] in
      guard let captureDevice else {
        throw CameraCaptureError.sessionNotConfigured
      }
      try Self.lockExposureFocusWhiteBalance(on: captureDevice)
      areExposureParametersLocked = true
    }
  }

  /// Restores continuous AE/AF/AWB after a locked capture session.
  func unlockExposureFocusWhiteBalance() async {
    areExposureParametersLocked = false
    guard state == .running else { return }
    try? await performOnSessionQueue { [self] in
      guard let captureDevice else { return }
      try Self.applyContinuousAutoExposureFocusWhiteBalance(on: captureDevice)
    }
  }

  private nonisolated static func lockExposureFocusWhiteBalance(
    on device: AVCaptureDevice
  ) throws {
    do {
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }

      let duration = device.exposureDuration
      let iso = device.iso
      if device.isExposureModeSupported(.custom) {
        let clampedDuration = min(
          max(duration, device.activeFormat.minExposureDuration),
          device.activeFormat.maxExposureDuration
        )
        let clampedISO = min(
          max(iso, device.activeFormat.minISO),
          device.activeFormat.maxISO
        )
        device.setExposureModeCustom(
          duration: clampedDuration,
          iso: clampedISO,
          completionHandler: nil
        )
      } else if device.isExposureModeSupported(.locked) {
        device.exposureMode = .locked
      }

      let lensPosition = device.lensPosition
      if device.isLockingFocusWithCustomLensPositionSupported {
        device.setFocusModeLocked(lensPosition: lensPosition, completionHandler: nil)
      } else if device.isFocusModeSupported(.locked) {
        device.focusMode = .locked
      }

      let gains = device.deviceWhiteBalanceGains
      if device.isLockingWhiteBalanceWithCustomDeviceGainsSupported {
        let maxGain = device.maxWhiteBalanceGain
        let clamped = AVCaptureDevice.WhiteBalanceGains(
          redGain: min(max(gains.redGain, 1), maxGain),
          greenGain: min(max(gains.greenGain, 1), maxGain),
          blueGain: min(max(gains.blueGain, 1), maxGain)
        )
        device.setWhiteBalanceModeLocked(with: clamped, completionHandler: nil)
      } else if device.isWhiteBalanceModeSupported(.locked) {
        device.whiteBalanceMode = .locked
      }

      device.isSubjectAreaChangeMonitoringEnabled = false
    } catch {
      throw CameraCaptureError.couldNotConfigureDevice(error)
    }
  }

  private func requestCameraAuthorization() async -> Bool {
    switch AVCaptureDevice.authorizationStatus(for: .video) {
    case .authorized:
      true
    case .notDetermined:
      await withCheckedContinuation { continuation in
        AVCaptureDevice.requestAccess(for: .video) { granted in
          continuation.resume(returning: granted)
        }
      }
    case .denied, .restricted:
      false
    @unknown default:
      false
    }
  }

  private nonisolated func configureSession() throws {
    session.beginConfiguration()
    do {
      try configureSessionContents()
      session.commitConfiguration()
    } catch {
      session.outputs.forEach(session.removeOutput)
      session.inputs.forEach(session.removeInput)
      captureDevice = nil
      maximumPhotoDimensions = CMVideoDimensions(width: 0, height: 0)
      session.commitConfiguration()
      throw error
    }
  }

  private nonisolated func configureSessionContents() throws {
    guard session.canSetSessionPreset(.photo) else {
      throw CameraCaptureError.photoPresetUnavailable
    }
    session.sessionPreset = .photo

    guard
      let device = AVCaptureDevice.default(
        .builtInUltraWideCamera,
        for: .video,
        position: .back
      )
    else {
      throw CameraCaptureError.ultraWideCameraUnavailable
    }

    let input: AVCaptureDeviceInput
    do {
      input = try AVCaptureDeviceInput(device: device)
    } catch {
      throw CameraCaptureError.couldNotCreateInput(error)
    }
    guard session.canAddInput(input) else {
      throw CameraCaptureError.couldNotAddInput
    }
    session.addInput(input)

    guard session.canAddOutput(photoOutput) else {
      throw CameraCaptureError.couldNotAddPhotoOutput
    }
    session.addOutput(photoOutput)
    photoOutput.maxPhotoQualityPrioritization = .quality
    guard photoOutput.availablePhotoCodecTypes.contains(.jpeg) else {
      throw CameraCaptureError.jpegPhotoUnavailable
    }

    videoOutput.alwaysDiscardsLateVideoFrames = true
    videoOutput.videoSettings = [
      kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
    ]
    videoOutput.setSampleBufferDelegate(videoMetadataDelegate, queue: videoOutputQueue)
    guard session.canAddOutput(videoOutput) else {
      throw CameraCaptureError.couldNotAddMetadataOutput
    }
    session.addOutput(videoOutput)

    try configure(device)
    captureDevice = device

    guard
      let maximumDimensions = device.activeFormat.supportedMaxPhotoDimensions.max(by: {
        Int64($0.width) * Int64($0.height) < Int64($1.width) * Int64($1.height)
      })
    else {
      throw CameraCaptureError.highResolutionPhotoUnavailable
    }
    maximumPhotoDimensions = maximumDimensions
    photoOutput.maxPhotoDimensions = maximumDimensions

    if let photoConnection = photoOutput.connection(with: .video),
      photoConnection.isVideoRotationAngleSupported(captureRotationDegrees)
    {
      photoConnection.videoRotationAngle = captureRotationDegrees
    }

    guard let metadataConnection = videoOutput.connection(with: .video) else {
      throw CameraCaptureError.couldNotCreateMetadataConnection
    }
    if metadataConnection.isVideoStabilizationSupported {
      metadataConnection.preferredVideoStabilizationMode = .off
    }
    if metadataConnection.isVideoRotationAngleSupported(captureRotationDegrees) {
      metadataConnection.videoRotationAngle = captureRotationDegrees
    }
    guard metadataConnection.isCameraIntrinsicMatrixDeliverySupported else {
      throw CameraCaptureError.cameraIntrinsicsUnsupported
    }
    metadataConnection.isCameraIntrinsicMatrixDeliveryEnabled = true
  }

  private nonisolated func configure(_ device: AVCaptureDevice) throws {
    do {
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }

      device.videoZoomFactor = 1
      Self.setContinuousAutoExposureFocusWhiteBalance(on: device)
    } catch {
      throw CameraCaptureError.couldNotConfigureDevice(error)
    }
  }

  private nonisolated static func applyContinuousAutoExposureFocusWhiteBalance(
    on device: AVCaptureDevice
  ) throws {
    do {
      try device.lockForConfiguration()
      defer { device.unlockForConfiguration() }
      Self.setContinuousAutoExposureFocusWhiteBalance(on: device)
    } catch {
      throw CameraCaptureError.couldNotConfigureDevice(error)
    }
  }

  private nonisolated static func setContinuousAutoExposureFocusWhiteBalance(
    on device: AVCaptureDevice
  ) {
    if device.isFocusModeSupported(.continuousAutoFocus) {
      device.focusMode = .continuousAutoFocus
    }
    if device.isExposureModeSupported(.continuousAutoExposure) {
      device.exposureMode = .continuousAutoExposure
    }
    if device.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) {
      device.whiteBalanceMode = .continuousAutoWhiteBalance
    }
    device.isSubjectAreaChangeMonitoringEnabled = true
  }

  private func waitForIntrinsics(timeoutSeconds: Double = 3) async throws {
    let deadline = ProcessInfo.processInfo.systemUptime + timeoutSeconds
    while ProcessInfo.processInfo.systemUptime < deadline {
      if intrinsicsStore.latest != nil {
        return
      }
      try await Task.sleep(for: .milliseconds(50))
    }
    throw CameraCaptureError.intrinsicsUnavailable
  }

  private func performOnSessionQueue(
    _ operation: @escaping @Sendable () throws -> Void
  ) async throws {
    try await withCheckedThrowingContinuation { continuation in
      sessionQueue.async {
        do {
          try operation()
          continuation.resume()
        } catch {
          continuation.resume(throwing: error)
        }
      }
    }
  }

  private nonisolated static func makeLensMetadata(
    device: AVCaptureDevice,
    maximumPhotoDimensions: CMVideoDimensions
  ) -> LensMetadata {
    LensMetadata(
      deviceType: device.deviceType.rawValue,
      position: device.position.description,
      uniqueID: device.uniqueID,
      modelID: device.modelID,
      localizedName: device.localizedName,
      activeFormatDescription: String(describing: device.activeFormat.formatDescription),
      videoFieldOfViewDegrees: Double(device.activeFormat.videoFieldOfView),
      minimumZoomFactor: Double(device.minAvailableVideoZoomFactor),
      maximumZoomFactor: Double(device.maxAvailableVideoZoomFactor),
      zoomFactor: Double(device.videoZoomFactor),
      lensPosition: Double(device.lensPosition),
      maximumPhotoWidth: Int(maximumPhotoDimensions.width),
      maximumPhotoHeight: Int(maximumPhotoDimensions.height)
    )
  }
}

private final class VideoFrameMetadataDelegate: NSObject,
  AVCaptureVideoDataOutputSampleBufferDelegate, @unchecked Sendable
{
  private let store: LatestCameraIntrinsicsStore
  private let captureRotationDegrees: Double
  private var onFrameReceived: (@Sendable () -> Void)?

  init(
    store: LatestCameraIntrinsicsStore,
    captureRotationDegrees: Double,
    onFrameReceived: (@Sendable () -> Void)? = nil
  ) {
    self.store = store
    self.captureRotationDegrees = captureRotationDegrees
    self.onFrameReceived = onFrameReceived
  }

  func setOnFrameReceived(_ callback: (@Sendable () -> Void)?) {
    self.onFrameReceived = callback
  }

  func captureOutput(
    _ output: AVCaptureOutput,
    didOutput sampleBuffer: CMSampleBuffer,
    from connection: AVCaptureConnection
  ) {
    onFrameReceived?()
    guard
      let attachment = CMGetAttachment(
        sampleBuffer,
        key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix,
        attachmentModeOut: nil
      ) as? Data
    else {
      return
    }
    guard attachment.count >= MemoryLayout<simd_float3x3>.size else { return }

    var matrix = matrix_identity_float3x3
    _ = withUnsafeMutableBytes(of: &matrix) { destination in
      attachment.copyBytes(to: destination)
    }

    guard let formatDescription = CMSampleBufferGetFormatDescription(sampleBuffer) else { return }
    let dimensions = CMVideoFormatDescriptionGetDimensions(formatDescription)
    let timestamp = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer))
    guard timestamp.isFinite else { return }

    store.update(
      CameraIntrinsicsSample(
        monotonicTimestampSeconds: timestamp,
        fx: Double(matrix.columns.0.x),
        fy: Double(matrix.columns.1.y),
        cx: Double(matrix.columns.2.x),
        cy: Double(matrix.columns.2.y),
        referenceWidth: Int(dimensions.width),
        referenceHeight: Int(dimensions.height),
        captureRotationDegrees: captureRotationDegrees
      )
    )
  }
}

private final class PhotoProcessorRegistry: @unchecked Sendable {
  private let lock = NSLock()
  private var processors: [UUID: PhotoCaptureProcessor] = [:]

  func insert(_ processor: PhotoCaptureProcessor, id: UUID) {
    lock.lock()
    processors[id] = processor
    lock.unlock()
  }

  func remove(_ id: UUID) {
    lock.lock()
    processors[id] = nil
    lock.unlock()
  }
}

private final class PhotoCaptureProcessor: NSObject, AVCapturePhotoCaptureDelegate,
  @unchecked Sendable
{
  private let motionStore: LatestMotionSampleStore
  private let intrinsicsStore: LatestCameraIntrinsicsStore
  private let lensMetadata: LensMetadata
  private let completion: @Sendable (Result<CapturedPhoto, Error>) -> Void

  private var captureTimestamp: Date?
  private var exposureMotionSample: MotionSample?
  private var exposureIntrinsicsSample: CameraIntrinsicsSample?
  private var photoData: Data?
  private var photoMetadata: PhotoMetadata?
  private var processingError: Error?
  private var didComplete = false

  init(
    motionStore: LatestMotionSampleStore,
    intrinsicsStore: LatestCameraIntrinsicsStore,
    lensMetadata: LensMetadata,
    completion: @escaping @Sendable (Result<CapturedPhoto, Error>) -> Void
  ) {
    self.motionStore = motionStore
    self.intrinsicsStore = intrinsicsStore
    self.lensMetadata = lensMetadata
    self.completion = completion
  }

  func photoOutput(
    _ output: AVCapturePhotoOutput,
    willCapturePhotoFor resolvedSettings: AVCaptureResolvedPhotoSettings
  ) {
    captureTimestamp = Date()
    exposureMotionSample = motionStore.latest
    exposureIntrinsicsSample = intrinsicsStore.latest
  }

  func photoOutput(
    _ output: AVCapturePhotoOutput,
    didFinishProcessingPhoto photo: AVCapturePhoto,
    error: Error?
  ) {
    if let error {
      processingError = error
      return
    }
    guard let data = photo.fileDataRepresentation() else {
      processingError = CameraCaptureError.photoDataUnavailable
      return
    }
    photoData = data
    photoMetadata = PhotoMetadata(photo: photo, encodedData: data)
  }

  func photoOutput(
    _ output: AVCapturePhotoOutput,
    didFinishCaptureFor resolvedSettings: AVCaptureResolvedPhotoSettings,
    error: Error?
  ) {
    guard !didComplete else { return }
    didComplete = true

    if let error = error ?? processingError {
      completion(.failure(error))
      return
    }
    guard let photoData, let photoMetadata else {
      completion(.failure(CameraCaptureError.photoDataUnavailable))
      return
    }
    guard let exposureMotionSample else {
      completion(.failure(CameraCaptureError.motionSampleUnavailable))
      return
    }
    guard let exposureIntrinsicsSample else {
      completion(.failure(CameraCaptureError.intrinsicsUnavailable))
      return
    }

    completion(
      .success(
        CapturedPhoto(
          data: photoData,
          captureTimestamp: captureTimestamp ?? Date(),
          motionSample: exposureMotionSample,
          intrinsicsSample: exposureIntrinsicsSample,
          lensMetadata: lensMetadata,
          photoMetadata: photoMetadata
        )
      )
    )
  }
}

extension PhotoMetadata {
  fileprivate init(photo: AVCapturePhoto, encodedData: Data) {
    let source = CGImageSourceCreateWithData(encodedData as CFData, nil)
    let encodedProperties = source.flatMap {
      CGImageSourceCopyPropertiesAtIndex($0, 0, nil) as? [String: Any]
    }
    let metadata = encodedProperties ?? photo.metadata
    let exif = metadata[kCGImagePropertyExifDictionary as String] as? [String: Any]
    let tiff = metadata[kCGImagePropertyTIFFDictionary as String] as? [String: Any]
    let dimensions = photo.resolvedSettings.photoDimensions

    self.init(
      codec: AVVideoCodecType.jpeg.rawValue,
      width: Self.integer(metadata[kCGImagePropertyPixelWidth as String])
        ?? Int(dimensions.width),
      height: Self.integer(metadata[kCGImagePropertyPixelHeight as String])
        ?? Int(dimensions.height),
      exifOrientation: Self.integer(metadata[kCGImagePropertyOrientation as String]),
      exposureDurationSeconds: Self.number(exif?[kCGImagePropertyExifExposureTime as String]),
      iso: Self.firstNumber(exif?[kCGImagePropertyExifISOSpeedRatings as String]),
      aperture: Self.number(exif?[kCGImagePropertyExifFNumber as String]),
      focalLengthMillimeters: Self.number(exif?[kCGImagePropertyExifFocalLength as String]),
      focalLength35mmEquivalent: Self.number(
        exif?[kCGImagePropertyExifFocalLenIn35mmFilm as String]),
      brightnessValue: Self.number(exif?[kCGImagePropertyExifBrightnessValue as String]),
      exposureBiasValue: Self.number(exif?[kCGImagePropertyExifExposureBiasValue as String]),
      lensMake: exif?[kCGImagePropertyExifLensMake as String] as? String
        ?? tiff?[kCGImagePropertyTIFFMake as String] as? String,
      lensModel: exif?[kCGImagePropertyExifLensModel as String] as? String
        ?? tiff?[kCGImagePropertyTIFFModel as String] as? String
    )
  }

  fileprivate static func number(_ value: Any?) -> Double? {
    (value as? NSNumber)?.doubleValue
  }

  fileprivate static func firstNumber(_ value: Any?) -> Double? {
    if let values = value as? [NSNumber] {
      return values.first?.doubleValue
    }
    return number(value)
  }

  fileprivate static func integer(_ value: Any?) -> Int? {
    (value as? NSNumber)?.intValue
  }
}

enum CameraCaptureError: LocalizedError {
  case permissionDenied
  case ultraWideCameraUnavailable
  case photoPresetUnavailable
  case couldNotCreateInput(Error)
  case couldNotAddInput
  case couldNotAddPhotoOutput
  case couldNotAddMetadataOutput
  case couldNotCreateMetadataConnection
  case couldNotConfigureDevice(Error)
  case highResolutionPhotoUnavailable
  case jpegPhotoUnavailable
  case cameraIntrinsicsUnsupported
  case intrinsicsUnavailable
  case sessionNotConfigured
  case sessionNotRunning
  case photoDataUnavailable
  case motionSampleUnavailable

  var errorDescription: String? {
    switch self {
    case .permissionDenied:
      "Camera access is required. Enable it in Settings and reopen Sphera."
    case .ultraWideCameraUnavailable:
      "This device does not provide a back ultra-wide camera."
    case .photoPresetUnavailable:
      "The camera cannot be configured for full-resolution photo capture."
    case .couldNotCreateInput(let error):
      "The ultra-wide camera input could not be created: \(error.localizedDescription)"
    case .couldNotAddInput:
      "The ultra-wide camera input could not be added to the capture session."
    case .couldNotAddPhotoOutput:
      "The full-resolution photo output could not be configured."
    case .couldNotAddMetadataOutput:
      "The camera calibration metadata output could not be configured."
    case .couldNotCreateMetadataConnection:
      "The camera calibration metadata connection is unavailable."
    case .couldNotConfigureDevice(let error):
      "The ultra-wide camera could not be configured: \(error.localizedDescription)"
    case .highResolutionPhotoUnavailable:
      "The ultra-wide camera did not report a high-resolution photo format."
    case .jpegPhotoUnavailable:
      "The ultra-wide camera does not provide the JPEG format required by the current Sphera engine."
    case .cameraIntrinsicsUnsupported:
      "This ultra-wide camera format does not deliver a camera intrinsic matrix."
    case .intrinsicsUnavailable:
      "No current camera intrinsic matrix is available."
    case .sessionNotConfigured:
      "The camera session has not been configured."
    case .sessionNotRunning:
      "The camera session is not running."
    case .photoDataUnavailable:
      "AVFoundation did not return the captured photo data."
    case .motionSampleUnavailable:
      "No CoreMotion pose was available at exposure time."
    }
  }
}

extension AVCaptureDevice.Position {
  fileprivate var description: String {
    switch self {
    case .back: "back"
    case .front: "front"
    case .unspecified: "unspecified"
    @unknown default: "unknown"
    }
  }
}
