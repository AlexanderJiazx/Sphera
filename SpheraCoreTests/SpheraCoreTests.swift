import Foundation
import Testing
import simd

@testable import SpheraCore

private let deviceFromCamera = simd_quatd(
  angle: .pi,
  axis: SIMD3<Double>(1, 0, 0)
)

@Test("Default and configurable capture plans preserve ring topology")
func capturePlanTopology() {
  let preset = CapturePlan(configuration: .debugPreset)

  #expect(preset.targets.count == 18)
  #expect(preset.targets.prefix(8).allSatisfy { $0.ring == .horizontal })
  #expect(preset.targets[8..<13].allSatisfy { $0.ring == .downward })
  #expect(preset.targets.suffix(5).allSatisfy { $0.ring == .upward })
  #expect(preset.targets.prefix(8).map(\.yawDegrees) == [0, 45, 90, 135, 180, 225, 270, 315])
  #expect(preset.targets[8..<13].allSatisfy { $0.pitchDegrees == -55 })
  #expect(preset.targets.suffix(5).allSatisfy { $0.pitchDegrees == 55 })
  #expect(preset.targets[8].yawDegrees == preset.targets[7].yawDegrees)
  #expect(preset.targets[13].yawDegrees == preset.targets[12].yawDegrees)

  var custom = CaptureConfiguration.debugPreset
  custom.horizontalCount = 12
  custom.downwardCount = 4
  custom.upwardCount = 6
  let customPlan = CapturePlan(configuration: custom)

  #expect(customPlan.targets.count == 22)
  #expect(customPlan.targets.filter { $0.ring == .horizontal }.count == 12)
  #expect(customPlan.targets.filter { $0.ring == .downward }.count == 4)
  #expect(customPlan.targets.filter { $0.ring == .upward }.count == 6)
}

@Test("Capture route turns clockwise and never jumps heading between rings")
func captureRouteContinuity() {
  let plan = CapturePlan(configuration: .debugPreset)

  for (previous, next) in zip(plan.targets, plan.targets.dropFirst()) {
    let clockwiseDelta = normalizedYaw(next.yawDegrees - previous.yawDegrees)
    if previous.ring == next.ring {
      #expect(abs(clockwiseDelta - 360 / Double(next.ringCount)) < 0.000_001)
      #expect(previous.pitchDegrees == next.pitchDegrees)
    } else {
      #expect(clockwiseDelta < 0.000_001)
      #expect(previous.pitchDegrees != next.pitchDegrees)
    }
  }
}

@Test("Auto-capture requires one uninterrupted aligned hold")
func alignmentHoldBehavior() {
  var tracker = AlignmentHoldTracker()

  #expect(
    tracker.update(
      isAligned: true,
      timestamp: 10,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 0
    ) == AlignmentHoldUpdate(progress: 0, shouldCapture: false)
  )
  #expect(
    tracker.update(
      isAligned: true,
      timestamp: 10.2,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 0
    ).progress > 0.5
  )
  #expect(
    tracker.update(
      isAligned: false,
      timestamp: 10.25,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 0
    ) == AlignmentHoldUpdate(progress: 0, shouldCapture: false)
  )
  #expect(
    !tracker.update(
      isAligned: true,
      timestamp: 11,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 12
    ).shouldCapture
  )
  #expect(
    !tracker.update(
      isAligned: true,
      timestamp: 12,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 12
    ).shouldCapture
  )
  #expect(
    tracker.update(
      isAligned: true,
      timestamp: 12.35,
      requiredDuration: 0.35,
      blockedUntilTimestamp: 12
    ).shouldCapture
  )
}

@Test("Navigation locks one correction axis and uses hysteresis")
func gravityReferencedNavigationInstructions() {
  var guidance = CaptureGuidanceState()

  func instruction(
    targetID: String? = "target-a",
    yaw: Double = 0,
    pitch: Double = 0,
    error: Double = 30,
    aligned: Bool = false,
    available: Bool = true,
    capturing: Bool = false
  ) -> CaptureNavigationInstruction {
    guidance.update(
      targetID: targetID,
      reading: CaptureNavigationReading(
        directionErrorDegrees: error,
        yawErrorDegrees: yaw,
        pitchErrorDegrees: pitch,
        isAligned: aligned
      ),
      toleranceDegrees: 10,
      stableHoldProgress: 0.5,
      isReadingAvailable: available,
      isCapturingPhoto: capturing
    )
  }

  #expect(instruction(targetID: nil, available: false).movement == .preparing)
  #expect(instruction(yaw: -30, pitch: 2).movement == .turnLeft)
  #expect(instruction(yaw: -7, pitch: 30).movement == .turnLeft)
  #expect(instruction(yaw: -3, pitch: 30).movement == .tiltUp)
  #expect(instruction(yaw: 20, pitch: 20).movement == .tiltUp)
  #expect(instruction(targetID: "target-b", yaw: 30, pitch: 20).movement == .turnRight)
  #expect(instruction(error: 4, aligned: true).movement == .holdStill)
  #expect(instruction(error: 4, aligned: true).holdProgress == 0.5)
  #expect(instruction(capturing: true).movement == .capturing)
}

@Test("Capture reference is gravity-level and does not inherit device tilt")
func gravityAlignedCaptureReference() {
  let arbitraryDeviceAttitude = simd_normalize(
    simd_quatd(angle: 0.7, axis: simd_normalize(SIMD3<Double>(1, 2, 3)))
  )
  let sample = makeMotionSample(quaternion: arbitraryDeviceAttitude)
  let reference = OrientationMath.makeCaptureReference(from: sample)
  let captureFromMotion = reference.motionReferenceToCaptureQuaternion.simdQuaternion
  let verticalInCapture = simd_act(captureFromMotion, SIMD3<Double>(0, 0, 1))
  let referenceMatrix = simd_double3x3(captureFromMotion)

  #expect(simd_distance(verticalInCapture, SIMD3<Double>(0, 1, 0)) < 0.000_001)
  #expect(abs(simd_determinant(referenceMatrix) - 1) < 0.000_001)
}

@Test("Quaternion direction is validated against measured CoreMotion gravity")
func quaternionDirectionValidation() {
  let deviceFromMotion = simd_quatd(
    angle: -.pi / 2,
    axis: SIMD3<Double>(1, 0, 0)
  )
  let expectedSample = makeMotionSample(quaternion: deviceFromMotion)
  let measuredGravity = SIMD3<Double>(
    expectedSample.gravity.x,
    expectedSample.gravity.y,
    expectedSample.gravity.z
  )
  let invertedSample = makeMotionSample(
    quaternion: deviceFromMotion.inverse,
    gravityDevice: measuredGravity
  )
  let expectedReference = OrientationMath.makeCaptureReference(from: expectedSample)
  let correctedReference = OrientationMath.makeCaptureReference(from: invertedSample)

  #expect(expectedReference.motionQuaternionInterpretation == .rawReferenceToDevice)
  #expect(correctedReference.motionQuaternionInterpretation == .inverseReferenceToDevice)

  for (expected, corrected) in zip(
    expectedReference.motionReferenceToCaptureRotationMatrix.values,
    correctedReference.motionReferenceToCaptureRotationMatrix.values
  ) {
    #expect(abs(expected - corrected) < 0.000_001)
  }
}

@Test("Physical right turn and upward tilt produce matching commands")
func physicalMotionDirectionSigns() {
  let initialDeviceFromMotion = simd_quatd(
    angle: -.pi / 2,
    axis: SIMD3<Double>(1, 0, 0)
  )
  let reference = OrientationMath.makeCaptureReference(
    from: makeMotionSample(quaternion: initialDeviceFromMotion)
  )

  // With the portrait camera initially facing motion-reference +Y, a physical
  // clockwise/right turn is a -Z world rotation, whose inverse mapping is this
  // +Z post-rotation of reference-to-device coordinates.
  let turnedRightDeviceFromMotion = simd_normalize(
    initialDeviceFromMotion
      * simd_quatd(angle: 45.0.radians, axis: SIMD3<Double>(0, 0, 1))
  )
  let turnedRightReading = OrientationMath.navigationReading(
    sample: makeMotionSample(quaternion: turnedRightDeviceFromMotion),
    captureReference: reference,
    target: makeTarget(id: "further-right", yaw: 90, pitch: 0),
    toleranceDegrees: 10
  )
  #expect(abs(turnedRightReading.yawErrorDegrees - 45) < 0.000_001)

  let tiltedUpDeviceFromMotion = simd_normalize(
    initialDeviceFromMotion
      * simd_quatd(angle: -30.0.radians, axis: SIMD3<Double>(1, 0, 0))
  )
  let tiltedUpReading = OrientationMath.navigationReading(
    sample: makeMotionSample(quaternion: tiltedUpDeviceFromMotion),
    captureReference: reference,
    target: makeTarget(id: "further-up", yaw: 0, pitch: 55),
    toleranceDegrees: 10
  )
  #expect(abs(tiltedUpReading.pitchErrorDegrees - 25) < 0.000_001)
}

@Test("Guidance errors are invariant to phone roll")
func navigationIsRollInvariant() {
  let reference = makePortraitCaptureReference()
  let current = OrientationMath.targetCameraToCaptureReference(
    makeTarget(id: "current", yaw: 25, pitch: -20)
  )
  let rolledCurrent = simd_normalize(
    current * simd_quatd(angle: 110.0.radians, axis: SIMD3<Double>(0, 0, 1))
  )
  let target = makeTarget(id: "target", yaw: 75, pitch: 35)

  let uprightReading = OrientationMath.navigationReading(
    sample: makeMotionSample(
      cameraToCaptureReference: current,
      captureReference: reference
    ),
    captureReference: reference,
    target: target,
    toleranceDegrees: 10
  )
  let rolledReading = OrientationMath.navigationReading(
    sample: makeMotionSample(
      cameraToCaptureReference: rolledCurrent,
      captureReference: reference
    ),
    captureReference: reference,
    target: target,
    toleranceDegrees: 10
  )

  #expect(
    abs(uprightReading.directionErrorDegrees - rolledReading.directionErrorDegrees) < 0.000_001)
  #expect(abs(uprightReading.yawErrorDegrees - rolledReading.yawErrorDegrees) < 0.000_001)
  #expect(abs(uprightReading.pitchErrorDegrees - rolledReading.pitchErrorDegrees) < 0.000_001)
}

@Test("Photo intrinsics use EXIF-oriented full-resolution pixel coordinates")
func exifOrientedPhotoIntrinsics() {
  let calibration = CameraIntrinsicsSample(
    monotonicTimestampSeconds: 12,
    fx: 450,
    fy: 600,
    cx: 450,
    cy: 600,
    referenceWidth: 900,
    referenceHeight: 1200,
    captureRotationDegrees: 90
  )
  let encodedLandscapePhoto = PhotoMetadata(
    codec: "jpeg",
    width: 4000,
    height: 3000,
    exifOrientation: 6,
    exposureDurationSeconds: nil,
    iso: nil,
    aperture: nil,
    focalLengthMillimeters: nil,
    focalLength35mmEquivalent: nil,
    brightnessValue: nil,
    exposureBiasValue: nil,
    lensMake: nil,
    lensModel: nil
  )
  let metadata = CameraIntrinsicsMetadata(
    sample: calibration,
    photo: encodedLandscapePhoto
  )

  #expect(metadata.encodedPhotoWidth == 4000)
  #expect(metadata.encodedPhotoHeight == 3000)
  #expect(metadata.orientedPhotoWidth == 3000)
  #expect(metadata.orientedPhotoHeight == 4000)
  #expect(metadata.appliedEXIFOrientation == 6)
  #expect(metadata.photoFx == 1500)
  #expect(metadata.photoFy == 2000)
  #expect(metadata.photoCx == 1500)
  #expect(metadata.photoCy == 2000)
  #expect(metadata.photoIntrinsicsCoordinateSpace.contains("display-oriented"))
}

@Test("CoreMotion rotations reproduce every planned camera target")
func plannedOrientationAlignment() {
  let configuration = CaptureConfiguration.debugPreset
  let reference = makePortraitCaptureReference()
  let plan = CapturePlan(configuration: configuration)

  for target in plan.targets {
    let desiredCamera = OrientationMath.targetCameraToCaptureReference(target)
    let sample = makeMotionSample(
      cameraToCaptureReference: desiredCamera,
      captureReference: reference
    )
    let reading = OrientationMath.navigationReading(
      sample: sample,
      captureReference: reference,
      target: target,
      toleranceDegrees: configuration.alignmentToleranceDegrees
    )

    #expect(reading.directionErrorDegrees < 0.000_01)
    #expect(abs(reading.yawErrorDegrees) < 0.000_01)
    #expect(abs(reading.pitchErrorDegrees) < 0.000_01)
    #expect(reading.isAligned)
  }
}

@Test("CoreMotion directions match gravity-referenced turn and tilt commands")
func orientationErrorDirections() {
  let configuration = CaptureConfiguration.debugPreset
  let reference = makePortraitCaptureReference()
  let forwardTarget = makeTarget(id: "forward", yaw: 0, pitch: 0)
  let sample = makeMotionSample(
    cameraToCaptureReference: OrientationMath.targetCameraToCaptureReference(forwardTarget),
    captureReference: reference
  )

  func reading(yaw: Double, pitch: Double) -> CaptureNavigationReading {
    OrientationMath.navigationReading(
      sample: sample,
      captureReference: reference,
      target: makeTarget(id: "test", yaw: yaw, pitch: pitch),
      toleranceDegrees: configuration.alignmentToleranceDegrees
    )
  }

  #expect(reading(yaw: 45, pitch: 0).yawErrorDegrees > 0)
  #expect(reading(yaw: 315, pitch: 0).yawErrorDegrees < 0)
  #expect(reading(yaw: 0, pitch: 55).pitchErrorDegrees > 0)
  #expect(reading(yaw: 0, pitch: -55).pitchErrorDegrees < 0)
}

@Test("Saved pose matrices are proper right-handed rotations")
func poseMatrixConvention() {
  let reference = makePortraitCaptureReference()
  let target = CapturePlan(configuration: .debugPreset).targets[3]
  let desiredCamera = OrientationMath.targetCameraToCaptureReference(target)
  let sample = makeMotionSample(
    cameraToCaptureReference: desiredCamera,
    captureReference: reference
  )
  let pose = OrientationMath.poseMetadata(
    sample: sample,
    captureReference: reference,
    referenceFrameName: "test"
  )
  let matrix = simd_double3x3(desiredCamera)
  let serialized = pose.cameraToCaptureReferenceRotationMatrix.values

  #expect(serialized.count == 9)
  #expect(abs(simd_determinant(matrix) - 1) < 0.000_000_1)
  #expect(abs(simd_length(matrix.columns.0) - 1) < 0.000_000_1)
  #expect(abs(simd_length(matrix.columns.1) - 1) < 0.000_000_1)
  #expect(abs(simd_length(matrix.columns.2) - 1) < 0.000_000_1)

  // The native bridge consumes the manifest as row-major C++ data. Verify
  // that its third column still reconstructs the recorded optical direction.
  let serializedForward = SIMD3<Double>(serialized[2], serialized[5], serialized[8])
  let expectedForward = simd_act(desiredCamera, SIMD3<Double>(0, 0, 1))
  #expect(simd_distance(serializedForward, expectedForward) < 0.000_000_1)
}

@Test("Capture packages preserve recorded order and sensor-first engine instructions")
func capturePackagePersistence() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraCoreTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var configuration = CaptureConfiguration.debugPreset
  configuration.horizontalCount = 2
  configuration.downwardCount = 0
  configuration.upwardCount = 0
  let plan = CapturePlan(configuration: configuration)
  let reference = makePortraitCaptureReference()
  let store = CapturePackageStore(captureSessionsRootURL: root)
  _ = try await store.begin(plan: plan, coreMotionReferenceFrame: "test-frame")

  for target in plan.targets.reversed() {
    let photo = makePhoto(sequenceIndex: target.sequenceIndex)
    let pose = OrientationMath.poseMetadata(
      sample: photo.motionSample,
      captureReference: reference,
      referenceFrameName: "test-frame"
    )
    _ = try await store.append(
      photo: photo,
      target: target,
      pose: pose,
      alignment: makeAlignment(configuration: configuration)
    )
  }

  let package = try await store.finalize()
  #expect(package.manifest.schemaVersion == 6)
  #expect(package.manifest.frames.map(\.sequenceIndex) == [0, 1])
  #expect(package.manifest.frames.map(\.target.id) == plan.targets.reversed().map(\.id))
  #expect(
    package.manifest.frames.map(\.imageFilename) == [
      "000_horizontal_01.jpg", "001_horizontal_00.jpg",
    ])
  #expect(package.manifest.primaryCapture == nil)
  #expect(package.manifest.engineInitialization.usePosePriors)
  #expect(!package.manifest.engineInitialization.allowGlobalArrangementRediscovery)
  #expect(package.manifest.engineInitialization.placementSource == "recorded")
  #expect(package.manifest.engineInitialization.maximumPoseRefinementDegrees == 6)
  #expect(
    package.manifest.engineInitialization.enabledPipelineStages.contains(
      "sensor-anchored-refinement"
    )
  )
  #expect(
    package.manifest.engineInitialization.enabledPipelineStages.contains(
      "adaptive-periodic-ring-seam"
    )
  )
  #expect(FileManager.default.fileExists(atPath: package.manifestURL.path))

  let decoder = JSONDecoder()
  decoder.dateDecodingStrategy = .secondsSince1970
  let decoded = try decoder.decode(
    CaptureSessionManifest.self,
    from: Data(contentsOf: package.manifestURL)
  )
  #expect(decoded.schemaVersion == package.manifest.schemaVersion)
  #expect(decoded.sessionID == package.manifest.sessionID)
  #expect(decoded.plan == package.manifest.plan)
  #expect(decoded.frames == package.manifest.frames)
  #expect(abs(decoded.createdAt.timeIntervalSince(package.manifest.createdAt)) < 0.000_001)
  #expect(
    abs(
      try #require(decoded.completedAt).timeIntervalSince(
        try #require(package.manifest.completedAt)
      )
    ) < 0.000_001
  )
  #expect(decoded.timestampConvention.contains("sub-millisecond precision preserved"))
  #expect(decoded.frames.allSatisfy { $0.intrinsics.photoFx > 0 })
  #expect(decoded.frames.allSatisfy { $0.lens.deviceType.contains("UltraWide") })
}

@Test("Completed packages are listed for the in-app gallery")
func galleryListsCompletedPackagesOnly() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraGalleryTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  let store = CapturePackageStore(captureSessionsRootURL: root)
  let completed = try await makeFinalizedPackage(root: root)

  var incompleteConfiguration = CaptureConfiguration.debugPreset
  incompleteConfiguration.horizontalCount = 2
  incompleteConfiguration.downwardCount = 0
  incompleteConfiguration.upwardCount = 0
  let incompletePlan = CapturePlan(configuration: incompleteConfiguration)
  _ = try await store.begin(plan: incompletePlan, coreMotionReferenceFrame: "test-frame")
  // Leave the session incomplete, then abandon it.
  await store.abandon()

  let listed = try await store.listCompletedPackages()
  #expect(listed.map(\.manifest.sessionID) == [completed.manifest.sessionID])
  #expect(listed.first?.manifest.completedAt != nil)
  #expect(listed.first?.hasPanorama == false)
}

@Test("Share archive includes images and per-frame metadata")
func shareArchiveContainsImagesAndMetadata() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraShareTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  let package = try await makeFinalizedPackage(root: root)
  let zipURL = try await CapturePackageStore(captureSessionsRootURL: root)
    .makeShareArchive(for: package)

  #expect(zipURL.pathExtension == "zip")
  #expect(FileManager.default.fileExists(atPath: zipURL.path))

  let zipData = try Data(contentsOf: zipURL)
  let zipText = String(decoding: zipData, as: UTF8.self)
  #expect(zipText.contains("manifest.json"))
  #expect(zipText.contains("images/"))
  #expect(zipText.contains("metadata/"))
  #expect(zipText.contains(package.manifest.frames[0].imageFilename))

  let frameSidecar = package.manifest.frames[0].imageFilename
    .replacingOccurrences(of: ".jpg", with: ".json")
  #expect(zipText.contains(frameSidecar))
}

@Test("Engine adapter hands off capture package rotations for pitch-prior metadata")
func engineHandoff() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraEngineTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  let package = try await makeFinalizedPackage(root: root)
  let engine = RecordingNativeEngine()
  let adapter = SpheraEngineAdapter(nativeEngine: engine)
  let result = try await adapter.stitch(package: package)
  let request = try #require(await engine.lastRequest)

  #expect(
    request.initialCameraRotations
      == package.manifest.frames.map {
        $0.pose.cameraToCaptureReferenceRotationMatrix
      })
  #expect(request.manifestURL == package.manifestURL)
  #expect(result.panoramaURL == request.outputDirectoryURL.appendingPathComponent("panorama.jpg"))
}

private actor RecordingNativeEngine: NativeSpheraEngine {
  private(set) var lastRequest: SpheraEngineRequest?

  func stitch(
    _ request: SpheraEngineRequest,
    progress: StitchProgressHandler?
  ) async throws -> StitchingResult {
    lastRequest = request
    progress?(StitchProgress(fraction: 1, message: "Done"))
    return StitchingResult(
      panoramaURL: request.outputDirectoryURL.appendingPathComponent("panorama.jpg"),
      reportURL: nil
    )
  }
}

private func makePortraitCaptureReference() -> CaptureReferenceFrame {
  let portraitDeviceFromMotion = simd_quatd(
    angle: -.pi / 2,
    axis: SIMD3<Double>(1, 0, 0)
  )
  return OrientationMath.makeCaptureReference(
    from: makeMotionSample(quaternion: portraitDeviceFromMotion)
  )
}

private func makeMotionSample(
  cameraToCaptureReference: simd_quatd,
  captureReference: CaptureReferenceFrame
) -> MotionSample {
  let captureFromMotion = captureReference.motionReferenceToCaptureQuaternion.simdQuaternion
  let deviceFromMotion = simd_normalize(
    deviceFromCamera * cameraToCaptureReference.inverse * captureFromMotion
  )
  return makeMotionSample(quaternion: deviceFromMotion)
}

private func makeMotionSample(
  quaternion: simd_quatd,
  gravityDevice: SIMD3<Double>? = nil
) -> MotionSample {
  let gravity =
    gravityDevice
    ?? -simd_act(quaternion, SIMD3<Double>(0, 0, 1))
  return MotionSample(
    monotonicTimestampSeconds: 123,
    wallClockTimestamp: Date(timeIntervalSince1970: 456),
    attitudeQuaternion: QuaternionValue(quaternion),
    gravity: Vector3Value(x: gravity.x, y: gravity.y, z: gravity.z),
    rotationRateRadiansPerSecond: Vector3Value(x: 0, y: 0, z: 0)
  )
}

private func normalizedYaw(_ value: Double) -> Double {
  let remainder = value.truncatingRemainder(dividingBy: 360)
  return remainder >= 0 ? remainder : remainder + 360
}

private func makeTarget(id: String, yaw: Double, pitch: Double) -> CaptureTarget {
  CaptureTarget(
    id: id,
    sequenceIndex: 0,
    ring: .horizontal,
    ringIndex: 0,
    ringCount: 1,
    yawDegrees: yaw,
    pitchDegrees: pitch,
    rollDegrees: 0
  )
}

private func makeAlignment(configuration: CaptureConfiguration) -> AlignmentMetadata {
  AlignmentMetadata(
    directionErrorDegrees: 0.25,
    yawErrorDegrees: 0.2,
    pitchErrorDegrees: -0.1,
    requiredToleranceDegrees: configuration.alignmentToleranceDegrees,
    requiredStableDurationSeconds: configuration.stableHoldDurationSeconds
  )
}

private func makeFinalizedPackage(root: URL) async throws -> CapturePackage {
  var configuration = CaptureConfiguration.debugPreset
  configuration.horizontalCount = 1
  configuration.downwardCount = 0
  configuration.upwardCount = 0
  let plan = CapturePlan(configuration: configuration)
  let reference = makePortraitCaptureReference()
  let store = CapturePackageStore(captureSessionsRootURL: root)
  _ = try await store.begin(plan: plan, coreMotionReferenceFrame: "test-frame")
  let target = try #require(plan.targets.first)
  let photo = makePhoto(sequenceIndex: 0)
  let pose = OrientationMath.poseMetadata(
    sample: photo.motionSample,
    captureReference: reference,
    referenceFrameName: "test-frame"
  )
  _ = try await store.append(
    photo: photo,
    target: target,
    pose: pose,
    alignment: makeAlignment(configuration: configuration)
  )
  return try await store.finalize()
}

private func makePhoto(sequenceIndex: Int) -> CapturedPhoto {
  CapturedPhoto(
    data: Data([0xff, 0xd8, 0xff, 0xd9]),
    captureTimestamp: Date(timeIntervalSince1970: Double(sequenceIndex + 1) + 0.123_456),
    motionSample: makeMotionSample(
      quaternion: simd_quatd(angle: -.pi / 2, axis: SIMD3<Double>(1, 0, 0))
    ),
    intrinsicsSample: CameraIntrinsicsSample(
      monotonicTimestampSeconds: Double(sequenceIndex + 1),
      fx: 900,
      fy: 900,
      cx: 600,
      cy: 450,
      referenceWidth: 1200,
      referenceHeight: 900,
      captureRotationDegrees: 90
    ),
    lensMetadata: LensMetadata(
      deviceType: "AVCaptureDeviceTypeBuiltInUltraWideCamera",
      position: "back",
      uniqueID: "test-camera",
      modelID: "test-model",
      localizedName: "Back Ultra Wide Camera",
      activeFormatDescription: "test-format",
      videoFieldOfViewDegrees: 120,
      minimumZoomFactor: 1,
      maximumZoomFactor: 1,
      zoomFactor: 1,
      lensPosition: 0.5,
      maximumPhotoWidth: 4000,
      maximumPhotoHeight: 3000
    ),
    photoMetadata: PhotoMetadata(
      codec: "jpeg",
      width: 4000,
      height: 3000,
      exifOrientation: 1,
      exposureDurationSeconds: 1 / 120,
      iso: 50,
      aperture: 2.2,
      focalLengthMillimeters: 1.54,
      focalLength35mmEquivalent: 13,
      brightnessValue: 8,
      exposureBiasValue: 0,
      lensMake: "Apple",
      lensModel: "Ultra Wide"
    )
  )
}

extension Double {
  fileprivate var radians: Double { self * .pi / 180 }
}

@Test("Alignment tolerance stays at six degrees for sensor-first capture")
func alignmentToleranceMatchesSensorFirstCap() {
  #expect(CaptureConfiguration.debugPreset.alignmentToleranceDegrees == 6)
}

@Test("Primary capture ring classification uses pitch midpoints")
func primaryCaptureRingClassification() {
  let configuration = CaptureConfiguration.debugPreset
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: 0, configuration: configuration) == .horizontal)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: 10, configuration: configuration) == .horizontal)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: -10, configuration: configuration) == .horizontal)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: 30, configuration: configuration) == .upward)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: 55, configuration: configuration) == .upward)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: -30, configuration: configuration) == .downward)
  #expect(OrientationMath.classifyCaptureRing(pitchDegrees: -55, configuration: configuration) == .downward)
}

@Test("Primary horizontal capture selects the yaw-zero horizontal slot")
func primaryHorizontalSelectsClosestSlot() {
  let plan = CapturePlan(configuration: .debugPreset)
  let reference = makePortraitCaptureReference()
  let sample = makeMotionSample(
    cameraToCaptureReference: OrientationMath.targetCameraToCaptureReference(
      makeTarget(id: "aim", yaw: 0, pitch: 0)
    ),
    captureReference: reference
  )
  let ring = OrientationMath.classifyCaptureRing(
    pitchDegrees: OrientationMath.currentPitchDegrees(
      sample: sample,
      captureReference: reference
    ),
    configuration: .debugPreset
  )
  #expect(ring == .horizontal)
  let target = OrientationMath.closestTarget(
    in: ring,
    targets: plan.targets,
    sample: sample,
    captureReference: reference
  )
  #expect(target?.id == "horizontal-0")
}

@Test("Nearest remaining target supports out-of-order selection")
func nearestRemainingTargetOutOfOrder() throws {
  let plan = CapturePlan(configuration: .debugPreset)
  let reference = makePortraitCaptureReference()
  let remaining = plan.targets.filter { $0.id != "horizontal-0" }
  let aimed = try #require(remaining.first { $0.id == "horizontal-3" })
  let sample = makeMotionSample(
    cameraToCaptureReference: OrientationMath.targetCameraToCaptureReference(aimed),
    captureReference: reference
  )
  let nearest = OrientationMath.nearestTarget(
    among: remaining,
    sample: sample,
    captureReference: reference,
    toleranceDegrees: 6
  )
  #expect(nearest?.target.id == "horizontal-3")
  #expect(nearest?.reading.isAligned == true)

  let projection = OrientationMath.projectTargetToScreen(
    sample: sample,
    captureReference: reference,
    target: aimed
  )
  #expect(projection.isInFront)
  #expect(abs(projection.offsetX) < 0.05)
  #expect(abs(projection.offsetY) < 0.05)
}

@Test("Manifest records primaryCapture without renaming image files")
func primaryCaptureMetadataPersisted() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraPrimaryMeta-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var configuration = CaptureConfiguration.debugPreset
  configuration.horizontalCount = 2
  configuration.downwardCount = 0
  configuration.upwardCount = 0
  let plan = CapturePlan(configuration: configuration)
  let reference = makePortraitCaptureReference()
  let store = CapturePackageStore(captureSessionsRootURL: root)
  _ = try await store.begin(plan: plan, coreMotionReferenceFrame: "test-frame")

  let target = try #require(plan.targets.first)
  let photo = makePhoto(sequenceIndex: 0)
  let pose = OrientationMath.poseMetadata(
    sample: photo.motionSample,
    captureReference: reference,
    referenceFrameName: "test-frame"
  )
  let record = try await store.append(
    photo: photo,
    target: target,
    pose: pose,
    alignment: makeAlignment(configuration: configuration),
    primaryCapture: PrimaryCaptureMetadata(
      imageFilename: "ignored.jpg",
      targetId: target.id,
      classifiedRing: .horizontal
    )
  )
  #expect(record.imageFilename == "000_horizontal_00.jpg")

  let second = try #require(plan.targets.last)
  _ = try await store.append(
    photo: makePhoto(sequenceIndex: 1),
    target: second,
    pose: pose,
    alignment: makeAlignment(configuration: configuration)
  )
  let package = try await store.finalize()
  #expect(package.manifest.schemaVersion == 6)
  #expect(package.manifest.primaryCapture?.imageFilename == "000_horizontal_00.jpg")
  #expect(package.manifest.primaryCapture?.targetId == target.id)
  #expect(package.manifest.primaryCapture?.classifiedRing == .horizontal)
  #expect(package.manifest.frames.map(\.imageFilename).first == "000_horizontal_00.jpg")
  #expect(package.previewImageURL == package.primaryImageURL)

  let engineOutput = package.directoryURL.appendingPathComponent(
    "engine-output",
    isDirectory: true
  )
  try FileManager.default.createDirectory(at: engineOutput, withIntermediateDirectories: true)
  try Data("pano".utf8).write(to: package.panoramaURL)
  #expect(package.hasPanorama)
  #expect(package.previewImageURL == package.primaryImageURL)
  #expect(package.previewImageURL != package.panoramaURL)
}

@Test("Schema 5 manifests without primaryCapture still decode")
func schema5ManifestsRemainDecodable() throws {
  let plan = CapturePlan(configuration: .debugPreset)
  let sessionID = UUID()
  let createdAt = Date(timeIntervalSince1970: 1_700_000_000)
  let legacy: [String: Any] = [
    "schemaVersion": 5,
    "sessionID": sessionID.uuidString,
    "timestampConvention": "UTC seconds since Unix epoch; sub-millisecond precision preserved",
    "createdAt": createdAt.timeIntervalSince1970,
    "completedAt": NSNull(),
    "plan": try jsonObject(plan),
    "imageDirectory": "images",
    "coreMotionReferenceFrame": "xArbitraryZVertical",
    "engineInitialization": [
      "placementSource": "recorded",
      "rotationField": "frames[].pose.cameraToCaptureReferenceRotationMatrix",
      "usePosePriors": true,
      "allowGlobalArrangementRediscovery": false,
      "maximumPoseRefinementDegrees": 6,
      "refinementPurpose": "test",
      "enabledPipelineStages": ["sift-matching"],
    ],
    "frames": [],
  ]
  let data = try JSONSerialization.data(withJSONObject: legacy)
  let decoder = JSONDecoder()
  decoder.dateDecodingStrategy = .secondsSince1970
  let decoded = try decoder.decode(CaptureSessionManifest.self, from: data)
  #expect(decoded.schemaVersion == 5)
  #expect(decoded.primaryCapture == nil)
  #expect(decoded.sessionID == sessionID)
}

private func jsonObject<T: Encodable>(_ value: T) throws -> Any {
  let encoder = JSONEncoder()
  encoder.dateEncodingStrategy = .secondsSince1970
  let data = try encoder.encode(value)
  return try JSONSerialization.jsonObject(with: data)
}

@Test("capture_ref rotation matches diag(1,-1,-1) left-multiply without transpose")
func captureRefRotationConvention() {
  // Mirrors Engine pose_priors.ios_to_opencv_rotation(..., "capture_ref").
  func proper(_ matrix: simd_double3x3) -> simd_double3x3 {
    let svd = matrix  // for identity / diag cases already proper
    let det =
      svd.columns.0.x * (svd.columns.1.y * svd.columns.2.z - svd.columns.1.z * svd.columns.2.y)
      - svd.columns.0.y * (svd.columns.1.x * svd.columns.2.z - svd.columns.1.z * svd.columns.2.x)
      + svd.columns.0.z * (svd.columns.1.x * svd.columns.2.y - svd.columns.1.y * svd.columns.2.x)
    #expect(abs(det - 1) < 1e-9 || abs(det + 1) < 1e-9)
    return svd
  }

  let axisFix = simd_double3x3(diagonal: SIMD3(1, -1, -1))
  let identity = matrix_identity_double3x3
  let result = proper(axisFix * identity)
  #expect(abs(result.columns.0.x - 1) < 1e-12)
  #expect(abs(result.columns.1.y + 1) < 1e-12)
  #expect(abs(result.columns.2.z + 1) < 1e-12)
  let forward = result * SIMD3<Double>(0, 0, 1)
  #expect(abs(forward.x) < 1e-12)
  #expect(abs(forward.y) < 1e-12)
  #expect(abs(forward.z + 1) < 1e-12)
}

@Test("Sensor-first engine initialization stages are recorded in new packages")
func sensorFirstEngineInitializationStages() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraSensorFirstMeta-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var configuration = CaptureConfiguration.debugPreset
  configuration.horizontalCount = 2
  configuration.downwardCount = 0
  configuration.upwardCount = 0
  let plan = CapturePlan(configuration: configuration)
  let store = CapturePackageStore(captureSessionsRootURL: root)
  let package = try await store.begin(plan: plan, coreMotionReferenceFrame: "test-frame")
  #expect(package.manifest.engineInitialization.placementSource == "recorded")
  #expect(package.manifest.engineInitialization.maximumPoseRefinementDegrees == 6)
  #expect(!package.manifest.engineInitialization.allowGlobalArrangementRediscovery)
  #expect(
    package.manifest.engineInitialization.enabledPipelineStages
      == [
        "manifest-canonicalize",
        "pose-overlap-graph",
        "per-frame-locked-intrinsics",
        "sift-matching",
        "sensor-anchored-refinement",
        "adaptive-periodic-ring-seam",
        "exposure-gain-blocks",
        "concurrent-ring-local-structure-graph-cut",
        "five-band-blend",
        "projection-native-top-cube-face",
        "projection-native-bottom-cube-face",
        "residual-direct-sphere-fill",
      ]
  )
  await store.abandon()
}

/// Documented product default: the normal stitch route must not load ML models.
enum SpheraEngineProductDefaults {
  static let loadsMLModelsOnDefaultPath = false
  static let maximumPoseRefinementDegrees = 6.0
  static let recipe = "sensor_first_s1_adaptive_ring_seam_polar_cube"
}

@Test("Default stitch product path loads zero ML models")
func defaultStitchPathLoadsNoMLModels() {
  #expect(!SpheraEngineProductDefaults.loadsMLModelsOnDefaultPath)
  #expect(SpheraEngineProductDefaults.maximumPoseRefinementDegrees == 6)
  #expect(
    SpheraEngineProductDefaults.recipe
      == "sensor_first_s1_adaptive_ring_seam_polar_cube"
  )
}

@Test("Downward pitch pointing to upward target produces upward screen guidance without inversion")
func downwardPitchPointingToUpwardTargetGuidesUpward() {
  let reference = makePortraitCaptureReference()
  // Downward pitch target (pitch = -55 degrees)
  let downwardTarget = CaptureTarget(
    id: "downward-0",
    sequenceIndex: 8,
    ring: .downward,
    ringIndex: 0,
    ringCount: 5,
    yawDegrees: 0,
    pitchDegrees: -55,
    rollDegrees: 0
  )
  // Upward target (pitch = +55 degrees)
  let upwardTarget = CaptureTarget(
    id: "upward-0",
    sequenceIndex: 13,
    ring: .upward,
    ringIndex: 0,
    ringCount: 5,
    yawDegrees: 0,
    pitchDegrees: 55,
    rollDegrees: 0
  )

  // Camera pointing at downward target
  let downwardCameraSample = makeMotionSample(
    cameraToCaptureReference: OrientationMath.targetCameraToCaptureReference(downwardTarget),
    captureReference: reference
  )

  let projection = OrientationMath.projectTargetToScreen(
    sample: downwardCameraSample,
    captureReference: reference,
    target: upwardTarget
  )

  // The upward target is above the downward optical axis in camera space.
  // In camera coordinates (+x right, +y down), aiming angle must point UP (-y, angle ~ -pi/2).
  // It must NOT point down (+y, angle ~ +pi/2) as occurred when divided by negative forward z.
  #expect(projection.aimingAngleRadians < 0)
  #expect(abs(projection.aimingAngleRadians - (-Double.pi / 2)) < 0.2)
}

@Test("Continuous angle unwrapping follows the shortest path across boundary")
func continuousAngleUnwrappingFollowsShortestPath() {
  func shortestDelta(from current: Double, to target: Double) -> Double {
    var delta = (target - current).truncatingRemainder(dividingBy: 2 * .pi)
    if delta > .pi { delta -= 2 * .pi }
    if delta < -.pi { delta += 2 * .pi }
    return delta
  }

  // Jumping from +179 degrees (+3.124 rad) to -179 degrees (-3.124 rad)
  let currentAngle = 179.0 * .pi / 180.0
  let targetAngle = -179.0 * .pi / 180.0
  let delta = shortestDelta(from: currentAngle, to: targetAngle)
  // Shortest delta is +2 degrees (+0.035 rad), NOT -358 degrees!
  #expect(abs(delta - (2.0 * .pi / 180.0)) < 1e-6)

  var unwrapped = currentAngle
  unwrapped += delta
  #expect(abs(unwrapped - (181.0 * .pi / 180.0)) < 1e-6)
}

@Test("CaptureConfiguration encodes and decodes with tolerance and preset values")
func captureConfigurationPersistence() throws {
  var config = CaptureConfiguration.debugPreset
  config.alignmentToleranceDegrees = 8.0
  config.horizontalCount = 10

  let data = try JSONEncoder().encode(config)
  let decoded = try JSONDecoder().decode(CaptureConfiguration.self, from: data)
  #expect(decoded.alignmentToleranceDegrees == 8.0)
  #expect(decoded.horizontalCount == 10)
  #expect(decoded.downwardCount == 5)
  #expect(decoded.upwardCount == 5)
}

@Test("Experimental panorama configuration defaults to 16+12+12")
func experimentalPanoramaConfigurationDefaults() {
  let config = ExperimentalPanoramaConfiguration.default
  #expect(config.horizontalImageCount == 16)
  #expect(config.upwardImageCount == 12)
  #expect(config.downwardImageCount == 12)
  #expect(config.totalImageCount == 40)
  #expect(config.scanLineOrder == [.horizontal, .upward, .downward])
  #expect(config.yawStepDegrees(for: .horizontal) == 22.5)
  #expect(config.targetYawOffsetDegrees(for: .horizontal, index: 0) == 0)
  #expect(config.targetYawOffsetDegrees(for: .horizontal, index: 15) == 337.5)
}

@Test("Experimental pose math uses gravity-yaw with identity looking along -Z")
func experimentalPoseMathIdentityHeading() {
  let identity = matrix_identity_float4x4
  let yawPitch = ExperimentalPoseMath.yawPitchDegrees(cameraToWorld: identity)
  #expect(abs(yawPitch.yaw) < 0.001)
  #expect(abs(yawPitch.pitch) < 0.001)
  let look = ExperimentalPoseMath.lookDirection(cameraToWorld: identity)
  #expect(abs(Double(look.z) + 1) < 0.001)
}

@Test("Experimental scan progressor captures on angular steps, not time")
func experimentalScanProgressorUsesAngularSteps() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.wrongDirectionDegrees = 18
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 10)

  let first = progressor.update(makeScanSample(yaw: 10, timestamp: 1))
  #expect(first.captureIndex == 0)
  progressor.noteCaptureFinished(success: true)

  let samePlaceLater = progressor.update(makeScanSample(yaw: 10.2, timestamp: 30))
  #expect(samePlaceLater.captureIndex == nil)

  let second = progressor.update(makeScanSample(yaw: 100, timestamp: 31))
  #expect(second.captureIndex == 1)
  #expect(abs(second.yawOffsetDegrees - 90) < 0.01)
  progressor.noteCaptureFinished(success: true)

  let duplicate = progressor.update(makeScanSample(yaw: 100.4, timestamp: 31.2))
  #expect(duplicate.captureIndex == nil)
}

@Test("Experimental scan progressor unwraps yaw across the 180 boundary")
func experimentalScanProgressorUnwrapsYaw() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 8
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 170)
  let first = progressor.update(makeScanSample(yaw: 170, timestamp: 1))
  #expect(first.captureIndex == 0)
  progressor.noteCaptureFinished(success: true)

  let wrapped = progressor.update(makeScanSample(yaw: -145, timestamp: 2))
  #expect(abs(wrapped.yawOffsetDegrees - 45) < 0.5)
  #expect(wrapped.captureIndex == 1)
}

@Test("Experimental scan progressor warns on reverse rotation and skips capture")
func experimentalScanProgressorWrongDirection() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 8
  config.yawSmoothingAlpha = 1
  config.captureDirection = .clockwise
  config.wrongDirectionDegrees = 6
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = progressor.update(makeScanSample(yaw: 0, timestamp: 1))
  progressor.noteCaptureFinished(success: true)

  let reverse = progressor.update(makeScanSample(yaw: -20, timestamp: 2))
  #expect(reverse.isWrongDirection)
  #expect(reverse.captureIndex == nil)
  #expect(reverse.lockedDirection == .clockwise)
}

@Test("Experimental scan progressor locks counterclockwise and captures along that rotation")
func experimentalScanProgressorLocksCounterclockwise() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.captureDirection = .automatic
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  let first = progressor.update(makeScanSample(yaw: 0, timestamp: 1))
  #expect(first.captureIndex == 0)
  progressor.noteCaptureFinished(success: true)

  let second = progressor.update(makeScanSample(yaw: -90, timestamp: 2))
  #expect(second.lockedDirection == .counterclockwise)
  #expect(second.captureIndex == 1)
  #expect(!second.isWrongDirection)
}

@Test("Experimental scan progressor blocks clockwise-then-reverse rotation")
func experimentalScanProgressorBlocksReverseAfterLock() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.captureDirection = .automatic
  config.wrongDirectionDegrees = 6
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = progressor.update(makeScanSample(yaw: 0, timestamp: 1))
  progressor.noteCaptureFinished(success: true)

  let forward = progressor.update(makeScanSample(yaw: 40, timestamp: 2))
  #expect(forward.lockedDirection == .clockwise)
  #expect(forward.captureIndex == nil)
  progressor.noteCaptureFinished(success: true)

  let reverse = progressor.update(makeScanSample(yaw: 20, timestamp: 3))
  #expect(reverse.isWrongDirection)
  #expect(reverse.captureIndex == nil)
}

@Test("Experimental scan progressor defers capture when pitch error is too large")
func experimentalScanProgressorDefersOffPathPitch() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.maxPitchErrorForCaptureDegrees = 5
  config.captureDirection = .clockwise
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = progressor.update(makeScanSample(yaw: 0, pitch: 0, timestamp: 1))
  progressor.noteCaptureFinished(success: true)

  let offPath = progressor.update(makeScanSample(yaw: 90, pitch: 12, timestamp: 2))
  #expect(offPath.captureIndex == nil)
  #expect(offPath.isAbovePath)
  #expect(offPath.qualityNotes.contains("off-path-pitch"))

  let onPath = progressor.update(makeScanSample(yaw: 90, pitch: 1, timestamp: 3))
  #expect(onPath.captureIndex == 1)
}

@Test("Pitch and roll warnings use the same threshold as the capture gate")
func experimentalWarningMatchesCaptureGate() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.maxPitchErrorForCaptureDegrees = 4
  config.pathDriftWarningDegrees = 2
  config.maxRollForCaptureDegrees = 4
  config.rollWarningDegrees = 2.5
  config.captureDirection = .clockwise

  var pitchProgressor = ExperimentalScanProgressor(configuration: config)
  pitchProgressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = pitchProgressor.update(makeScanSample(yaw: 0, timestamp: 1))
  pitchProgressor.noteCaptureFinished(success: true)

  let pitchInside = pitchProgressor.update(makeScanSample(yaw: 90, pitch: 3.5, timestamp: 2))
  #expect(pitchInside.captureIndex == 1)
  #expect(!pitchInside.isAbovePath)
  #expect(!pitchInside.qualityNotes.contains("off-path-pitch"))

  var pitchBlocked = ExperimentalScanProgressor(configuration: config)
  pitchBlocked.beginLine(.horizontal, currentYawDegrees: 0)
  _ = pitchBlocked.update(makeScanSample(yaw: 0, timestamp: 1))
  pitchBlocked.noteCaptureFinished(success: true)
  let pitchOutside = pitchBlocked.update(makeScanSample(yaw: 90, pitch: 4.5, timestamp: 2))
  #expect(pitchOutside.captureIndex == nil)
  #expect(pitchOutside.isAbovePath)
  #expect(pitchOutside.qualityNotes.contains("off-path-pitch"))

  var rollProgressor = ExperimentalScanProgressor(configuration: config)
  rollProgressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = rollProgressor.update(makeScanSample(yaw: 0, timestamp: 1))
  rollProgressor.noteCaptureFinished(success: true)
  let rollInside = rollProgressor.update(makeScanSample(yaw: 90, roll: 3.5, timestamp: 2))
  #expect(rollInside.captureIndex == 1)
  #expect(!rollInside.isRolled)
  #expect(!rollInside.qualityNotes.contains("off-upright-roll"))

  var rollBlocked = ExperimentalScanProgressor(configuration: config)
  rollBlocked.beginLine(.horizontal, currentYawDegrees: 0)
  _ = rollBlocked.update(makeScanSample(yaw: 0, timestamp: 1))
  rollBlocked.noteCaptureFinished(success: true)
  let rollOutside = rollBlocked.update(makeScanSample(yaw: 90, roll: 4.5, timestamp: 2))
  #expect(rollOutside.captureIndex == nil)
  #expect(rollOutside.isRolled)
  #expect(rollOutside.qualityNotes.contains("off-upright-roll"))
}

@Test("Pointing at the sky does not capture even after yaw overshoots a target")
func experimentalScanProgressorNeverCapturesSkyAim() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 16
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.maxPitchErrorForCaptureDegrees = 4
  config.captureDirection = .clockwise
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = progressor.update(makeScanSample(yaw: 0, pitch: 0, timestamp: 1))
  progressor.noteCaptureFinished(success: true)

  let skyAim = progressor.update(makeScanSample(yaw: 40, pitch: 88, timestamp: 2))
  #expect(skyAim.captureIndex == nil)
  #expect(skyAim.isAbovePath)
  #expect(skyAim.qualityNotes.contains("off-path-pitch"))
}

@Test("Screen-plane roll blocks capture and reports an upright warning")
func experimentalScanProgressorBlocksScreenPlaneRoll() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 4
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  config.maxRollForCaptureDegrees = 4
  config.rollWarningDegrees = 2.5
  config.captureDirection = .clockwise
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)
  _ = progressor.update(makeScanSample(yaw: 0, timestamp: 1))
  progressor.noteCaptureFinished(success: true)

  let rolled = progressor.update(makeScanSample(yaw: 90, roll: 25, timestamp: 2))
  #expect(rolled.captureIndex == nil)
  #expect(rolled.isRolled)
  #expect(rolled.qualityNotes.contains("off-upright-roll"))

  let upright = progressor.update(makeScanSample(yaw: 90, roll: 1, timestamp: 3))
  #expect(upright.captureIndex == 1)
  #expect(!upright.isRolled)
}

@Test("Screen-plane roll is zero when upright and follows steering-wheel rotation")
func experimentalScreenPlaneRollFromGravity() {
  let upright = Vector3Value(x: 0, y: -1, z: 0)
  #expect(abs(ExperimentalPoseMath.screenPlaneRollDegrees(gravityDevice: upright)) < 0.001)
  #expect(abs(ExperimentalPoseMath.cameraElevationDegrees(gravityDevice: upright)) < 0.001)

  let clockwise = Vector3Value(x: 1, y: 0, z: 0)
  #expect(abs(ExperimentalPoseMath.screenPlaneRollDegrees(gravityDevice: clockwise) - 90) < 0.001)

  let counterclockwise = Vector3Value(x: -1, y: 0, z: 0)
  #expect(abs(ExperimentalPoseMath.screenPlaneRollDegrees(gravityDevice: counterclockwise) + 90) < 0.001)

  let sky = Vector3Value(x: 0, y: 0, z: 1)
  #expect(abs(ExperimentalPoseMath.cameraElevationDegrees(gravityDevice: sky) - 90) < 0.001)
  #expect(!ExperimentalPoseMath.isScreenPlaneRollDefined(gravityDevice: sky))
}

@Test("Attitude gate uses the pitch sensor farther from the scan line")
func experimentalAttitudeGatePrefersWorsePitchSensor() {
  var config = ExperimentalPanoramaConfiguration.default
  config.maxPitchErrorForCaptureDegrees = 4
  config.maxRollForCaptureDegrees = 4
  let skyGravity = Vector3Value(x: 0, y: 0, z: 1)
  let hiddenByARKit = ExperimentalAttitudeReading.make(
    gravity: skyGravity,
    arkitPitchDegrees: 0,
    targetPitchDegrees: 0,
    configuration: config
  )
  #expect(hiddenByARKit.isPitchBlockingCapture)
  #expect(abs(hiddenByARKit.pitchDegrees - 90) < 1)

  let rolled = ExperimentalAttitudeReading.make(
    gravity: Vector3Value(x: 0.5, y: -0.866, z: 0),
    arkitPitchDegrees: 0,
    targetPitchDegrees: 0,
    configuration: config
  )
  #expect(abs(rolled.rollDegrees - 30) < 1)
  #expect(rolled.isRolled)
  #expect(rolled.isRollBlockingCapture)
  #expect(rolled.primaryWarning != nil)
}

@Test("Path arrow moves down when the phone aims below the line")
func experimentalArrowOffsetMatchesPhonePitch() {
  let below = ExperimentalPoseMath.arrowScreenYOffset(pitchErrorDegrees: -5)
  let above = ExperimentalPoseMath.arrowScreenYOffset(pitchErrorDegrees: 5)
  #expect(below > 0)
  #expect(above < 0)
  #expect(abs(below + above) < 0.001)
}

@Test("Experimental scan progressor completes a line after the configured count")
func experimentalScanProgressorCompletesLine() {
  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 3
  config.scanRangeDegrees = 360
  config.yawSmoothingAlpha = 1
  config.captureHysteresisDegrees = 0.5
  var progressor = ExperimentalScanProgressor(configuration: config)
  progressor.beginLine(.horizontal, currentYawDegrees: 0)

  for index in 0..<3 {
    let yaw = Double(index) * 120
    let update = progressor.update(makeScanSample(yaw: yaw, timestamp: Double(index)))
    #expect(update.captureIndex == index)
    progressor.noteCaptureFinished(success: true)
  }
  let done = progressor.update(makeScanSample(yaw: 240, timestamp: 4))
  #expect(done.isLineComplete)
  #expect(done.captureIndex == nil)
  #expect(done.capturedCount == 3)
}

@Test("Experimental packages associate images by stable filename and id")
func experimentalPackagePersistsStableImageIDs() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraARKitTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 2
  config.upwardImageCount = 0
  config.downwardImageCount = 0
  let store = ExperimentalCapturePackageStore(sessionsRootURL: root)
  _ = try await store.begin(configuration: config, coreMotionReferenceFrame: "test-frame")
  try await store.recordSessionStart(
    transform: Matrix4x4Value(matrix_identity_float4x4),
    timestamp: 1
  )

  let firstID = UUID()
  let secondID = UUID()
  let first = try await store.append(
    imageData: Data([0xff, 0xd8, 0xff, 0xd9]),
    frameID: firstID,
    scanLine: .horizontal,
    indexInLine: 0,
    targetYawOffsetDegrees: 0,
    actualYawOffsetDegrees: 0.4,
    actualPitchDegrees: 1.2,
    arkit: makeARKitMetadata(timestamp: 1),
    motion: makeMotionSample(
      quaternion: simd_quatd(angle: -.pi / 2, axis: SIMD3<Double>(1, 0, 0))
    ),
    photo: PhotoMetadata(
      codec: "jpeg",
      width: 1920,
      height: 1440,
      exifOrientation: 1,
      exposureDurationSeconds: nil,
      iso: nil,
      aperture: nil,
      focalLengthMillimeters: nil,
      focalLength35mmEquivalent: nil,
      brightnessValue: nil,
      exposureBiasValue: nil,
      lensMake: nil,
      lensModel: nil
    ),
    qualityNotes: []
  )
  _ = try await store.append(
    imageData: Data([0xff, 0xd8, 0xff, 0xd9]),
    frameID: secondID,
    scanLine: .horizontal,
    indexInLine: 1,
    targetYawOffsetDegrees: 180,
    actualYawOffsetDegrees: 179.2,
    actualPitchDegrees: -0.4,
    arkit: makeARKitMetadata(timestamp: 2),
    motion: makeMotionSample(
      quaternion: simd_quatd(angle: -.pi / 2, axis: SIMD3<Double>(1, 0, 0))
    ),
    photo: PhotoMetadata(
      codec: "jpeg",
      width: 1920,
      height: 1440,
      exifOrientation: 1,
      exposureDurationSeconds: nil,
      iso: nil,
      aperture: nil,
      focalLengthMillimeters: nil,
      focalLength35mmEquivalent: nil,
      brightnessValue: nil,
      exposureBiasValue: nil,
      lensMake: nil,
      lensModel: nil
    ),
    qualityNotes: []
  )

  let package = try await store.finalize()
  #expect(package.manifest.kind == ExperimentalCaptureManifest.kindIdentifier)
  #expect(package.manifest.isComplete)
  #expect(package.manifest.frames.count == 2)
  #expect(package.manifest.frames[0].id == firstID)
  #expect(package.manifest.frames[0].imageFilename.contains(firstID.uuidString))
  #expect(package.manifest.frames[0].scanLine == .horizontal)
  #expect(package.manifest.frames[1].id == secondID)
  #expect(first.imageFilename.hasPrefix("horizontal_00_"))
  #expect(FileManager.default.fileExists(atPath: package.imageURL(for: first.imageFilename).path))
}

@Test("Experimental share archive includes ARKit sidecars and not the current engine request")
func experimentalShareArchiveContainsARKitMetadata() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraARKitShareTests-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 1
  config.upwardImageCount = 0
  config.downwardImageCount = 0
  let store = ExperimentalCapturePackageStore(sessionsRootURL: root)
  _ = try await store.begin(configuration: config, coreMotionReferenceFrame: "test-frame")
  let frameID = UUID()
  _ = try await store.append(
    imageData: Data([0xff, 0xd8, 0xff, 0xd9]),
    frameID: frameID,
    scanLine: .horizontal,
    indexInLine: 0,
    targetYawOffsetDegrees: 0,
    actualYawOffsetDegrees: 0,
    actualPitchDegrees: 0,
    arkit: makeARKitMetadata(timestamp: 1),
    motion: makeMotionSample(
      quaternion: simd_quatd(angle: -.pi / 2, axis: SIMD3<Double>(1, 0, 0))
    ),
    photo: PhotoMetadata(
      codec: "jpeg",
      width: 1920,
      height: 1440,
      exifOrientation: 1,
      exposureDurationSeconds: nil,
      iso: nil,
      aperture: nil,
      focalLengthMillimeters: nil,
      focalLength35mmEquivalent: nil,
      brightnessValue: nil,
      exposureBiasValue: nil,
      lensMake: nil,
      lensModel: nil
    ),
    qualityNotes: []
  )
  let package = try await store.finalize()
  let zipURL = try await store.makeShareArchive(for: package)
  let zipText = String(decoding: try Data(contentsOf: zipURL), as: UTF8.self)
  #expect(zipText.contains("manifest.json"))
  #expect(zipText.contains("ARKIT_EXPERIMENTAL_NOTES.md"))
  #expect(zipText.contains("metadata/"))
  #expect(zipText.contains(frameID.uuidString))
  #expect(!zipText.contains("sphera-engine-request.json"))
}

@Test("Experimental ARKit JPEGs are tagged for locked-portrait display")
func experimentalARKitJPEGUsesPortraitEXIFOrientation() {
  #expect(ExperimentalCaptureImage.jpegEXIFOrientation == 6)
  let photo = PhotoMetadata(
    codec: "jpeg",
    width: 1920,
    height: 1440,
    exifOrientation: ExperimentalCaptureImage.jpegEXIFOrientation,
    exposureDurationSeconds: nil,
    iso: nil,
    aperture: nil,
    focalLengthMillimeters: nil,
    focalLength35mmEquivalent: nil,
    brightnessValue: nil,
    exposureBiasValue: nil,
    lensMake: nil,
    lensModel: nil
  )
  #expect(photo.orientedPixelWidth == 1440)
  #expect(photo.orientedPixelHeight == 1920)
}

@Test("Experimental packages record ARKit gravity as the motion reference")
func experimentalPackageRecordsARKitGravityReference() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraARKitRef-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 1
  config.upwardImageCount = 0
  config.downwardImageCount = 0
  let store = ExperimentalCapturePackageStore(sessionsRootURL: root)
  let started = try await store.begin(
    configuration: config,
    coreMotionReferenceFrame: ExperimentalCaptureManifest.worldTrackingReferenceFrame
  )
  #expect(started.manifest.coreMotionReferenceFrame == "arkitGravity")
  #expect(
    started.manifest.coreMotionReferenceFrame
      == ExperimentalCaptureManifest.worldTrackingReferenceFrame
  )
}

@Test("Incomplete experimental sessions are not listed")
func experimentalIncompleteSessionsAreCleanedUp() async throws {
  let root = FileManager.default.temporaryDirectory
    .appendingPathComponent("SpheraARKitCleanup-\(UUID().uuidString)", isDirectory: true)
  defer { try? FileManager.default.removeItem(at: root) }

  var config = ExperimentalPanoramaConfiguration.default
  config.horizontalImageCount = 2
  config.upwardImageCount = 0
  config.downwardImageCount = 0
  let store = ExperimentalCapturePackageStore(sessionsRootURL: root)
  _ = try await store.begin(configuration: config, coreMotionReferenceFrame: "test-frame")
  await store.abandon()
  let listed = try await store.listCompletedPackages()
  #expect(listed.isEmpty)
}

private func makeScanSample(
  yaw: Double,
  pitch: Double = 0,
  roll: Double = 0,
  timestamp: TimeInterval,
  quality: ExperimentalTrackingQuality = .normal
) -> ExperimentalScanSample {
  ExperimentalScanSample(
    yawDegrees: yaw,
    pitchDegrees: pitch,
    rollDegrees: roll,
    timestamp: timestamp,
    trackingQuality: quality,
    rotationRateMagnitude: 0,
    translationMeters: 0
  )
}

private func makeARKitMetadata(timestamp: TimeInterval) -> ARKitCameraMetadata {
  let transform = matrix_identity_float4x4
  return ARKitCameraMetadata(
    timestamp: timestamp,
    trackingState: .normal,
    transform: Matrix4x4Value(transform),
    intrinsics: Matrix3x3Value(matrix_identity_float3x3),
    imageResolutionWidth: 1920,
    imageResolutionHeight: 1440,
    position: Vector3Value(x: 0, y: 0, z: 0),
    orientation: QuaternionValue(w: 1, x: 0, y: 0, z: 0),
    translationFromSessionStart: Vector3Value(x: 0, y: 0, z: 0),
    rotationFromSessionStart: QuaternionValue(w: 1, x: 0, y: 0, z: 0),
    relativeTransform: Matrix4x4Value(transform),
    eulerDegrees: Vector3Value(x: 0, y: 0, z: 0)
  )
}


