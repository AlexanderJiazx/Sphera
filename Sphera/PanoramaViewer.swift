import CoreMotion
import SceneKit
import simd
import SwiftUI
import UIKit

/// Equirectangular panorama viewer driven by touch and device rotation.
///
/// Device motion must NOT use CMAttitude.roll/pitch/yaw. Those Euler angles are
/// a pose-dependent Tait–Bryan decomposition: after any tilt, "horizontal" turn
/// mixes into pitch (the bug we hit). Spherical viewers map the full attitude
/// quaternion into SceneKit instead — same approach as CTPanoramaView.
struct PanoramaViewer: UIViewRepresentable {
  let imageURL: URL

  func makeCoordinator() -> Coordinator {
    Coordinator()
  }

  func makeUIView(context: Context) -> SCNView {
    let view = SCNView(frame: .zero)
    view.backgroundColor = .black
    view.scene = SCNScene()
    view.isPlaying = true
    view.rendersContinuously = true
    view.antialiasingMode = .multisampling4X

    let camera = SCNCamera()
    camera.fieldOfView = context.coordinator.fieldOfView
    camera.zNear = 0.01
    camera.zFar = 100

    let cameraNode = SCNNode()
    cameraNode.camera = camera
    view.scene?.rootNode.addChildNode(cameraNode)
    view.pointOfView = cameraNode
    context.coordinator.cameraNode = cameraNode
    context.coordinator.sceneView = view

    let pan = UIPanGestureRecognizer(
      target: context.coordinator,
      action: #selector(Coordinator.pan(_:))
    )
    pan.minimumNumberOfTouches = 1
    pan.maximumNumberOfTouches = 2
    pan.delegate = context.coordinator
    view.addGestureRecognizer(pan)

    let pinch = UIPinchGestureRecognizer(
      target: context.coordinator,
      action: #selector(Coordinator.pinch(_:))
    )
    pinch.delegate = context.coordinator
    view.addGestureRecognizer(pinch)

    let rotation = UIRotationGestureRecognizer(
      target: context.coordinator,
      action: #selector(Coordinator.rotate(_:))
    )
    rotation.delegate = context.coordinator
    view.addGestureRecognizer(rotation)

    context.coordinator.installPanorama(from: imageURL, in: view)
    context.coordinator.startDeviceMotion()
    context.coordinator.applyCameraOrientation(motionQuaternion: nil)
    return view
  }

  func updateUIView(_ view: SCNView, context: Context) {
    guard context.coordinator.loadedURL != imageURL else { return }
    context.coordinator.installPanorama(from: imageURL, in: view)
  }

  static func dismantleUIView(_ uiView: SCNView, coordinator: Coordinator) {
    coordinator.stopDeviceMotion()
  }

  final class Coordinator: NSObject, UIGestureRecognizerDelegate {
    fileprivate weak var cameraNode: SCNNode?
    fileprivate weak var sceneView: SCNView?
    fileprivate var loadedURL: URL?
    fileprivate var fieldOfView: CGFloat = 72

    private let motionManager = CMMotionManager()
    /// Latest absolute CM attitude quaternion (device → reference frame).
    private var latestMotionQuaternion: simd_quatf?
    private var sphereOrientation: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private weak var panoramaNode: SCNNode?

    /// CoreMotion reference is Z-up; SceneKit is Y-up with the camera looking
    /// down −Z. Portrait apps remap with a −π/2 rotation about X
    /// (CTPanoramaView / common SceneKit+CoreMotion spherical viewers).
    private let coreMotionToSceneKit = simd_quatf(
      angle: -.pi / 2,
      axis: SIMD3<Float>(1, 0, 0)
    )

    /// Historical viewer offset: SceneKit default look is −Z, while the stitch
    /// engine places capture heading 0 at +Z.
    private let captureHeadingAlign = simd_quatf(
      angle: .pi,
      axis: SIMD3<Float>(0, 1, 0)
    )

    fileprivate func installPanorama(from url: URL, in view: SCNView) {
      guard let image = UIImage(contentsOfFile: url.path) else { return }

      panoramaNode?.removeFromParentNode()
      let sphere = SCNSphere(radius: 10)
      sphere.segmentCount = 192
      let material = SCNMaterial()
      material.lightingModel = .constant
      material.cullMode = .front
      material.isDoubleSided = false
      material.diffuse.contents = image
      // Inside-sphere viewing mirrors S; flip it back so right stays right.
      material.diffuse.contentsTransform = SCNMatrix4MakeScale(-1, 1, 1)
      material.diffuse.wrapS = .repeat
      material.diffuse.wrapT = .clamp
      sphere.materials = [material]

      let node = SCNNode(geometry: sphere)
      node.simdOrientation = sphereOrientation
      view.scene?.rootNode.addChildNode(node)
      panoramaNode = node
      loadedURL = url
      view.setNeedsDisplay()
    }

    fileprivate func startDeviceMotion() {
      guard motionManager.isDeviceMotionAvailable else { return }
      guard !motionManager.isDeviceMotionActive else { return }

      latestMotionQuaternion = nil
      motionManager.deviceMotionUpdateInterval = 1.0 / 60.0
      motionManager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: .main) {
        [weak self] motion, _ in
        guard let self, let motion else { return }
        let q = motion.attitude.quaternion
        self.latestMotionQuaternion = simd_quatf(
          ix: Float(q.x),
          iy: Float(q.y),
          iz: Float(q.z),
          r: Float(q.w)
        )
        self.applyCameraOrientation(motionQuaternion: self.latestMotionQuaternion)
      }
    }

    fileprivate func stopDeviceMotion() {
      motionManager.stopDeviceMotionUpdates()
      latestMotionQuaternion = nil
    }

    @objc fileprivate func pan(_ recognizer: UIPanGestureRecognizer) {
      guard let view = recognizer.view else { return }
      if recognizer.state == .changed {
        let translation = recognizer.translation(in: view)
        recognizer.setTranslation(.zero, in: view)
        let width = max(view.bounds.width, 1)
        let height = max(view.bounds.height, 1)
        // Grab-the-sphere drag (reversed from look-drag).
        let deltaYaw = -Float(translation.x / width) * .pi
        let deltaPitch = -Float(translation.y / height) * (.pi / 2)
        applyTouchDelta(yaw: deltaYaw, pitch: deltaPitch, roll: 0)
        (view as? SCNView)?.setNeedsDisplay()
      }
    }

    @objc fileprivate func rotate(_ recognizer: UIRotationGestureRecognizer) {
      guard let view = recognizer.view else { return }
      if recognizer.state == .changed {
        let deltaRotation = Float(recognizer.rotation)
        recognizer.rotation = 0
        applyTouchDelta(yaw: 0, pitch: 0, roll: -deltaRotation)
        (view as? SCNView)?.setNeedsDisplay()
      }
    }

    @objc fileprivate func pinch(_ recognizer: UIPinchGestureRecognizer) {
      if recognizer.state == .changed {
        let scale = recognizer.scale
        recognizer.scale = 1.0
        fieldOfView = min(100, max(38, fieldOfView / scale))
        cameraNode?.camera?.fieldOfView = fieldOfView
        (recognizer.view as? SCNView)?.setNeedsDisplay()
      }
    }

    func gestureRecognizer(
      _ gestureRecognizer: UIGestureRecognizer,
      shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer
    ) -> Bool {
      guard let view = sceneView else { return false }
      return otherGestureRecognizer.view == view
    }

    private func applyTouchDelta(yaw: Float, pitch: Float, roll: Float) {
      guard let cameraNode else { return }
      let qYaw = simd_quatf(angle: yaw, axis: SIMD3<Float>(0, 1, 0))
      let qPitch = simd_quatf(angle: pitch, axis: SIMD3<Float>(1, 0, 0))
      let qRoll = simd_quatf(angle: roll, axis: SIMD3<Float>(0, 0, 1))
      let qDeltaCam = qYaw * qPitch * qRoll

      let camOrient = cameraNode.simdOrientation
      let qDeltaWorld = camOrient * qDeltaCam * camOrient.inverse

      sphereOrientation = simd_normalize(qDeltaWorld * sphereOrientation)

      SCNTransaction.begin()
      SCNTransaction.disableActions = true
      panoramaNode?.simdOrientation = sphereOrientation
      SCNTransaction.commit()
    }

    /// Compose SceneKit camera orientation purely from the full CM quaternion.
    fileprivate func applyCameraOrientation(motionQuaternion: simd_quatf?) {
      guard let cameraNode else { return }

      var orientation = motionQuaternion ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
      orientation = coreMotionToSceneKit * orientation
      orientation = captureHeadingAlign * orientation

      SCNTransaction.begin()
      SCNTransaction.disableActions = true
      cameraNode.simdOrientation = orientation
      SCNTransaction.commit()
    }
  }
}
