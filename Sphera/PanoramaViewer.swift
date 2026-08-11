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
    pan.maximumNumberOfTouches = 1
    view.addGestureRecognizer(pan)

    let pinch = UIPinchGestureRecognizer(
      target: context.coordinator,
      action: #selector(Coordinator.pinch(_:))
    )
    view.addGestureRecognizer(pinch)

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

  final class Coordinator: NSObject {
    fileprivate weak var cameraNode: SCNNode?
    fileprivate weak var sceneView: SCNView?
    fileprivate var loadedURL: URL?
    fileprivate var fieldOfView: CGFloat = 72

    private let motionManager = CMMotionManager()
    /// Latest absolute CM attitude quaternion (device → reference frame).
    private var latestMotionQuaternion: simd_quatf?
    private var touchYawOffset: Float = 0
    private var touchPitchOffset: Float = 0
    private var gestureStartYawOffset: Float = 0
    private var gestureStartPitchOffset: Float = 0
    private var gestureStartFieldOfView: CGFloat = 72
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
      view.scene?.rootNode.addChildNode(node)
      panoramaNode = node
      loadedURL = url
      view.setNeedsDisplay()
    }

    fileprivate func startDeviceMotion() {
      guard motionManager.isDeviceMotionAvailable else { return }
      guard !motionManager.isDeviceMotionActive else { return }

      latestMotionQuaternion = nil
      // xArbitraryZVertical: Z = gravity up, X locked when updates start.
      // Starting updates when the viewer opens makes "forward" match the
      // phone pose at open without multiplyByInverseOf + Euler hacks.
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
      switch recognizer.state {
      case .began:
        gestureStartYawOffset = touchYawOffset
        gestureStartPitchOffset = touchPitchOffset
      case .changed:
        let translation = recognizer.translation(in: view)
        let width = max(view.bounds.width, 1)
        let height = max(view.bounds.height, 1)
        // Grab-the-sphere drag (reversed from look-drag).
        touchYawOffset =
          gestureStartYawOffset + Float(translation.x / width) * .pi
        touchPitchOffset =
          gestureStartPitchOffset + Float(translation.y / height) * (.pi / 2)
        applyCameraOrientation(motionQuaternion: latestMotionQuaternion)
        (view as? SCNView)?.setNeedsDisplay()
      default:
        break
      }
    }

    @objc fileprivate func pinch(_ recognizer: UIPinchGestureRecognizer) {
      switch recognizer.state {
      case .began:
        gestureStartFieldOfView = fieldOfView
      case .changed:
        fieldOfView = min(100, max(38, gestureStartFieldOfView / recognizer.scale))
        cameraNode?.camera?.fieldOfView = fieldOfView
        (recognizer.view as? SCNView)?.setNeedsDisplay()
      default:
        break
      }
    }

    /// Compose SceneKit camera orientation from the full CM quaternion.
    ///
    /// Order matches CTPanoramaView spherical + both-controls mode:
    /// `yaw_world * (Rx(-π/2) * q_cm) * pitch_local`, plus capture heading align.
    fileprivate func applyCameraOrientation(motionQuaternion: simd_quatf?) {
      guard let cameraNode else { return }

      var orientation = motionQuaternion ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
      orientation = coreMotionToSceneKit * orientation
      orientation = captureHeadingAlign * orientation

      let yawTouch = simd_quatf(
        angle: touchYawOffset,
        axis: SIMD3<Float>(0, 1, 0)
      )
      let pitchTouch = simd_quatf(
        angle: touchPitchOffset,
        axis: SIMD3<Float>(1, 0, 0)
      )
      // World-Y yaw on the left, camera-local-X pitch on the right.
      orientation = yawTouch * orientation * pitchTouch

      SCNTransaction.begin()
      SCNTransaction.disableActions = true
      cameraNode.simdOrientation = orientation
      SCNTransaction.commit()
    }
  }
}
