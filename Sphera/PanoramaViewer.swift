import SceneKit
import SwiftUI
import UIKit

/// Touch-driven equirectangular panorama viewer. It intentionally uses no
/// motion input, so viewing remains independent from capture orientation.
struct PanoramaViewer: UIViewRepresentable {
  let imageURL: URL

  func makeCoordinator() -> Coordinator {
    Coordinator()
  }

  func makeUIView(context: Context) -> SCNView {
    let view = SCNView(frame: .zero)
    view.backgroundColor = .black
    view.scene = SCNScene()
    view.isPlaying = false
    view.rendersContinuously = false
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
    context.coordinator.applyCameraOrientation()
    return view
  }

  func updateUIView(_ view: SCNView, context: Context) {
    guard context.coordinator.loadedURL != imageURL else { return }
    context.coordinator.installPanorama(from: imageURL, in: view)
  }

  final class Coordinator: NSObject {
    fileprivate weak var cameraNode: SCNNode?
    fileprivate var loadedURL: URL?
    fileprivate var fieldOfView: CGFloat = 72

    private var yaw: Float = 0
    private var pitch: Float = 0
    private var gestureStartYaw: Float = 0
    private var gestureStartPitch: Float = 0
    private var gestureStartFieldOfView: CGFloat = 72
    private weak var panoramaNode: SCNNode?

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
      // Back-face viewing mirrors a texture. Mirror S once more so turning
      // right in the captured scene remains turning right in the viewer.
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

    @objc fileprivate func pan(_ recognizer: UIPanGestureRecognizer) {
      guard let view = recognizer.view else { return }
      switch recognizer.state {
      case .began:
        gestureStartYaw = yaw
        gestureStartPitch = pitch
      case .changed:
        let translation = recognizer.translation(in: view)
        let width = max(view.bounds.width, 1)
        let height = max(view.bounds.height, 1)
        yaw = gestureStartYaw - Float(translation.x / width) * .pi
        pitch = min(
          .pi / 2 - 0.02,
          max(-.pi / 2 + 0.02, gestureStartPitch - Float(translation.y / height) * .pi / 2)
        )
        applyCameraOrientation()
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

    fileprivate func applyCameraOrientation() {
      // SceneKit cameras look down -Z. A pi yaw starts the viewer at the
      // capture session's zero-heading, which the engine places at +Z.
      cameraNode?.eulerAngles = SCNVector3(pitch, yaw + .pi, 0)
    }
  }
}
