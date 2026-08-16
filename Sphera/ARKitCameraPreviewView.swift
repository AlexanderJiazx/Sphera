import CoreVideo
import SwiftUI
import UIKit

struct ARKitCameraPreviewView: UIViewRepresentable {
  let service: ARKitTrackingService
  var isSourceReady: Bool = false

  func makeUIView(context: Context) -> ARKitPreviewContainerView {
    let view = ARKitPreviewContainerView(frame: Self.initialPreviewFrame)
    service.attachPreview(view)
    view.setSourceReady(isSourceReady, animated: false)
    return view
  }

  func updateUIView(_ uiView: ARKitPreviewContainerView, context: Context) {
    if service.previewView !== uiView {
      service.attachPreview(uiView)
    }
    uiView.setSourceReady(isSourceReady, animated: true)
  }

  static func dismantleUIView(_ uiView: ARKitPreviewContainerView, coordinator: ()) {
    uiView.detach()
  }

  private static var initialPreviewFrame: CGRect {
    let scenes = UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }
    if let bounds = scenes.first(where: { $0.activationState == .foregroundActive })?.coordinateSpace.bounds
      ?? scenes.first?.coordinateSpace.bounds
    {
      return bounds
    }
    return UIScreen.main.bounds
  }
}

final class ARKitPreviewContainerView: UIView {
  weak var trackingService: ARKitTrackingService?

  private let imageLayer = CALayer()
  private let blurEffectView = UIVisualEffectView(effect: UIBlurEffect(style: .dark))

  override init(frame: CGRect) {
    super.init(frame: frame)
    isUserInteractionEnabled = false
    backgroundColor = .black
    clipsToBounds = true
    imageLayer.contentsGravity = .resizeAspectFill
    imageLayer.magnificationFilter = .linear
    imageLayer.minificationFilter = .linear
    layer.addSublayer(imageLayer)
    layoutLockedPortraitPreview()

    blurEffectView.translatesAutoresizingMaskIntoConstraints = false
    addSubview(blurEffectView)
    NSLayoutConstraint.activate([
      blurEffectView.topAnchor.constraint(equalTo: topAnchor),
      blurEffectView.leadingAnchor.constraint(equalTo: leadingAnchor),
      blurEffectView.trailingAnchor.constraint(equalTo: trailingAnchor),
      blurEffectView.bottomAnchor.constraint(equalTo: bottomAnchor),
    ])
    blurEffectView.alpha = 1
  }

  required init?(coder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }

  override func layoutSubviews() {
    super.layoutSubviews()
    layoutLockedPortraitPreview()
  }

  func display(_ pixelBuffer: CVPixelBuffer) {
    CATransaction.begin()
    CATransaction.setDisableActions(true)
    imageLayer.contents = pixelBuffer
    CATransaction.commit()
  }

  func detach() {
    trackingService?.detachPreview(self)
    trackingService = nil
  }

  func setSourceReady(_ ready: Bool, animated: Bool) {
    let targetAlpha: CGFloat = ready ? 0 : 1
    if abs(blurEffectView.alpha - targetAlpha) < 0.01 { return }
    if animated, ready {
      UIView.animate(
        withDuration: 0.35,
        delay: 0.05,
        options: [.curveEaseOut, .beginFromCurrentState]
      ) {
        self.blurEffectView.alpha = 0
      }
    } else {
      blurEffectView.alpha = targetAlpha
    }
  }

  /// Matches the standard camera's portrait lock: the sensor is landscape,
  /// and the layer is rotated 90° before the first frame arrives.
  private func layoutLockedPortraitPreview() {
    let size = bounds.size
    guard size.width > 0, size.height > 0 else { return }
    CATransaction.begin()
    CATransaction.setDisableActions(true)
    imageLayer.bounds = CGRect(x: 0, y: 0, width: size.height, height: size.width)
    imageLayer.position = CGPoint(x: size.width / 2, y: size.height / 2)
    imageLayer.setAffineTransform(CGAffineTransform(rotationAngle: .pi / 2))
    CATransaction.commit()
  }
}
