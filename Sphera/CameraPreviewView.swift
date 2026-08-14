import AVFoundation
import SwiftUI
import UIKit

struct CameraPreviewView: UIViewRepresentable {
  let session: AVCaptureSession
  let isSourceReady: Bool

  func makeUIView(context: Context) -> CameraPreviewContainerView {
    let view = CameraPreviewContainerView()
    view.previewLayer.session = session
    view.setSourceReady(isSourceReady, animated: false)
    return view
  }

  func updateUIView(_ uiView: CameraPreviewContainerView, context: Context) {
    uiView.previewLayer.session = session
    if let connection = uiView.previewLayer.connection,
      connection.isVideoRotationAngleSupported(90)
    {
      connection.videoRotationAngle = 90
    }
    uiView.setSourceReady(isSourceReady, animated: true)
  }
}

final class CameraPreviewContainerView: UIView {
  override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }

  var previewLayer: AVCaptureVideoPreviewLayer {
    layer as! AVCaptureVideoPreviewLayer
  }

  private let blurEffectView = UIVisualEffectView(effect: UIBlurEffect(style: .dark))

  override init(frame: CGRect) {
    super.init(frame: frame)
    isUserInteractionEnabled = false
    previewLayer.videoGravity = .resizeAspectFill

    blurEffectView.translatesAutoresizingMaskIntoConstraints = false
    addSubview(blurEffectView)
    NSLayoutConstraint.activate([
      blurEffectView.topAnchor.constraint(equalTo: topAnchor),
      blurEffectView.leadingAnchor.constraint(equalTo: leadingAnchor),
      blurEffectView.trailingAnchor.constraint(equalTo: trailingAnchor),
      blurEffectView.bottomAnchor.constraint(equalTo: bottomAnchor),
    ])
    blurEffectView.alpha = 1.0
  }

  func setSourceReady(_ ready: Bool, animated: Bool) {
    let targetAlpha: CGFloat = ready ? 0.0 : 1.0
    if animated {
      if ready {
        UIView.animate(
          withDuration: 0.35,
          delay: 0.05,
          options: [.curveEaseOut, .beginFromCurrentState]
        ) {
          self.blurEffectView.alpha = 0.0
        }
      } else {
        blurEffectView.alpha = 1.0
      }
    } else {
      blurEffectView.alpha = targetAlpha
    }
  }

  required init?(coder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }
}
