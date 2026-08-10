import AVFoundation
import SwiftUI
import UIKit

struct CameraPreviewView: UIViewRepresentable {
  let session: AVCaptureSession

  func makeUIView(context: Context) -> CameraPreviewContainerView {
    let view = CameraPreviewContainerView()
    view.previewLayer.session = session
    return view
  }

  func updateUIView(_ uiView: CameraPreviewContainerView, context: Context) {
    uiView.previewLayer.session = session
    if let connection = uiView.previewLayer.connection,
      connection.isVideoRotationAngleSupported(90)
    {
      connection.videoRotationAngle = 90
    }
  }
}

final class CameraPreviewContainerView: UIView {
  override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }

  var previewLayer: AVCaptureVideoPreviewLayer {
    layer as! AVCaptureVideoPreviewLayer
  }

  override init(frame: CGRect) {
    super.init(frame: frame)
    isUserInteractionEnabled = false
    previewLayer.videoGravity = .resizeAspectFill
  }

  required init?(coder: NSCoder) {
    fatalError("init(coder:) has not been implemented")
  }
}
