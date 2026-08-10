#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace sphera {

enum class CaptureRing {
  horizontal,
  downward,
  upward,
};

struct CameraIntrinsics {
  double fx = 0;
  double fy = 0;
  double cx = 0;
  double cy = 0;
  int referenceWidth = 0;
  int referenceHeight = 0;
};

struct FrameInput {
  std::filesystem::path imagePath;
  std::string imageFilename;
  int sequenceIndex = 0;
  CaptureRing ring = CaptureRing::horizontal;
  int ringIndex = 0;
  int ringCount = 0;
  int exifOrientation = 1;
  CameraIntrinsics intrinsics;

  /// Row-major rotation mapping display-oriented camera coordinates into the
  /// gravity-level capture reference frame.
  std::array<double, 9> cameraToCaptureReferenceRotation{};
};

struct StitchRequest {
  std::vector<FrameInput> frames;
  std::filesystem::path outputDirectory;
  double maximumPoseRefinementDegrees = 8;
  int outputWidth = 4096;
};

struct StitchArtifacts {
  std::filesystem::path panoramaPath;
  std::filesystem::path reportPath;
};

/// iOS-native port of the existing Sphera OpenCV stages. The capture pose is
/// the camera solution's starting point; this engine deliberately has no
/// global arrangement-estimation entry point.
class PanoramaEngine final {
public:
  static StitchArtifacts stitch(const StitchRequest &request);
};

} // namespace sphera
