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
  double yawDegrees = 0;
  double pitchDegrees = 0;
  int exifOrientation = 1;
  CameraIntrinsics intrinsics;

  /// Row-major rotation mapping display-oriented camera coordinates into the
  /// gravity-level capture reference frame. Used for locked-K pitch prior only;
  /// arrangement is rediscovered from matches.
  std::array<double, 9> cameraToCaptureReferenceRotation{};
};

struct StitchRequest {
  std::vector<FrameInput> frames;
  std::filesystem::path outputDirectory;
};

struct StitchArtifacts {
  std::filesystem::path panoramaPath;
  std::filesystem::path reportPath;
};

/// iOS-native outdoor stitch recipe:
/// match-based estimate + locked shared intrinsics + CoreMotion ring pitch prior.
class PanoramaEngine final {
public:
  static StitchArtifacts stitch(const StitchRequest &request);
};

} // namespace sphera
