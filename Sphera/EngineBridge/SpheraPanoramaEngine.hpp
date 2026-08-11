#pragma once

#include <array>
#include <filesystem>
#include <functional>
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

  /// Row-major camera→capture-reference rotation from CoreMotion metadata.
  /// Converted with capture_ref and stored as camera-to-world in CameraParams.R.
  std::array<double, 9> cameraToCaptureReferenceRotation{};
};

using StitchProgressCallback =
    std::function<void(double fraction, const std::string &message)>;

struct StitchRequest {
  std::vector<FrameInput> frames;
  std::filesystem::path outputDirectory;
  /// Optional LoFTR match cache. Loaded only when enableLegacyLearnedMatches.
  std::filesystem::path learnedMatchCacheDirectory;
  /// Developer diagnostic only. Default false — product path loads zero ML.
  bool enableLegacyLearnedMatches = false;
  StitchProgressCallback progress;
};

struct StitchArtifacts {
  std::filesystem::path panoramaPath;
  std::filesystem::path reportPath;
  std::filesystem::path contributionMapPath;
};

/// Sensor-first S1 + adaptive periodic ring seam (no ML on the default path).
class PanoramaEngine final {
public:
  static StitchArtifacts stitch(const StitchRequest &request);
};

} // namespace sphera
