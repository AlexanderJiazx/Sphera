#pragma once

#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#pragma clang diagnostic pop

namespace sphera {

struct DirectSphereFillStats {
  bool enabled = true;
  int filledPixels = 0;
  int remainingPixels = 0;
  std::vector<int> filledPixelsByInput;
};

/// Fill uncovered equirectangular pixels by direct inverse projection.
///
/// `composeScaleImages` must already be oriented and resized to composeScale.
/// `cameras` are at workScale (OpenCV detail::CameraParams); intrinsics are
/// scaled by composeScale / workScale. camera.R is camera-to-world (capture_ref);
/// world→camera uses R.t().
DirectSphereFillStats fillEquirectangularHoles(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras, double workScale,
    double composeScale, int rowChunk = 96);

} // namespace sphera
