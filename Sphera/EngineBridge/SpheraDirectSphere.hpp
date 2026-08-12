#pragma once

#include "SpheraPoseOverlap.hpp"

#include <string>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#pragma clang diagnostic pop

namespace sphera {

struct PolarCubeFaceStats {
  bool enabled = false;
  std::string pole = "top";
  int sourceCount = 0;
  int feedCount = 0;
  int replacedPixels = 0;
  int newlyCoveredPixels = 0;
  double graphCutSeconds = 0;
  double elapsedSeconds = 0;
  int topologyPrunedComponents = 0;
  int topologyReassignedPixels = 0;
  bool centralPairSelected = false;
  double centralPairCoverage = 0;
  double centralPairScore = 0;
  double centralPairSeconds = 0;
  std::vector<int> centralPairInputIndices;
  bool centralPairGateRejected = false;
  bool responseFieldAccepted = false;
  int responseFieldEquationCount = 0;
  int responseFieldPairCount = 0;
  double responseFieldSeconds = 0;
  double responseFieldMedianBefore = 0;
  double responseFieldMedianAfter = 0;
  double responseFieldP90Before = 0;
  double responseFieldP90After = 0;
  double responseFieldPeakMegabytes = 0;
  bool responseFieldGateRejected = false;
  std::vector<cv::Vec2d> responseFieldGainRangesByInput;
  cv::Vec3d photometricGainsBGR = cv::Vec3d(1, 1, 1);
  bool longitudeGainAccepted = false;
  bool longitudeGainRejectedByCapPressure = false;
  int longitudeGainSupportedColumns = 0;
  double longitudeGainMinimum = 1.0;
  double longitudeGainMaximum = 1.0;
  double longitudeGainP05 = 1.0;
  double longitudeGainP95 = 1.0;
  std::vector<int> selectedPixelsByInput;
};

/// Render the zenith in a regular perspective domain before inverse mapping.
///
/// OpenCV's spherical warper is singular at the pole.  This compositor uses
/// only upward capture frames, finds ownership and blends on a top cube face,
/// then feathers the central face into the equirectangular result.  It updates
/// `mask` and `labels` for newly covered pixels so direct sphere fill can remain
/// as a fallback for any residual holes.
PolarCubeFaceStats composeTopCubeFace(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<PoseFrameLayout> &layouts, double workScale,
    double composeScale, int seamSize = 256, int composeSize = 1024,
    double fieldOfViewDegrees = 100.0, int blendBands = 5,
    double fullReplacementLatitudeDegrees = 78.0,
    double minimumReplacementLatitudeDegrees = 69.0);

/// Render a gated nadir cube face using downward capture frames.
///
/// Unlike the zenith path, this returns without modifying the panorama unless
/// the same-ray response solve is strongly supported. This protects textured
/// outdoor ground, where camera translation makes a single-viewpoint bottom
/// reconstruction invalid.
PolarCubeFaceStats composeBottomCubeFace(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<PoseFrameLayout> &layouts, double workScale,
    double composeScale, int seamSize = 256, int composeSize = 1024,
    double fieldOfViewDegrees = 100.0, int blendBands = 5,
    double fullReplacementLatitudeDegrees = 78.0,
    double minimumReplacementLatitudeDegrees = 69.0);

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
