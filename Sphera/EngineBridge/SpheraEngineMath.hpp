#pragma once

#include <array>
#include <cmath>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#pragma clang diagnostic pop

namespace sphera {

/// Project a nearly-rotational matrix onto SO(3) via SVD (det = +1).
cv::Mat properRotation(const cv::Mat &input);

/// Rodrigues rotation vector for an SO(3) matrix.
cv::Vec3d rotationVector(const cv::Mat &rotation);

/// Geodesic angle of an SO(3) matrix in degrees.
double rotationAngleDegrees(const cv::Mat &rotation);

/// 3×3 skew-symmetric matrix for a 3-vector.
cv::Matx33d skew(const cv::Vec3d &vector);

/// Capture-metadata camera→reference → OpenCV detail R as camera-to-world.
/// Matches Python `ios_to_opencv_rotation(..., "capture_ref")`:
/// `proper(diag(1,-1,-1) @ R_ios)` with NO transpose.
cv::Mat iosToOpenCVRotationCaptureRef(const std::array<double, 9> &rowMajorIos);

cv::Mat iosToOpenCVRotationCaptureRef(const cv::Mat &rotationIos);

/// Yaw (deg) / pitch (deg) from a camera-to-world rotation (OpenCV axes).
std::pair<double, double> yawPitchFromCameraToWorld(const cv::Mat &rotation);

cv::Mat matFromRowMajor9(const std::array<double, 9> &values);

std::array<double, 9> rowMajor9FromMat(const cv::Mat &rotation);

double medianOf(std::vector<double> values);

double percentileOf(std::vector<double> values, double percentile);

} // namespace sphera
