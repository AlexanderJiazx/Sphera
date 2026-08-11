#include "SpheraEngineMath.hpp"

#include <algorithm>
#include <stdexcept>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/calib3d.hpp>
#pragma clang diagnostic pop

namespace sphera {

cv::Mat properRotation(const cv::Mat &input) {
  cv::Mat rotation64;
  input.convertTo(rotation64, CV_64F);
  if (rotation64.rows != 3 || rotation64.cols != 3 ||
      !cv::checkRange(rotation64)) {
    throw std::runtime_error("A camera rotation is not a finite 3x3 matrix");
  }
  cv::SVD decomposition(rotation64, cv::SVD::FULL_UV);
  cv::Mat left = decomposition.u.clone();
  cv::Mat rotation = left * decomposition.vt;
  if (cv::determinant(rotation) < 0) {
    left.col(2) *= -1;
    rotation = left * decomposition.vt;
  }
  return rotation;
}

cv::Vec3d rotationVector(const cv::Mat &rotation) {
  cv::Mat vector;
  cv::Rodrigues(properRotation(rotation), vector);
  return cv::Vec3d(vector.at<double>(0), vector.at<double>(1),
                   vector.at<double>(2));
}

double rotationAngleDegrees(const cv::Mat &rotation) {
  const cv::Vec3d vector = rotationVector(rotation);
  return std::sqrt(vector.dot(vector)) * (180.0 / CV_PI);
}

cv::Matx33d skew(const cv::Vec3d &vector) {
  const double x = vector[0];
  const double y = vector[1];
  const double z = vector[2];
  return cv::Matx33d(0.0, -z, y, z, 0.0, -x, -y, x, 0.0);
}

cv::Mat matFromRowMajor9(const std::array<double, 9> &values) {
  cv::Mat matrix(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      matrix.at<double>(row, column) =
          values[static_cast<std::size_t>(row * 3 + column)];
    }
  }
  return matrix;
}

std::array<double, 9> rowMajor9FromMat(const cv::Mat &rotation) {
  cv::Mat matrix64;
  rotation.convertTo(matrix64, CV_64F);
  std::array<double, 9> values{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      values[static_cast<std::size_t>(row * 3 + column)] =
          matrix64.at<double>(row, column);
    }
  }
  return values;
}

cv::Mat iosToOpenCVRotationCaptureRef(const cv::Mat &rotationIos) {
  // Left-multiply world-axis fix; do not transpose (camera-to-world).
  static const cv::Matx33d kAxisFix(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                                    -1.0);
  return properRotation(cv::Mat(kAxisFix) * rotationIos);
}

cv::Mat
iosToOpenCVRotationCaptureRef(const std::array<double, 9> &rowMajorIos) {
  return iosToOpenCVRotationCaptureRef(matFromRowMajor9(rowMajorIos));
}

std::pair<double, double> yawPitchFromCameraToWorld(const cv::Mat &rotation) {
  cv::Mat rotation64;
  rotation.convertTo(rotation64, CV_64F);
  const cv::Vec3d forward =
      cv::Vec3d(rotation64.at<double>(0, 2), rotation64.at<double>(1, 2),
                rotation64.at<double>(2, 2));
  const double yaw =
      std::atan2(forward[0], forward[2]) * (180.0 / CV_PI);
  const double pitch =
      std::atan2(-forward[1], std::hypot(forward[0], forward[2])) *
      (180.0 / CV_PI);
  return {yaw, pitch};
}

double medianOf(std::vector<double> values) {
  if (values.empty()) {
    throw std::runtime_error("Cannot compute median of an empty set");
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(),
                   values.begin() + static_cast<std::ptrdiff_t>(mid),
                   values.end());
  if (values.size() % 2 == 1) {
    return values[mid];
  }
  const double upper = values[mid];
  std::nth_element(values.begin(),
                   values.begin() + static_cast<std::ptrdiff_t>(mid - 1),
                   values.end());
  return 0.5 * (values[mid - 1] + upper);
}

double percentileOf(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  percentile = std::clamp(percentile, 0.0, 100.0);
  std::sort(values.begin(), values.end());
  if (values.size() == 1) {
    return values.front();
  }
  const double rank =
      (percentile / 100.0) * static_cast<double>(values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(rank));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(rank));
  if (lower == upper) {
    return values[lower];
  }
  const double weight = rank - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

} // namespace sphera
