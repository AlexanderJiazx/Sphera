#include "SpheraPanoramaEngine.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#pragma clang diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace sphera {
namespace {

// Outdoor fullmeta winner (outputs/outdoor_fullmeta_best/report.json).
constexpr double kWorkMegapixels = 1.0;
constexpr double kSeamMegapixels = 0.12;
constexpr int kComposeSourceMaximumDimension = 2000;
constexpr int kMaximumFeatures = 6000;
constexpr double kSiftContrastThreshold = 0.005;
constexpr float kMatchConfidence = 0.3f;
constexpr float kConfidenceThreshold = 0.12f;
constexpr int kIntraRingRadius = 1;
constexpr double kCrossRingTolerance = 0.14; // turns (= 50.4°)
constexpr double kPitchPriorWeight = 1.0;
constexpr double kRingSeamOverlapFraction = 0.25;
constexpr double kBlendStrength = 2.0;
constexpr double kPeriodicBlendPaddingFraction = 0.08;
constexpr int kSeamDilateIterations = 1;
constexpr double kPi = 3.14159265358979323846;

using cv::detail::CameraParams;
using cv::detail::ImageFeatures;
using cv::detail::MatchesInfo;

struct FrameLayout {
  int ring = 0;
  int localIndex = 0;
  int ringSize = 0;
  int direction = 1;
  double phase = 0;
};

struct PreparedFrame {
  FrameInput input;
  CameraIntrinsics intrinsics;
  double workScale = 1;
  int featureCount = 0;
  FrameLayout layout;
};

struct PitchPriorRecord {
  int index = 0;
  int ring = 0;
  double originalPitchDegrees = 0;
  double priorPitchDegrees = 0;
  double blendedPitchDegrees = 0;
  double ringDeltaDegrees = 0;
};

struct SeamProducts {
  std::vector<cv::UMat> masks;
  std::vector<cv::Point> corners;
  cv::Ptr<cv::detail::ExposureCompensator> compensator;
  int contributingFrameCount = 0;
  std::string exposureStatus;
  std::string seamStatus;
  double seamScale = 1;
  float warperScale = 1;
};

std::string ringName(CaptureRing ring) {
  switch (ring) {
  case CaptureRing::horizontal:
    return "horizontal";
  case CaptureRing::downward:
    return "downward";
  case CaptureRing::upward:
    return "upward";
  }
}

void validateRequest(const StitchRequest &request) {
  if (request.frames.size() < 2) {
    throw std::runtime_error(
        "Sphera needs at least two frames to stitch a panorama");
  }
  for (const FrameInput &frame : request.frames) {
    if (!std::filesystem::is_regular_file(frame.imagePath)) {
      throw std::runtime_error("Captured image is missing: " +
                               frame.imagePath.string());
    }
    if (frame.intrinsics.fx <= 0 || frame.intrinsics.fy <= 0 ||
        frame.intrinsics.referenceWidth <= 0 ||
        frame.intrinsics.referenceHeight <= 0) {
      throw std::runtime_error("Invalid camera intrinsics for " +
                               frame.imageFilename);
    }
    for (double value : frame.cameraToCaptureReferenceRotation) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Invalid recorded pose for " +
                                 frame.imageFilename);
      }
    }
  }
}

cv::Mat orthonormalizedRotation(const cv::Mat &input) {
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


cv::Mat iosToOpenCVRotationY180(const FrameInput &frame) {
  // pose_priors.ios_to_opencv_rotation(..., "y180"): camera-to-world, no
  // transpose. Used only for the CoreMotion pitch prior.
  cv::Mat captureRotation(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      captureRotation.at<double>(row, column) =
          frame.cameraToCaptureReferenceRotation[static_cast<std::size_t>(
              row * 3 + column)];
    }
  }
  const cv::Matx33d axisFix(-1, 0, 0, 0, 1, 0, 0, 0, -1);
  return orthonormalizedRotation(captureRotation * cv::Mat(axisFix));
}

cv::Mat iosToOpenCVRotationY180W2C(const FrameInput &frame) {
  // OpenCV detail CameraParams.R is world-to-camera (y180_w2c).
  return orthonormalizedRotation(iosToOpenCVRotationY180(frame).t());
}

cv::Mat applyExifOrientation(const cv::Mat &encoded, int orientation) {
  cv::Mat oriented;
  switch (orientation) {
  case 2:
    cv::flip(encoded, oriented, 1);
    break;
  case 3:
    cv::rotate(encoded, oriented, cv::ROTATE_180);
    break;
  case 4:
    cv::flip(encoded, oriented, 0);
    break;
  case 5:
    cv::transpose(encoded, oriented);
    break;
  case 6:
    cv::rotate(encoded, oriented, cv::ROTATE_90_CLOCKWISE);
    break;
  case 7:
    cv::transpose(encoded, oriented);
    cv::flip(oriented, oriented, -1);
    break;
  case 8:
    cv::rotate(encoded, oriented, cv::ROTATE_90_COUNTERCLOCKWISE);
    break;
  default:
    oriented = encoded;
    break;
  }
  return oriented;
}

cv::Mat loadOrientedImage(const FrameInput &frame) {
  cv::Mat encoded =
      cv::imread(frame.imagePath.string(),
                 cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
  if (encoded.empty()) {
    throw std::runtime_error("OpenCV could not decode " + frame.imageFilename);
  }
  // Some iOS decoders already honor EXIF despite IGNORE_ORIENTATION. If the
  // bitmap already matches the calibrated display-oriented size, keep it.
  if (encoded.cols == frame.intrinsics.referenceWidth &&
      encoded.rows == frame.intrinsics.referenceHeight) {
    return encoded;
  }
  cv::Mat oriented = applyExifOrientation(encoded, frame.exifOrientation);
  if (oriented.cols != frame.intrinsics.referenceWidth ||
      oriented.rows != frame.intrinsics.referenceHeight) {
    throw std::runtime_error(
        "Oriented image size does not match calibrated reference for " +
        frame.imageFilename + " (got " + std::to_string(oriented.cols) + "x" +
        std::to_string(oriented.rows) + ", expected " +
        std::to_string(frame.intrinsics.referenceWidth) + "x" +
        std::to_string(frame.intrinsics.referenceHeight) + ")");
  }
  return oriented;
}

double megapixelScale(cv::Size size, double targetMegapixels) {
  const double pixels = static_cast<double>(size.width) * size.height;
  if (pixels <= 0) {
    return 1;
  }
  return std::min(1.0, std::sqrt(targetMegapixels * 1'000'000.0 / pixels));
}

double maximumDimensionScale(cv::Size size, int maximumDimension) {
  const int currentMaximum = std::max(size.width, size.height);
  return currentMaximum > maximumDimension
             ? static_cast<double>(maximumDimension) / currentMaximum
             : 1.0;
}

cv::Mat resizedImage(const cv::Mat &source, double scale) {
  if (std::abs(scale - 1.0) < 1e-6) {
    return source;
  }
  cv::Mat resized;
  const int interpolation = scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR;
  cv::resize(source, resized, cv::Size(), scale, scale, interpolation);
  return resized;
}

CameraIntrinsics intrinsicsAdjustedToDecodedSize(const FrameInput &frame,
                                                 cv::Size decodedSize) {
  CameraIntrinsics adjusted = frame.intrinsics;
  const double scaleX =
      static_cast<double>(decodedSize.width) / frame.intrinsics.referenceWidth;
  const double scaleY = static_cast<double>(decodedSize.height) /
                        frame.intrinsics.referenceHeight;
  adjusted.fx *= scaleX;
  adjusted.cx *= scaleX;
  adjusted.fy *= scaleY;
  adjusted.cy *= scaleY;
  adjusted.referenceWidth = decodedSize.width;
  adjusted.referenceHeight = decodedSize.height;
  return adjusted;
}

double circularDistance(double a, double b) {
  const double distance = std::abs(a - b);
  return std::min(distance, 1.0 - distance);
}

double medianOf(std::vector<double> values) {
  if (values.empty()) {
    throw std::runtime_error("Cannot compute median of an empty set");
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid),
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

int pitchSignForRing(CaptureRing ring) {
  switch (ring) {
  case CaptureRing::horizontal:
    return 0;
  case CaptureRing::downward:
    return -1;
  case CaptureRing::upward:
    return 1;
  }
}

int ringOrderIndex(CaptureRing ring) {
  switch (ring) {
  case CaptureRing::horizontal:
    return 0;
  case CaptureRing::downward:
    return 1;
  case CaptureRing::upward:
    return 2;
  }
}

/// Horizon → downward → upward; within each ring, ascending yaw so direction=+1.
std::vector<PreparedFrame>
buildAdaptiveLayout(const std::vector<FrameInput> &frames) {
  std::vector<CaptureRing> ringOrder = {
      CaptureRing::horizontal, CaptureRing::downward, CaptureRing::upward};
  std::unordered_map<int, std::vector<FrameInput>> byRing;
  for (const FrameInput &frame : frames) {
    byRing[ringOrderIndex(frame.ring)].push_back(frame);
  }

  std::vector<PreparedFrame> prepared;
  prepared.reserve(frames.size());
  int layoutRing = 0;
  for (CaptureRing ring : ringOrder) {
    auto found = byRing.find(ringOrderIndex(ring));
    if (found == byRing.end() || found->second.empty()) {
      continue;
    }
    std::vector<FrameInput> members = found->second;
    if (members.size() < 2) {
      throw std::runtime_error(
          "Each capture ring must contain at least two images (" +
          ringName(ring) + ")");
    }
    std::sort(members.begin(), members.end(),
              [](const FrameInput &left, const FrameInput &right) {
                if (left.yawDegrees != right.yawDegrees) {
                  return left.yawDegrees < right.yawDegrees;
                }
                return left.ringIndex < right.ringIndex;
              });

    const int ringSize = static_cast<int>(members.size());
    constexpr int direction = 1;
    for (int localIndex = 0; localIndex < ringSize; ++localIndex) {
      PreparedFrame item;
      item.input = members[static_cast<std::size_t>(localIndex)];
      item.input.ringIndex = localIndex;
      item.input.ringCount = ringSize;
      item.layout.ring = layoutRing;
      item.layout.localIndex = localIndex;
      item.layout.ringSize = ringSize;
      item.layout.direction = direction;
      item.layout.phase =
          std::fmod(direction * localIndex / static_cast<double>(ringSize),
                    1.0);
      if (item.layout.phase < 0) {
        item.layout.phase += 1.0;
      }
      prepared.push_back(std::move(item));
    }
    ++layoutRing;
  }

  if (prepared.size() != frames.size()) {
    throw std::runtime_error(
        "Adaptive ring layout did not retain every captured frame");
  }
  return prepared;
}

cv::Mat buildMatchMask(const std::vector<PreparedFrame> &frames,
                       int &approvedPairCount) {
  const int count = static_cast<int>(frames.size());
  cv::Mat mask = cv::Mat::zeros(count, count, CV_8U);
  approvedPairCount = 0;
  for (int left = 0; left < count; ++left) {
    for (int right = left + 1; right < count; ++right) {
      const FrameLayout &first = frames[static_cast<std::size_t>(left)].layout;
      const FrameLayout &second = frames[static_cast<std::size_t>(right)].layout;
      bool allowed = false;
      if (first.ring == second.ring) {
        const int forward =
            (first.localIndex - second.localIndex + first.ringSize) %
            first.ringSize;
        const int backward =
            (second.localIndex - first.localIndex + first.ringSize) %
            first.ringSize;
        allowed = std::min(forward, backward) <= kIntraRingRadius;
      } else {
        allowed =
            circularDistance(first.phase, second.phase) <= kCrossRingTolerance;
      }
      if (allowed) {
        mask.at<uchar>(left, right) = 1;
        mask.at<uchar>(right, left) = 1;
        ++approvedPairCount;
      }
    }
  }
  return mask;
}

std::pair<double, double> yawPitchFromRotation(const cv::Mat &rotation) {
  cv::Mat rotation64;
  rotation.convertTo(rotation64, CV_64F);
  const cv::Vec3d forward(rotation64.at<double>(0, 2),
                          rotation64.at<double>(1, 2),
                          rotation64.at<double>(2, 2));
  const double yaw =
      std::atan2(forward[0], forward[2]) * 180.0 / kPi;
  const double pitch =
      std::atan2(-forward[1], std::hypot(forward[0], forward[2])) * 180.0 /
      kPi;
  return {yaw, pitch};
}

cv::Mat rotationBetween(const cv::Vec3d &sourceIn, const cv::Vec3d &targetIn) {
  cv::Vec3d source = sourceIn / std::max(cv::norm(sourceIn), 1e-12);
  cv::Vec3d target = targetIn / std::max(cv::norm(targetIn), 1e-12);
  cv::Vec3d cross = source.cross(target);
  const double sine = cv::norm(cross);
  const double cosine = std::clamp(source.dot(target), -1.0, 1.0);
  if (sine < 1e-9) {
    if (cosine > 0) {
      return cv::Mat::eye(3, 3, CV_64F);
    }
    cv::Vec3d axis = source.cross(cv::Vec3d(1, 0, 0));
    if (cv::norm(axis) < 1e-6) {
      axis = source.cross(cv::Vec3d(0, 1, 0));
    }
    axis /= cv::norm(axis);
    cv::Mat matrix;
    cv::Rodrigues(axis * kPi, matrix);
    return matrix;
  }
  cv::Vec3d axis = cross / sine;
  cv::Mat matrix;
  cv::Rodrigues(axis * std::atan2(sine, cosine), matrix);
  return matrix;
}

void applyLockedIntrinsics(std::vector<CameraParams> &cameras,
                           const std::vector<PreparedFrame> &frames,
                           bool lockSharedFocal) {
  std::vector<double> focals;
  std::vector<double> aspects;
  focals.reserve(frames.size());
  aspects.reserve(frames.size());
  for (const PreparedFrame &frame : frames) {
    focals.push_back(frame.intrinsics.fx * frame.workScale);
    aspects.push_back(frame.intrinsics.fy /
                      std::max(frame.intrinsics.fx, 1e-9));
  }
  const double sharedFocal = lockSharedFocal ? medianOf(focals) : 0;
  const double sharedAspect = lockSharedFocal ? medianOf(aspects) : 0;
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    const PreparedFrame &frame = frames[index];
    cameras[index].focal =
        lockSharedFocal ? sharedFocal : focals[index];
    cameras[index].aspect =
        lockSharedFocal ? sharedAspect : aspects[index];
    cameras[index].ppx = frame.intrinsics.cx * frame.workScale;
    cameras[index].ppy = frame.intrinsics.cy * frame.workScale;
  }
}

std::vector<CameraParams>
camerasFromRecordedPoses(const std::vector<PreparedFrame> &frames) {
  std::vector<CameraParams> cameras;
  cameras.reserve(frames.size());
  for (const PreparedFrame &frame : frames) {
    CameraParams camera;
    cv::Mat rotation32;
    iosToOpenCVRotationY180W2C(frame.input).convertTo(rotation32, CV_32F);
    camera.R = rotation32;
    camera.t = cv::Mat::zeros(3, 1, CV_64F);
    cameras.push_back(camera);
  }
  applyLockedIntrinsics(cameras, frames, true);
  return cameras;
}

struct LearnedMatchStats {
  bool enabled = false;
  int acceptedPairs = 0;
  int totalCorrespondences = 0;
  std::string mode = "unavailable";
};

LearnedMatchStats applyLoFTRMatchCache(
    std::vector<ImageFeatures> &features,
    std::vector<MatchesInfo> &pairwiseMatches,
    const std::vector<PreparedFrame> &frames,
    const cv::Mat &topologyMask,
    const std::filesystem::path &cacheDirectory) {
  LearnedMatchStats stats;
  if (cacheDirectory.empty()) {
    return stats;
  }
  const auto manifestPath = cacheDirectory / "manifest.json";
  if (!std::filesystem::exists(manifestPath)) {
    return stats;
  }

  stats.enabled = true;
  stats.mode = "loftr_outdoor_coarse_augment";

  // Pair files are listed as pair_SS_TT.bin with float32 x0,y0,x1,y1,conf.
  const int count = static_cast<int>(frames.size());
  for (int source = 0; source < count; ++source) {
    for (int target = source + 1; target < count; ++target) {
      if (!topologyMask.at<uchar>(source, target)) {
        continue;
      }
      char name[64];
      std::snprintf(name, sizeof(name), "pair_%02d_%02d.bin", source, target);
      const auto pairPath = cacheDirectory / name;
      if (!std::filesystem::exists(pairPath)) {
        continue;
      }
      std::ifstream input(pairPath, std::ios::binary);
      if (!input) {
        continue;
      }
      input.seekg(0, std::ios::end);
      const std::streamoff bytes = input.tellg();
      input.seekg(0, std::ios::beg);
      if (bytes <= 0 || bytes % (5 * sizeof(float)) != 0) {
        continue;
      }
      const int matchCount = static_cast<int>(bytes / (5 * sizeof(float)));
      std::vector<float> values(static_cast<std::size_t>(matchCount) * 5);
      input.read(reinterpret_cast<char *>(values.data()), bytes);
      if (!input) {
        continue;
      }

      const double workScale = frames[static_cast<std::size_t>(source)].workScale;
      // LoFTR cache coordinates are in the 480x640 model input space.
      constexpr double kLoFTRWidth = 480.0;
      constexpr double kLoFTRHeight = 640.0;
      const double srcW = frames[static_cast<std::size_t>(source)].intrinsics.referenceWidth * workScale;
      const double srcH = frames[static_cast<std::size_t>(source)].intrinsics.referenceHeight * workScale;
      const double dstW = frames[static_cast<std::size_t>(target)].intrinsics.referenceWidth * workScale;
      const double dstH = frames[static_cast<std::size_t>(target)].intrinsics.referenceHeight * workScale;
      const double scaleSrcX = srcW / kLoFTRWidth;
      const double scaleSrcY = srcH / kLoFTRHeight;
      const double scaleDstX = dstW / kLoFTRWidth;
      const double scaleDstY = dstH / kLoFTRHeight;

      std::vector<cv::Point2f> srcPts;
      std::vector<cv::Point2f> dstPts;
      srcPts.reserve(static_cast<std::size_t>(matchCount));
      dstPts.reserve(static_cast<std::size_t>(matchCount));
      for (int index = 0; index < matchCount; ++index) {
        const float *row = values.data() + index * 5;
        srcPts.emplace_back(row[0] * scaleSrcX, row[1] * scaleSrcY);
        dstPts.emplace_back(row[2] * scaleDstX, row[3] * scaleDstY);
      }
      if (srcPts.size() < 8) {
        continue;
      }

      // OpenCV stores MatchesInfo::H in image-centre-origin coordinates:
      // BestOf2NearestMatcher subtracts half the image size from every keypoint
      // before findHomography, and HomographyBasedEstimator relies on that when
      // it derives rotations with the principal point at the origin. Fitting in
      // top-left-origin coordinates injects a half-image translation into every
      // spanning-tree edge, which is tens of degrees of bogus rotation per edge.
      const cv::Size sourceSize = features[static_cast<std::size_t>(source)].img_size;
      const cv::Size targetSize = features[static_cast<std::size_t>(target)].img_size;
      const cv::Point2f sourceCentre(0.5f * sourceSize.width,
                                     0.5f * sourceSize.height);
      const cv::Point2f targetCentre(0.5f * targetSize.width,
                                     0.5f * targetSize.height);
      auto centredHomography = [&](const std::vector<cv::Point2f> &from,
                                   const std::vector<cv::Point2f> &to,
                                   cv::Mat &mask) {
        std::vector<cv::Point2f> centredFrom(from.size());
        std::vector<cv::Point2f> centredTo(to.size());
        for (std::size_t index = 0; index < from.size(); ++index) {
          centredFrom[index] = from[index] - sourceCentre;
          centredTo[index] = to[index] - targetCentre;
        }
        return cv::findHomography(centredFrom, centredTo, cv::RANSAC, 3.0, mask);
      };

      cv::Mat inlierMask;
      const cv::Mat learnedHomography = centredHomography(srcPts, dstPts, inlierMask);
      if (learnedHomography.empty() || inlierMask.empty()) {
        continue;
      }
      auto appendKeypoint = [](ImageFeatures &feature, const cv::Point2f &point) {
        const int index = static_cast<int>(feature.keypoints.size());
        feature.keypoints.emplace_back(point, 8.0f);
        cv::Mat descriptors = feature.descriptors.getMat(cv::ACCESS_RW);
        if (descriptors.empty()) {
          descriptors = cv::Mat::zeros(1, 128, CV_32F);
        } else {
          cv::Mat row = cv::Mat::zeros(1, descriptors.cols, descriptors.type());
          descriptors.push_back(row);
        }
        feature.descriptors = descriptors.getUMat(cv::ACCESS_READ);
        return index;
      };
      // Only the geometrically consistent subset earns a place in the graph.
      std::vector<cv::DMatch> learnedMatches;
      learnedMatches.reserve(srcPts.size());
      int inliers = 0;
      for (int index = 0; index < static_cast<int>(srcPts.size()); ++index) {
        if (!inlierMask.at<uchar>(index)) {
          continue;
        }
        ++inliers;
        const int srcIdx = appendKeypoint(
            features[static_cast<std::size_t>(source)],
            srcPts[static_cast<std::size_t>(index)]);
        const int dstIdx = appendKeypoint(
            features[static_cast<std::size_t>(target)],
            dstPts[static_cast<std::size_t>(index)]);
        learnedMatches.emplace_back(srcIdx, dstIdx, 0.0f);
      }

      if (inliers < 8) {
        continue;
      }

      // Augment rather than replace: keep the SIFT matches for this pair and
      // refit on the union. Overwriting them let a weak learned pair destroy a
      // strong SIFT pair, which is how the graph ended up worse than plain SIFT.
      const std::size_t forwardSlot =
          static_cast<std::size_t>(source * count + target);
      std::vector<cv::DMatch> combined = pairwiseMatches[forwardSlot].matches;
      combined.insert(combined.end(), learnedMatches.begin(),
                      learnedMatches.end());

      std::vector<cv::Point2f> combinedSrc;
      std::vector<cv::Point2f> combinedDst;
      combinedSrc.reserve(combined.size());
      combinedDst.reserve(combined.size());
      for (const cv::DMatch &match : combined) {
        combinedSrc.push_back(features[static_cast<std::size_t>(source)]
                                  .keypoints[static_cast<std::size_t>(
                                      match.queryIdx)]
                                  .pt);
        combinedDst.push_back(features[static_cast<std::size_t>(target)]
                                  .keypoints[static_cast<std::size_t>(
                                      match.trainIdx)]
                                  .pt);
      }
      cv::Mat combinedMask;
      const cv::Mat homography =
          centredHomography(combinedSrc, combinedDst, combinedMask);
      if (homography.empty() || combinedMask.empty()) {
        continue;
      }
      const int combinedInliers = cv::countNonZero(combinedMask);
      if (combinedInliers < 8) {
        continue;
      }

      MatchesInfo info;
      info.src_img_idx = source;
      info.dst_img_idx = target;
      info.matches = combined;
      info.inliers_mask.assign(combined.size(), 0);
      for (std::size_t index = 0; index < combined.size(); ++index) {
        info.inliers_mask[index] =
            combinedMask.at<uchar>(static_cast<int>(index)) ? 1 : 0;
      }
      info.num_inliers = combinedInliers;
      info.H = homography;
      info.confidence =
          std::min(3.0, combinedInliers / (8.0 + 0.3 * combined.size()));

      pairwiseMatches[forwardSlot] = info;
      MatchesInfo reverse = info;
      reverse.src_img_idx = target;
      reverse.dst_img_idx = source;
      if (!info.H.empty()) {
        reverse.H = info.H.inv();
      }
      for (cv::DMatch &match : reverse.matches) {
        std::swap(match.queryIdx, match.trainIdx);
      }
      pairwiseMatches[static_cast<std::size_t>(target * count + source)] = reverse;

      stats.acceptedPairs += 1;
      stats.totalCorrespondences += inliers;
    }
  }
  return stats;
}

bool normalizeWorldOrientation(std::vector<CameraParams> &cameras,
                               const std::vector<PreparedFrame> &frames,
                               const std::vector<int> &ringPitchSigns) {
  std::unordered_map<int, std::vector<double>> pitchByRing;
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    pitchByRing[frames[index].layout.ring].push_back(
        yawPitchFromRotation(cameras[index].R).second);
  }
  double score = 0;
  for (const auto &entry : pitchByRing) {
    if (entry.first < 0 ||
        entry.first >= static_cast<int>(ringPitchSigns.size())) {
      throw std::runtime_error("Ring pitch signs must contain one value per ring");
    }
    score += ringPitchSigns[static_cast<std::size_t>(entry.first)] *
             medianOf(entry.second);
  }
  if (score >= 0) {
    return false;
  }
  const cv::Matx33d worldRoll(-1, 0, 0, 0, -1, 0, 0, 0, 1);
  const cv::Mat roll(worldRoll);
  for (CameraParams &camera : cameras) {
    cv::Mat rotated = roll * orthonormalizedRotation(camera.R);
    cv::Mat rotation32;
    rotated.convertTo(rotation32, CV_32F);
    camera.R = rotation32;
  }
  return true;
}

std::vector<PitchPriorRecord>
pullRingPitchesFromMetadata(std::vector<CameraParams> &cameras,
                            const std::vector<PreparedFrame> &frames,
                            double weight) {
  weight = std::clamp(weight, 0.0, 1.0);
  std::vector<cv::Mat> priorC2W;
  priorC2W.reserve(frames.size());
  for (const PreparedFrame &frame : frames) {
    priorC2W.push_back(iosToOpenCVRotationY180(frame.input));
  }

  std::unordered_map<int, std::vector<int>> rings;
  for (std::size_t index = 0; index < frames.size(); ++index) {
    rings[frames[index].layout.ring].push_back(static_cast<int>(index));
  }

  std::vector<PitchPriorRecord> records;
  for (const auto &entry : rings) {
    const std::vector<int> &indices = entry.second;
    std::vector<double> estimated;
    std::vector<double> priorPitches;
    estimated.reserve(indices.size());
    priorPitches.reserve(indices.size());
    for (int index : indices) {
      estimated.push_back(yawPitchFromRotation(cameras[static_cast<std::size_t>(index)].R)
                              .second);
      priorPitches.push_back(
          yawPitchFromRotation(priorC2W[static_cast<std::size_t>(index)]).second);
    }

    double priorMedian = medianOf(priorPitches);
    const double estimatedMedian = medianOf(estimated);
    if (estimatedMedian * priorMedian < 0 && std::abs(estimatedMedian) > 5 &&
        std::abs(priorMedian) > 5) {
      priorMedian = -priorMedian;
      for (double &value : priorPitches) {
        value = -value;
      }
    }

    const double targetMedian =
        estimatedMedian * (1.0 - weight) + priorMedian * weight;
    const double delta = targetMedian - estimatedMedian;

    for (std::size_t local = 0; local < indices.size(); ++local) {
      const int index = indices[local];
      CameraParams &camera = cameras[static_cast<std::size_t>(index)];
      const auto [ey, ep] = yawPitchFromRotation(camera.R);
      const double framePrior = priorPitches[local];
      const double desired =
          ep + delta * weight + (framePrior - priorMedian) * weight * 0.35;
      const double eyRad = ey * kPi / 180.0;
      const double desiredRad = desired * kPi / 180.0;
      const cv::Vec3d desiredForward(
          std::sin(eyRad) * std::cos(desiredRad), -std::sin(desiredRad),
          std::cos(eyRad) * std::cos(desiredRad));
      cv::Mat rotation64;
      camera.R.convertTo(rotation64, CV_64F);
      const cv::Vec3d currentForward(rotation64.at<double>(0, 2),
                                     rotation64.at<double>(1, 2),
                                     rotation64.at<double>(2, 2));
      const cv::Mat alignment = rotationBetween(currentForward, desiredForward);
      cv::Mat updated;
      orthonormalizedRotation(alignment * rotation64)
          .convertTo(updated, CV_32F);
      camera.R = updated;

      PitchPriorRecord record;
      record.index = index;
      record.ring = entry.first;
      record.originalPitchDegrees = ep;
      record.priorPitchDegrees = framePrior;
      record.blendedPitchDegrees = desired;
      record.ringDeltaDegrees = delta;
      records.push_back(record);
    }
  }
  return records;
}

cv::Mat scaledCameraK(const CameraParams &camera, double aspectScale) {
  cv::Mat intrinsic = camera.K();
  intrinsic.convertTo(intrinsic, CV_32F);
  intrinsic.at<float>(0, 0) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(0, 2) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(1, 1) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(1, 2) *= static_cast<float>(aspectScale);
  return intrinsic;
}

void applyRingBandPriors(std::vector<cv::UMat> &masks,
                         const std::vector<cv::Point> &corners,
                         const std::vector<PreparedFrame> &frames,
                         double overlapFraction) {
  std::unordered_map<int, std::vector<double>> centersByRing;
  for (std::size_t index = 0; index < masks.size(); ++index) {
    cv::Mat mask = masks[index].getMat(cv::ACCESS_READ);
    std::vector<cv::Point> nonzero;
    cv::findNonZero(mask, nonzero);
    if (nonzero.empty()) {
      continue;
    }
    std::vector<int> rows;
    rows.reserve(nonzero.size());
    for (const cv::Point &point : nonzero) {
      rows.push_back(point.y);
    }
    std::nth_element(rows.begin(), rows.begin() + rows.size() / 2, rows.end());
    centersByRing[frames[index].layout.ring].push_back(
        corners[index].y + rows[rows.size() / 2]);
  }
  if (centersByRing.size() < 2) {
    return;
  }

  std::unordered_map<int, double> ringCenters;
  for (auto &entry : centersByRing) {
    ringCenters[entry.first] = medianOf(entry.second);
  }
  std::vector<int> ordered;
  ordered.reserve(ringCenters.size());
  for (const auto &entry : ringCenters) {
    ordered.push_back(entry.first);
  }
  std::sort(ordered.begin(), ordered.end(),
            [&](int left, int right) {
              return ringCenters[left] < ringCenters[right];
            });

  std::unordered_map<int, double> lowerBounds;
  std::unordered_map<int, double> upperBounds;
  for (int ring : ordered) {
    lowerBounds[ring] = -1e300;
    upperBounds[ring] = 1e300;
  }
  for (std::size_t index = 0; index + 1 < ordered.size(); ++index) {
    const int first = ordered[index];
    const int second = ordered[index + 1];
    const double firstCenter = ringCenters[first];
    const double secondCenter = ringCenters[second];
    const double gap = secondCenter - firstCenter;
    const double midpoint = 0.5 * (firstCenter + secondCenter);
    const double halfOverlap = gap * overlapFraction / 2.0;
    upperBounds[first] = midpoint + halfOverlap;
    lowerBounds[second] = midpoint - halfOverlap;
  }

  for (std::size_t index = 0; index < masks.size(); ++index) {
    const int ring = frames[index].layout.ring;
    cv::Mat writable = masks[index].getMat(cv::ACCESS_RW);
    for (int row = 0; row < writable.rows; ++row) {
      const double globalY = corners[index].y + row;
      if (globalY < lowerBounds[ring] || globalY > upperBounds[ring]) {
        writable.row(row).setTo(0);
      }
    }
  }
}

cv::Mat transferSeamMask(const cv::Mat &seamMask, cv::Point seamCorner,
                         cv::Point composeCorner, cv::Size composeSize,
                         double scaleRatio, int dilationIterations) {
  cv::Mat expanded;
  cv::dilate(seamMask, expanded, cv::Mat(), cv::Point(-1, -1),
             std::max(0, dilationIterations));
  cv::Mat transform = cv::Mat::zeros(2, 3, CV_64F);
  transform.at<double>(0, 0) = scaleRatio;
  transform.at<double>(0, 2) = seamCorner.x * scaleRatio - composeCorner.x;
  transform.at<double>(1, 1) = scaleRatio;
  transform.at<double>(1, 2) = seamCorner.y * scaleRatio - composeCorner.y;
  cv::Mat transferred;
  cv::warpAffine(expanded, transferred, transform, composeSize, cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT, cv::Scalar(0));
  return transferred;
}

bool feedPeriodicImage(cv::detail::Blender &blender, const cv::Mat &image,
                       const cv::Mat &mask, cv::Point corner,
                       const cv::Rect &destination, int sphereWidth) {
  bool contributed = false;
  for (int shift : {-sphereWidth, 0, sphereWidth}) {
    const cv::Rect shiftedBounds(corner.x + shift, corner.y, image.cols,
                                 image.rows);
    const cv::Rect intersection = shiftedBounds & destination;
    if (intersection.empty()) {
      continue;
    }
    const cv::Rect sourceRegion(intersection.x - shiftedBounds.x,
                                intersection.y - shiftedBounds.y,
                                intersection.width, intersection.height);
    if (cv::countNonZero(mask(sourceRegion)) == 0) {
      continue;
    }
    blender.feed(image(sourceRegion), mask(sourceRegion), intersection.tl());
    contributed = true;
  }
  return contributed;
}

void makeLongitudeBoundaryContinuous(cv::Mat &panorama, cv::Mat &mask) {
  if (panorama.cols < 2 || panorama.type() != CV_8UC3 || mask.type() != CV_8U) {
    return;
  }
  for (int row = 0; row < panorama.rows; ++row) {
    const bool leftValid = mask.at<uchar>(row, 0) != 0;
    const bool rightValid = mask.at<uchar>(row, panorama.cols - 1) != 0;
    if (leftValid && rightValid) {
      const cv::Vec3b left = panorama.at<cv::Vec3b>(row, 0);
      const cv::Vec3b right = panorama.at<cv::Vec3b>(row, panorama.cols - 1);
      cv::Vec3b average;
      for (int channel = 0; channel < 3; ++channel) {
        average[channel] = static_cast<uchar>(
            (static_cast<unsigned int>(left[channel]) + right[channel]) / 2);
      }
      panorama.at<cv::Vec3b>(row, 0) = average;
      panorama.at<cv::Vec3b>(row, panorama.cols - 1) = average;
    } else if (leftValid) {
      panorama.at<cv::Vec3b>(row, panorama.cols - 1) =
          panorama.at<cv::Vec3b>(row, 0);
      mask.at<uchar>(row, panorama.cols - 1) = 255;
    } else if (rightValid) {
      panorama.at<cv::Vec3b>(row, 0) =
          panorama.at<cv::Vec3b>(row, panorama.cols - 1);
      mask.at<uchar>(row, 0) = 255;
    }
  }
}

SeamProducts buildSeamsAndExposure(const std::vector<PreparedFrame> &frames,
                                   const std::vector<CameraParams> &cameras,
                                   float warperScaleWork, double workScale,
                                   double seamScale) {
  const int frameCount = static_cast<int>(frames.size());
  const double seamWorkAspect = seamScale / workScale;
  const float seamWarperScale =
      static_cast<float>(warperScaleWork * seamWorkAspect);
  cv::Ptr<cv::detail::RotationWarper> warper =
      cv::makePtr<cv::detail::SphericalWarper>(seamWarperScale);

  std::vector<cv::UMat> warpedImages(static_cast<std::size_t>(frameCount));
  std::vector<cv::UMat> warpedMasks(static_cast<std::size_t>(frameCount));
  std::vector<cv::Point> corners(static_cast<std::size_t>(frameCount));

  for (int index = 0; index < frameCount; ++index) {
    cv::Mat fullImage = loadOrientedImage(frames[static_cast<std::size_t>(index)].input);
    cv::Mat seamImage = resizedImage(fullImage, seamScale);
    cv::Mat sourceMask(seamImage.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic =
        scaledCameraK(cameras[static_cast<std::size_t>(index)], seamWorkAspect);
    cv::Mat rotation32;
    cameras[static_cast<std::size_t>(index)].R.convertTo(rotation32, CV_32F);

    corners[static_cast<std::size_t>(index)] = warper->warp(
        seamImage, intrinsic, rotation32, cv::INTER_LINEAR, cv::BORDER_REFLECT,
        warpedImages[static_cast<std::size_t>(index)]);
    warper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT,
                 warpedMasks[static_cast<std::size_t>(index)]);
  }

  applyRingBandPriors(warpedMasks, corners, frames, kRingSeamOverlapFraction);

  SeamProducts products;
  products.corners = corners;
  products.seamScale = seamScale;
  products.warperScale = seamWarperScale;
  products.masks.reserve(warpedMasks.size());
  for (const cv::UMat &mask : warpedMasks) {
    products.masks.push_back(mask.clone());
  }

  products.compensator = cv::detail::ExposureCompensator::createDefault(
      cv::detail::ExposureCompensator::GAIN_BLOCKS);
  try {
    products.compensator->feed(corners, warpedImages, products.masks);
    products.exposureStatus = "gain-blocks";
  } catch (const cv::Exception &) {
    products.compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
    products.exposureStatus = "disabled-after-open-cv-error";
  }

  // Winner uses graph-cut on RAW warped images (not exposure-compensated).
  std::vector<cv::UMat> floatingImages(warpedImages.size());
  for (std::size_t index = 0; index < warpedImages.size(); ++index) {
    warpedImages[index].convertTo(floatingImages[index], CV_32F);
  }

  try {
    cv::detail::GraphCutSeamFinder seamFinder(
        cv::detail::GraphCutSeamFinderBase::COST_COLOR_GRAD);
    seamFinder.find(floatingImages, corners, products.masks);
    int contributingFrames = 0;
    for (const cv::UMat &mask : products.masks) {
      contributingFrames += cv::countNonZero(mask) > 0 ? 1 : 0;
    }
    if (contributingFrames == 0) {
      products.masks.clear();
      for (const cv::UMat &mask : warpedMasks) {
        products.masks.push_back(mask.clone());
      }
      products.contributingFrameCount = static_cast<int>(std::count_if(
          warpedMasks.begin(), warpedMasks.end(),
          [](const cv::UMat &mask) { return cv::countNonZero(mask) > 0; }));
      products.seamStatus = "ring-prior-fallback-after-empty-graphcut-result";
    } else {
      products.contributingFrameCount = contributingFrames;
      products.seamStatus = "graphcut-color-gradient-with-ring-priors";
    }
  } catch (const cv::Exception &) {
    products.masks.clear();
    for (const cv::UMat &mask : warpedMasks) {
      products.masks.push_back(mask.clone());
    }
    products.contributingFrameCount = static_cast<int>(std::count_if(
        warpedMasks.begin(), warpedMasks.end(),
        [](const cv::UMat &mask) { return cv::countNonZero(mask) > 0; }));
    products.seamStatus = "ring-prior-fallback-after-open-cv-error";
  }

  return products;
}

std::filesystem::path composePanorama(
    const std::vector<PreparedFrame> &frames,
    const std::vector<CameraParams> &cameras, const SeamProducts &seams,
    float warperScaleWork, double workScale, double composeScale,
    const std::filesystem::path &outputDirectory) {
  const double composeWorkAspect = composeScale / workScale;
  const float composeWarperScale =
      static_cast<float>(warperScaleWork * composeWorkAspect);
  cv::Ptr<cv::detail::RotationWarper> warper =
      cv::makePtr<cv::detail::SphericalWarper>(composeWarperScale);

  std::vector<cv::Point> composeCorners(frames.size());
  std::vector<cv::Size> composeSizes(frames.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const cv::Size scaledSize(
        static_cast<int>(std::lround(frames[index].intrinsics.referenceWidth *
                                     composeScale)),
        static_cast<int>(std::lround(frames[index].intrinsics.referenceHeight *
                                     composeScale)));
    const cv::Mat intrinsic = scaledCameraK(cameras[index], composeWorkAspect);
    cv::Mat rotation32;
    cameras[index].R.convertTo(rotation32, CV_32F);
    const cv::Rect roi = warper->warpRoi(scaledSize, intrinsic, rotation32);
    composeCorners[index] = roi.tl();
    composeSizes[index] = roi.size();
  }

  const cv::Rect geometricRoi =
      cv::detail::resultRoi(composeCorners, composeSizes);
  const int sphereWidth =
      static_cast<int>(std::lround(2.0 * kPi * composeWarperScale));
  const int sphereHeight =
      static_cast<int>(std::lround(kPi * composeWarperScale));
  const int sphereLeft =
      static_cast<int>(std::lround(-kPi * composeWarperScale));
  const cv::Rect composeRoi(sphereLeft, geometricRoi.y, sphereWidth,
                            geometricRoi.height);
  const int periodicPadding = static_cast<int>(
      std::lround(sphereWidth * kPeriodicBlendPaddingFraction));
  const cv::Rect blenderRoi(composeRoi.x - periodicPadding, composeRoi.y,
                            composeRoi.width + 2 * periodicPadding,
                            composeRoi.height);

  const double blendWidth =
      std::sqrt(static_cast<double>(blenderRoi.width) * blenderRoi.height) *
      kBlendStrength / 100.0;
  const int blendNumBands =
      std::max(1, static_cast<int>(std::log2(std::max(2.0, blendWidth)) - 1.0));

  cv::detail::MultiBandBlender blender(false, blendNumBands);
  blender.setNumBands(blendNumBands);
  blender.prepare(blenderRoi);
  int feedCount = 0;

  for (std::size_t index = 0; index < frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(frames[index].input);
    cv::Mat image = resizedImage(fullImage, composeScale);
    cv::Mat sourceMask(image.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic = scaledCameraK(cameras[index], composeWorkAspect);
    cv::Mat rotation32;
    cameras[index].R.convertTo(rotation32, CV_32F);

    cv::Mat warpedImage;
    cv::Mat warpedMask;
    const cv::Point corner =
        warper->warp(image, intrinsic, rotation32, cv::INTER_LINEAR,
                     cv::BORDER_REFLECT, warpedImage);
    warper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT, warpedMask);
    seams.compensator->apply(static_cast<int>(index), corner, warpedImage,
                             warpedMask);

    const cv::Mat seamMask = transferSeamMask(
        seams.masks[index].getMat(cv::ACCESS_READ), seams.corners[index],
        corner, warpedMask.size(), composeScale / seams.seamScale,
        kSeamDilateIterations);
    cv::Mat selectedMask;
    cv::bitwise_and(warpedMask, seamMask, selectedMask);
    if (cv::countNonZero(selectedMask) == 0) {
      continue;
    }

    cv::Mat warped16;
    warpedImage.convertTo(warped16, CV_16S);
    if (feedPeriodicImage(blender, warped16, selectedMask, corner, blenderRoi,
                          sphereWidth)) {
      ++feedCount;
    }
  }

  if (feedCount == 0) {
    throw std::runtime_error(
        "No captured frame contributed pixels to the panorama");
  }

  cv::Mat blended;
  cv::Mat blendedMask;
  blender.blend(blended, blendedMask);
  if (blended.empty() || blendedMask.empty()) {
    throw std::runtime_error(
        "OpenCV returned an empty panorama after blending");
  }

  const int availableWidth =
      std::max(0, blended.cols - periodicPadding);
  const int cropWidth = std::min(sphereWidth, availableWidth);
  const int cropHeight = std::min(sphereHeight, blended.rows);
  if (cropWidth <= 0 || cropHeight <= 0) {
    throw std::runtime_error(
        "The blended panorama did not have a usable equirectangular crop");
  }
  const cv::Rect centralRegion(periodicPadding, 0, cropWidth, cropHeight);

  cv::Mat panorama16 = blended(centralRegion).clone();
  cv::Mat panoramaMask = blendedMask(centralRegion).clone();
  cv::Mat panorama;
  panorama16.convertTo(panorama, CV_8U);
  panorama.setTo(cv::Scalar::all(0), panoramaMask == 0);
  makeLongitudeBoundaryContinuous(panorama, panoramaMask);

  // Full 2:1 equirectangular canvas when geometric ROI is shorter than π.
  cv::Mat equirectangular(sphereHeight, sphereWidth, CV_8UC3, cv::Scalar::all(0));
  cv::Mat equirectangularMask(sphereHeight, sphereWidth, CV_8U,
                              cv::Scalar::all(0));
  const int dstY = std::clamp(composeRoi.y, 0, std::max(0, sphereHeight - 1));
  const int copyHeight =
      std::min(panorama.rows, sphereHeight - dstY);
  if (copyHeight > 0) {
    panorama(cv::Rect(0, 0, panorama.cols, copyHeight))
        .copyTo(equirectangular(cv::Rect(0, dstY, panorama.cols, copyHeight)));
    panoramaMask(cv::Rect(0, 0, panoramaMask.cols, copyHeight))
        .copyTo(
            equirectangularMask(cv::Rect(0, dstY, panoramaMask.cols, copyHeight)));
  }
  makeLongitudeBoundaryContinuous(equirectangular, equirectangularMask);

  const std::filesystem::path panoramaPath =
      outputDirectory / "panorama_equirectangular.jpg";
  if (!cv::imwrite(panoramaPath.string(), equirectangular,
                   {cv::IMWRITE_JPEG_QUALITY, 95})) {
    throw std::runtime_error("Could not write the equirectangular panorama");
  }
  return panoramaPath;
}

void writeReport(const std::vector<PreparedFrame> &frames,
                 const std::vector<int> &ringSizes,
                 const std::vector<int> &ringDirections,
                 const std::vector<int> &ringPitchSigns, int approvedPairCount,
                 bool orientationFlipped,
                 const std::string &cameraSeed,
                 const std::string &cameraFallbackReason,
                 const LearnedMatchStats &learnedStats,
                 const std::vector<PitchPriorRecord> &pitchRecords,
                 const SeamProducts &seams,
                 const std::filesystem::path &panoramaPath,
                 const std::filesystem::path &reportPath,
                 double elapsedSeconds, double workScale, double seamScale,
                 double composeScale) {
  cv::FileStorage report(reportPath.string(),
                         cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  if (!report.isOpened()) {
    throw std::runtime_error("Could not create the native engine report");
  }

  report << "engine" << "sphera-ios-native";
  report << "engine_contract_version" << 2;
  report << "opencv_version" << CV_VERSION;
  report << "status" << "success";
  report << "elapsed_seconds" << elapsedSeconds;
  report << "recipe" << "outdoor_fullmeta_best";

  report << "configuration" << "{";
  report << "work_megapix" << kWorkMegapixels;
  report << "seam_megapix" << kSeamMegapixels;
  report << "compose_max_dimension" << kComposeSourceMaximumDimension;
  report << "work_scale" << workScale;
  report << "seam_scale" << seamScale;
  report << "compose_scale" << composeScale;
  report << "match_confidence" << kMatchConfidence;
  report << "confidence_threshold" << kConfidenceThreshold;
  report << "intra_ring_radius" << kIntraRingRadius;
  report << "cross_ring_tolerance" << kCrossRingTolerance;
  report << "ring_sizes" << "[";
  for (int size : ringSizes) {
    report << size;
  }
  report << "]";
  report << "ring_directions" << "[";
  for (int direction : ringDirections) {
    report << direction;
  }
  report << "]";
  report << "ring_pitch_signs" << "[";
  for (int sign : ringPitchSigns) {
    report << sign;
  }
  report << "]";
  report << "pose_placement"
         << (cameraSeed.find("recorded") != std::string::npos ? "recorded-fallback"
                                                              : "estimate");
  report << "lock_intrinsics" << true;
  report << "lock_shared_focal" << true;
  report << "bundle_adjust" << true;
  report << "ba_refine_intrinsics" << false;
  report << "wave_correct" << true;
  report << "normalize_world_orientation" << true;
  report << "pose_outlier_regularization" << false;
  report << "pitch_prior_weight" << kPitchPriorWeight;
  report << "learned_match_mode" << learnedStats.mode;
  report << "learned_accepted_pairs" << learnedStats.acceptedPairs;
  report << "learned_correspondences" << learnedStats.totalCorrespondences;
  report << "seam_finder" << "graphcut";
  report << "ring_seam_prior" << true;
  report << "ring_seam_overlap_fraction" << kRingSeamOverlapFraction;
  report << "blend_strength" << kBlendStrength;
  report << "periodic_blend_padding_fraction" << kPeriodicBlendPaddingFraction;
  report << "}";

  report << "camera_estimation" << "{";
  report << "seed" << cameraSeed;
  report << "fallback_reason" << cameraFallbackReason;
  report << "global_arrangement_rediscovery" << true;
  report << "orientation_flipped" << orientationFlipped;
  report << "approved_topology_pair_count" << approvedPairCount;
  report << "pitch_prior_adjustments" << "[";
  for (const PitchPriorRecord &record : pitchRecords) {
    report << "{";
    report << "index" << record.index;
    report << "ring" << record.ring;
    report << "original_pitch_degrees" << record.originalPitchDegrees;
    report << "prior_pitch_degrees" << record.priorPitchDegrees;
    report << "blended_pitch_degrees" << record.blendedPitchDegrees;
    report << "ring_delta_degrees" << record.ringDeltaDegrees;
    report << "}";
  }
  report << "]";
  report << "}";

  report << "pipeline" << "["
         << "adaptive-ring-layout"
         << "sift-feature-matching"
         << "topology-masked-pairs"
         << "homography-camera-estimate"
         << "locked-shared-intrinsics"
         << "ray-bundle-adjustment-rotations"
         << "wave-correct"
         << "normalize-world-orientation"
         << "coremotion-ring-pitch-prior"
         << "relock-intrinsics"
         << "spherical-warp"
         << "ring-seam-priors"
         << "gain-block-exposure-correction"
         << "graphcut-seam-optimization"
         << "periodic-multiband-blending"
         << "]";
  report << "exposure_correction" << seams.exposureStatus;
  report << "seam_optimization" << seams.seamStatus;
  report << "seam_contributing_frame_count" << seams.contributingFrameCount;
  report << "output" << "{";
  report << "panorama_equirectangular" << panoramaPath.string();
  report << "}";
  report << "frames" << "[";
  for (std::size_t index = 0; index < frames.size(); ++index) {
    report << "{";
    report << "sequence_index" << frames[index].input.sequenceIndex;
    report << "image" << frames[index].input.imageFilename;
    report << "ring" << ringName(frames[index].input.ring);
    report << "layout_ring" << frames[index].layout.ring;
    report << "layout_local_index" << frames[index].layout.localIndex;
    report << "layout_phase" << frames[index].layout.phase;
    report << "yaw_degrees" << frames[index].input.yawDegrees;
    report << "feature_count" << frames[index].featureCount;
    report << "intrinsics" << "{";
    report << "fx" << frames[index].intrinsics.fx;
    report << "fy" << frames[index].intrinsics.fy;
    report << "cx" << frames[index].intrinsics.cx;
    report << "cy" << frames[index].intrinsics.cy;
    report << "reference_width" << frames[index].intrinsics.referenceWidth;
    report << "reference_height" << frames[index].intrinsics.referenceHeight;
    report << "}";
    report << "}";
  }
  report << "]";
  report.release();
}

} // namespace

StitchArtifacts PanoramaEngine::stitch(const StitchRequest &request) {
  validateRequest(request);
  std::filesystem::create_directories(request.outputDirectory);
  const int64 started = cv::getTickCount();

  std::vector<PreparedFrame> frames = buildAdaptiveLayout(request.frames);


  std::vector<int> ringSizes;
  std::vector<int> ringDirections;
  std::vector<int> ringPitchSigns;
  for (const PreparedFrame &frame : frames) {
    if (ringSizes.size() == static_cast<std::size_t>(frame.layout.ring)) {
      ringSizes.push_back(frame.layout.ringSize);
      ringDirections.push_back(frame.layout.direction);
      ringPitchSigns.push_back(pitchSignForRing(frame.input.ring));
    }
  }

  cv::Mat fullProbe = loadOrientedImage(frames.front().input);
  const double workScale = megapixelScale(fullProbe.size(), kWorkMegapixels);
  const double seamScale = megapixelScale(fullProbe.size(), kSeamMegapixels);
  const double composeScale =
      maximumDimensionScale(fullProbe.size(), kComposeSourceMaximumDimension);

  std::vector<ImageFeatures> features(frames.size());
  cv::Ptr<cv::SIFT> featureFinder = cv::SIFT::create(
      kMaximumFeatures, 3, kSiftContrastThreshold, 15, 1.6);

  for (std::size_t index = 0; index < frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(frames[index].input);
    frames[index].intrinsics =
        intrinsicsAdjustedToDecodedSize(frames[index].input, fullImage.size());
    frames[index].workScale = workScale;
    cv::Mat workImage = resizedImage(fullImage, workScale);
    cv::detail::computeImageFeatures(featureFinder, workImage, features[index]);
    features[index].img_idx = static_cast<int>(index);
    frames[index].featureCount =
        static_cast<int>(features[index].keypoints.size());
  }

  int approvedPairCount = 0;
  cv::Mat topologyMask = buildMatchMask(frames, approvedPairCount);
  cv::Ptr<cv::detail::BestOf2NearestMatcher> matcher =
      cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, kMatchConfidence, 4,
                                                     4);
  std::vector<MatchesInfo> pairwiseMatches;
  (*matcher)(features, pairwiseMatches, topologyMask.getUMat(cv::ACCESS_READ));
  matcher->collectGarbage();

  const LearnedMatchStats learnedStats = applyLoFTRMatchCache(
      features, pairwiseMatches, frames, topologyMask,
      request.learnedMatchCacheDirectory);

  // Prefer LoFTR-augmented SIFT graph. Fall back to CoreMotion poses only if
  // the match graph still cannot support a full estimate.

  std::string cameraSeed = "homography-estimator";
  std::string cameraFallbackReason;
  std::vector<CameraParams> cameras;
  bool usedMatchEstimate = false;

  std::vector<int> component = cv::detail::leaveBiggestComponent(
      features, pairwiseMatches, kConfidenceThreshold);
  if (static_cast<int>(component.size()) != static_cast<int>(frames.size())) {
    cameraFallbackReason =
        "Confident match component is missing frames (" +
        std::to_string(component.size()) + "/" +
        std::to_string(frames.size()) + ")";
  } else {
    bool reordered = false;
    for (int index = 0; index < static_cast<int>(frames.size()); ++index) {
      if (component[static_cast<std::size_t>(index)] != index) {
        reordered = true;
        break;
      }
    }
    if (reordered) {
      cameraFallbackReason =
          "Confident match component reordered frames; refusing to drop inputs";
    } else {
      cv::Ptr<cv::detail::Estimator> estimator =
          cv::makePtr<cv::detail::HomographyBasedEstimator>();
      if (!(*estimator)(features, pairwiseMatches, cameras)) {
        cameraFallbackReason = "OpenCV homography-based camera estimation failed";
        cameras.clear();
      } else {
        for (CameraParams &camera : cameras) {
          cv::Mat rotation32;
          camera.R.convertTo(rotation32, CV_32F);
          camera.R = rotation32;
        }
        applyLockedIntrinsics(cameras, frames, true);

        // Keep a copy: if ray BA fails we must NOT throw away the LoFTR-backed
        // homography cameras and fall back to CoreMotion (that path looks broken).
        std::vector<CameraParams> camerasBeforeBA = cameras;

        cv::Ptr<cv::detail::BundleAdjusterRay> adjuster =
            cv::makePtr<cv::detail::BundleAdjusterRay>();
        adjuster->setConfThresh(kConfidenceThreshold);
        adjuster->setRefinementMask(cv::Mat::zeros(3, 3, CV_8U));
        if ((*adjuster)(features, pairwiseMatches, cameras)) {
          applyLockedIntrinsics(cameras, frames, true);
          usedMatchEstimate = true;
          cameraSeed = "homography-estimator+ray-ba";
        } else {
          // Retry BA with a softer confidence gate before giving up on matches.
          cameras = camerasBeforeBA;
          adjuster->setConfThresh(std::max(0.05f, kConfidenceThreshold * 0.5f));
          if ((*adjuster)(features, pairwiseMatches, cameras)) {
            applyLockedIntrinsics(cameras, frames, true);
            usedMatchEstimate = true;
            cameraSeed = "homography-estimator+ray-ba-soft";
            cameraFallbackReason =
                "Ray BA required a softer confidence threshold after LoFTR augment";
          } else {
            cameras = camerasBeforeBA;
            applyLockedIntrinsics(cameras, frames, true);
            usedMatchEstimate = true;
            cameraSeed = "homography-estimator-no-ba";
            cameraFallbackReason =
                "OpenCV ray bundle adjustment failed; kept LoFTR/homography cameras";
          }
        }
      }
    }
  }

  if (!usedMatchEstimate) {
    if (cameraFallbackReason.empty()) {
      cameraFallbackReason = "match-based camera estimate unavailable";
    }
    cameraSeed = "recorded-device-pose-fallback";
    cameras = camerasFromRecordedPoses(frames);
  }


  std::vector<cv::Mat> waveRotations;
  waveRotations.reserve(cameras.size());
  for (const CameraParams &camera : cameras) {
    waveRotations.push_back(camera.R.clone());
  }
  cv::detail::waveCorrect(waveRotations, cv::detail::WAVE_CORRECT_HORIZ);
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    cameras[index].R = waveRotations[index];
  }

  const bool orientationFlipped =
      normalizeWorldOrientation(cameras, frames, ringPitchSigns);

  // Pitch prior is most useful after match-based estimate. Recorded-pose
  // fallback already comes from CoreMotion, so keep weight but allow a light
  // blend for gauge consistency after wave correction.
  const std::vector<PitchPriorRecord> pitchRecords =
      pullRingPitchesFromMetadata(cameras, frames, kPitchPriorWeight);
  applyLockedIntrinsics(cameras, frames, true);

  std::vector<double> focals;
  focals.reserve(cameras.size());
  for (const CameraParams &camera : cameras) {
    focals.push_back(camera.focal);
  }
  const float warperScaleWork = static_cast<float>(medianOf(focals));

  features.clear();
  pairwiseMatches.clear();

  SeamProducts seams = buildSeamsAndExposure(frames, cameras, warperScaleWork,
                                             workScale, seamScale);
  const std::filesystem::path panoramaPath =
      composePanorama(frames, cameras, seams, warperScaleWork, workScale,
                      composeScale, request.outputDirectory);
  const std::filesystem::path reportPath =
      request.outputDirectory / "report.json";
  const double elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  writeReport(frames, ringSizes, ringDirections, ringPitchSigns,
              approvedPairCount, orientationFlipped, cameraSeed,
              cameraFallbackReason, learnedStats, pitchRecords, seams,
              panoramaPath, reportPath, elapsedSeconds, workScale, seamScale,
              composeScale);
  return StitchArtifacts{panoramaPath, reportPath};
}

} // namespace sphera
