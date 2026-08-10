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
#include <opencv2/stitching/detail/warpers.hpp>
#pragma clang diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <stdexcept>

namespace sphera {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWorkMegapixels = 0.60;
constexpr double kSeamMegapixels = 0.10;
constexpr int kComposeSourceMaximumDimension = 2048;
constexpr int kMaximumFeatures = 5000;
constexpr double kMatchGraphConfidence = 0.10;

using cv::detail::CameraParams;
using cv::detail::ImageFeatures;
using cv::detail::MatchesInfo;

struct FrameGeometry {
  CameraIntrinsics intrinsics;
  double workScale = 1;
  cv::Mat priorRotation;
  int featureCount = 0;
  double appliedPoseCorrectionDegrees = 0;
};

struct RefinementSummary {
  bool attempted = false;
  bool applied = false;
  int approvedPairCount = 0;
  int confidentPairCount = 0;
  int rejectedOutlierCount = 0;
  std::string status = "pose-priors-only";
};

struct SeamProducts {
  std::vector<cv::UMat> masks;
  cv::Ptr<cv::detail::ExposureCompensator> compensator;
  int contributingFrameCount = 0;
  std::string exposureStatus;
  std::string seamStatus;
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
  if (request.outputWidth < 1024 || request.outputWidth > 8192 ||
      request.outputWidth % 2 != 0) {
    throw std::runtime_error(
        "The panorama output width must be an even value from 1024 to 8192");
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

cv::Mat openCVRotationFromCapturePose(const FrameInput &frame) {
  cv::Mat captureRotation(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      captureRotation.at<double>(row, column) =
          frame.cameraToCaptureReferenceRotation[static_cast<std::size_t>(
              row * 3 + column)];
    }
  }

  // Capture coordinates use +Y for gravity-up and -Z for the initial heading.
  // OpenCV's spherical projection uses +Y toward image-down and +Z at the
  // equirectangular center. A pi rotation about X changes basis without a
  // reflection and maps the session's first heading to panorama longitude 0.
  const cv::Matx33d captureToOpenCVValues(1, 0, 0, 0, -1, 0, 0, 0, -1);
  const cv::Mat captureToOpenCV(captureToOpenCVValues, true);
  return orthonormalizedRotation(captureToOpenCV * captureRotation);
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
  return applyExifOrientation(encoded, frame.exifOrientation);
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
  if (std::abs(scale - 1.0) < 0.0001) {
    return source;
  }
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(), scale, scale, cv::INTER_AREA);
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

cv::Mat intrinsicMatrix(const CameraIntrinsics &intrinsics, double imageScale) {
  const cv::Matx33f values(static_cast<float>(intrinsics.fx * imageScale), 0,
                           static_cast<float>(intrinsics.cx * imageScale), 0,
                           static_cast<float>(intrinsics.fy * imageScale),
                           static_cast<float>(intrinsics.cy * imageScale), 0, 0,
                           1);
  return cv::Mat(values, true);
}

CameraParams cameraAtWorkScale(const FrameGeometry &geometry) {
  CameraParams camera;
  camera.focal = geometry.intrinsics.fx * geometry.workScale;
  camera.aspect = geometry.intrinsics.fy / geometry.intrinsics.fx;
  camera.ppx = geometry.intrinsics.cx * geometry.workScale;
  camera.ppy = geometry.intrinsics.cy * geometry.workScale;
  geometry.priorRotation.convertTo(camera.R, CV_32F);
  camera.t = cv::Mat::zeros(3, 1, CV_64F);
  return camera;
}

cv::Vec3d opticalDirection(const cv::Mat &rotation) {
  cv::Mat rotation64;
  rotation.convertTo(rotation64, CV_64F);
  cv::Vec3d direction(rotation64.at<double>(0, 2), rotation64.at<double>(1, 2),
                      rotation64.at<double>(2, 2));
  const double length = cv::norm(direction);
  return length > 0 ? direction / length : cv::Vec3d(0, 0, 1);
}

double angularSeparationDegrees(const cv::Mat &left, const cv::Mat &right) {
  const cv::Vec3d leftDirection = opticalDirection(left);
  const cv::Vec3d rightDirection = opticalDirection(right);
  const double cosine =
      std::clamp(leftDirection.dot(rightDirection), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / kPi;
}

bool areCircularNeighbors(const FrameInput &left, const FrameInput &right) {
  if (left.ring != right.ring || left.ringCount <= 1 ||
      right.ringCount != left.ringCount) {
    return false;
  }
  const int direct = std::abs(left.ringIndex - right.ringIndex);
  const int circular = std::min(direct, left.ringCount - direct);
  return circular == 1;
}

bool isUpperLowerPair(const FrameInput &left, const FrameInput &right) {
  return (left.ring == CaptureRing::upward &&
          right.ring == CaptureRing::downward) ||
         (left.ring == CaptureRing::downward &&
          right.ring == CaptureRing::upward);
}

cv::Mat makeTopologyMask(const StitchRequest &request,
                         const std::vector<FrameGeometry> &geometry,
                         int &approvedPairCount) {
  const int count = static_cast<int>(request.frames.size());
  cv::Mat mask = cv::Mat::zeros(count, count, CV_8U);
  approvedPairCount = 0;
  for (int left = 0; left < count; ++left) {
    for (int right = left + 1; right < count; ++right) {
      const bool sameRingNeighbor =
          areCircularNeighbors(request.frames[left], request.frames[right]);
      const bool adjacentRingOverlap =
          request.frames[left].ring != request.frames[right].ring &&
          !isUpperLowerPair(request.frames[left], request.frames[right]) &&
          angularSeparationDegrees(geometry[left].priorRotation,
                                   geometry[right].priorRotation) <= 92.0;
      if (sameRingNeighbor || adjacentRingOverlap) {
        mask.at<uchar>(left, right) = 255;
        mask.at<uchar>(right, left) = 255;
        ++approvedPairCount;
      }
    }
  }
  return mask;
}

bool confidentMatchGraphIsConnected(const std::vector<MatchesInfo> &matches,
                                    int frameCount, int &confidentPairCount) {
  std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(frameCount));
  confidentPairCount = 0;
  for (int left = 0; left < frameCount; ++left) {
    for (int right = left + 1; right < frameCount; ++right) {
      const MatchesInfo &match =
          matches[static_cast<std::size_t>(left * frameCount + right)];
      if (match.num_inliers >= 8 && match.confidence >= kMatchGraphConfidence) {
        adjacency[left].push_back(right);
        adjacency[right].push_back(left);
        ++confidentPairCount;
      }
    }
  }

  std::vector<bool> visited(static_cast<std::size_t>(frameCount), false);
  std::queue<int> pending;
  visited[0] = true;
  pending.push(0);
  while (!pending.empty()) {
    const int current = pending.front();
    pending.pop();
    for (int neighbor : adjacency[current]) {
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        pending.push(neighbor);
      }
    }
  }
  return std::all_of(visited.begin(), visited.end(),
                     [](bool value) { return value; });
}

cv::Mat clampRotationToPrior(const cv::Mat &candidate, const cv::Mat &prior,
                             double maximumRadians, double &appliedDegrees) {
  cv::Mat candidateRotation = orthonormalizedRotation(candidate);
  cv::Mat priorRotation = orthonormalizedRotation(prior);
  cv::Mat delta =
      orthonormalizedRotation(candidateRotation * priorRotation.t());
  cv::Mat rotationVector;
  cv::Rodrigues(delta, rotationVector);
  double angle = cv::norm(rotationVector);
  if (!std::isfinite(angle)) {
    appliedDegrees = 0;
    return priorRotation;
  }
  if (angle > maximumRadians && angle > 0) {
    rotationVector *= maximumRadians / angle;
    angle = maximumRadians;
  }
  cv::Mat boundedDelta;
  cv::Rodrigues(rotationVector, boundedDelta);
  appliedDegrees = angle * 180.0 / kPi;
  return orthonormalizedRotation(boundedDelta * priorRotation);
}

std::vector<cv::Mat>
refinePoses(const StitchRequest &request, std::vector<FrameGeometry> &geometry,
            const std::vector<ImageFeatures> &features,
            const std::vector<MatchesInfo> &pairwiseMatches,
            const std::vector<CameraParams> &priorCameras,
            RefinementSummary &summary) {
  std::vector<cv::Mat> rotations;
  rotations.reserve(geometry.size());
  for (const FrameGeometry &frame : geometry) {
    rotations.push_back(frame.priorRotation.clone());
  }

  if (!confidentMatchGraphIsConnected(pairwiseMatches,
                                      static_cast<int>(geometry.size()),
                                      summary.confidentPairCount)) {
    summary.status = "skipped-disconnected-feature-graph";
    return rotations;
  }
  if (request.maximumPoseRefinementDegrees <= 0) {
    summary.status = "disabled-by-capture-configuration";
    return rotations;
  }

  summary.attempted = true;
  try {
    std::vector<CameraParams> candidates = priorCameras;
    cv::Ptr<cv::detail::BundleAdjusterRay> adjuster =
        cv::makePtr<cv::detail::BundleAdjusterRay>();
    adjuster->setConfThresh(kMatchGraphConfidence);
    adjuster->setRefinementMask(cv::Mat::zeros(3, 3, CV_8U));
    adjuster->setTermCriteria(cv::TermCriteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 80, 1e-6));
    if (!(*adjuster)(features, pairwiseMatches, candidates)) {
      summary.status = "bundle-adjustment-declined";
      return rotations;
    }

    // Bundle adjustment has a global rotational gauge. Re-anchor its solution
    // to the first recorded pose before measuring or limiting any correction.
    const cv::Mat refinedAnchor = orthonormalizedRotation(candidates[0].R);
    const cv::Mat priorAnchor =
        orthonormalizedRotation(geometry[0].priorRotation);
    const cv::Mat gaugeAlignment = priorAnchor * refinedAnchor.t();
    const double maximumRadians =
        request.maximumPoseRefinementDegrees * kPi / 180.0;
    const double rejectionThresholdRadians =
        std::max(maximumRadians * 2.0, 12.0 * kPi / 180.0);
    bool appliedAnyCorrection = false;

    for (std::size_t index = 0; index < candidates.size(); ++index) {
      const cv::Mat gaugeAligned =
          gaugeAlignment * orthonormalizedRotation(candidates[index].R);
      const cv::Mat rawDelta = orthonormalizedRotation(
          gaugeAligned * geometry[index].priorRotation.t());
      cv::Mat rawRotationVector;
      cv::Rodrigues(rawDelta, rawRotationVector);
      const double rawCorrection = cv::norm(rawRotationVector);
      if (!std::isfinite(rawCorrection) ||
          rawCorrection > rejectionThresholdRadians) {
        ++summary.rejectedOutlierCount;
        continue;
      }
      rotations[index] = clampRotationToPrior(
          gaugeAligned, geometry[index].priorRotation, maximumRadians,
          geometry[index].appliedPoseCorrectionDegrees);
      appliedAnyCorrection =
          appliedAnyCorrection ||
          geometry[index].appliedPoseCorrectionDegrees > 0.0001;
    }
    summary.applied = appliedAnyCorrection;
    if (!appliedAnyCorrection && summary.rejectedOutlierCount > 0) {
      summary.status = "bundle-adjustment-outliers-rejected";
    } else if (summary.rejectedOutlierCount > 0) {
      summary.status =
          "bounded-pose-refinement-applied-with-outlier-rejections";
    } else {
      summary.status = "bounded-pose-refinement-applied";
    }
  } catch (const cv::Exception &) {
    summary.status = "bundle-adjustment-failed-using-pose-priors";
  }
  return rotations;
}

void applyRingBandPrior(cv::UMat &mask, cv::Point corner, CaptureRing ring,
                        int sphereHeight) {
  double minimumFraction = 0;
  double maximumFraction = 1;
  switch (ring) {
  case CaptureRing::upward:
    maximumFraction = 0.65;
    break;
  case CaptureRing::horizontal:
    minimumFraction = 0.18;
    maximumFraction = 0.82;
    break;
  case CaptureRing::downward:
    minimumFraction = 0.35;
    break;
  }

  cv::Mat writable = mask.getMat(cv::ACCESS_RW);
  const int minimumY =
      static_cast<int>(std::floor(minimumFraction * sphereHeight));
  const int maximumY =
      static_cast<int>(std::ceil(maximumFraction * sphereHeight));
  for (int row = 0; row < writable.rows; ++row) {
    const int globalY = corner.y + row;
    if (globalY < minimumY || globalY >= maximumY || globalY < 0 ||
        globalY >= sphereHeight) {
      writable.row(row).setTo(0);
    }
  }
}

SeamProducts buildSeamsAndExposure(const StitchRequest &request,
                                   const std::vector<FrameGeometry> &geometry,
                                   const std::vector<cv::Mat> &rotations) {
  const int frameCount = static_cast<int>(request.frames.size());
  constexpr int seamSphereWidth = 1024;
  constexpr int seamSphereHeight = seamSphereWidth / 2;
  const float sphereScale = static_cast<float>(seamSphereWidth / (2.0 * kPi));
  cv::Ptr<cv::detail::RotationWarper> warper =
      cv::makePtr<cv::detail::SphericalWarper>(sphereScale);

  std::vector<cv::UMat> warpedImages(static_cast<std::size_t>(frameCount));
  std::vector<cv::UMat> warpedMasks(static_cast<std::size_t>(frameCount));
  std::vector<cv::Point> corners(static_cast<std::size_t>(frameCount));

  for (int index = 0; index < frameCount; ++index) {
    cv::Mat fullImage = loadOrientedImage(request.frames[index]);
    const double seamImageScale =
        megapixelScale(fullImage.size(), kSeamMegapixels);
    cv::Mat seamImage = resizedImage(fullImage, seamImageScale);
    cv::Mat sourceMask(seamImage.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic =
        intrinsicMatrix(geometry[index].intrinsics, seamImageScale);
    cv::Mat rotation32;
    rotations[index].convertTo(rotation32, CV_32F);

    corners[index] =
        warper->warp(seamImage, intrinsic, rotation32, cv::INTER_LINEAR,
                     cv::BORDER_REFLECT, warpedImages[index]);
    warper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT, warpedMasks[index]);
    applyRingBandPrior(warpedMasks[index], corners[index],
                       request.frames[index].ring, seamSphereHeight);
  }

  SeamProducts products;
  products.masks.reserve(warpedMasks.size());
  for (const cv::UMat &mask : warpedMasks) {
    products.masks.push_back(mask.clone());
  }

  products.compensator = cv::detail::ExposureCompensator::createDefault(
      cv::detail::ExposureCompensator::GAIN_BLOCKS);
  try {
    products.compensator->feed(corners, warpedImages, warpedMasks);
    products.exposureStatus = "gain-blocks";
  } catch (const cv::Exception &) {
    products.compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
    products.exposureStatus = "disabled-after-open-cv-error";
  }

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

std::filesystem::path composePanorama(
    const StitchRequest &request, const std::vector<FrameGeometry> &geometry,
    const std::vector<cv::Mat> &rotations, const SeamProducts &seams) {
  const int sphereWidth = request.outputWidth;
  const int sphereHeight = sphereWidth / 2;
  const int sphereLeft = -sphereWidth / 2;
  const int periodicPadding = std::max(64, sphereWidth / 32);
  const cv::Rect paddedDestination(sphereLeft - periodicPadding, 0,
                                   sphereWidth + periodicPadding * 2,
                                   sphereHeight);
  const float sphereScale = static_cast<float>(sphereWidth / (2.0 * kPi));
  cv::Ptr<cv::detail::RotationWarper> warper =
      cv::makePtr<cv::detail::SphericalWarper>(sphereScale);

  cv::detail::MultiBandBlender blender(false, 5, CV_32F);
  blender.setNumBands(5);
  blender.prepare(paddedDestination);
  int feedCount = 0;

  for (std::size_t index = 0; index < request.frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(request.frames[index]);
    const double imageScale =
        maximumDimensionScale(fullImage.size(), kComposeSourceMaximumDimension);
    cv::Mat image = resizedImage(fullImage, imageScale);
    cv::Mat sourceMask(image.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic =
        intrinsicMatrix(geometry[index].intrinsics, imageScale);
    cv::Mat rotation32;
    rotations[index].convertTo(rotation32, CV_32F);

    cv::Mat warpedImage;
    cv::Mat warpedMask;
    const cv::Point corner =
        warper->warp(image, intrinsic, rotation32, cv::INTER_LINEAR,
                     cv::BORDER_REFLECT, warpedImage);
    warper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT, warpedMask);
    seams.compensator->apply(static_cast<int>(index), corner, warpedImage,
                             warpedMask);

    cv::Mat dilatedSeam;
    cv::dilate(seams.masks[index].getMat(cv::ACCESS_READ), dilatedSeam,
               cv::Mat(), cv::Point(-1, -1), 1);
    cv::Mat transferredSeam;
    cv::resize(dilatedSeam, transferredSeam, warpedMask.size(), 0, 0,
               cv::INTER_LINEAR);
    cv::threshold(transferredSeam, transferredSeam, 0, 255, cv::THRESH_BINARY);
    cv::Mat selectedMask;
    cv::bitwise_and(warpedMask, transferredSeam, selectedMask);
    if (cv::countNonZero(selectedMask) == 0) {
      continue;
    }

    cv::Mat warped16;
    warpedImage.convertTo(warped16, CV_16S);
    if (feedPeriodicImage(blender, warped16, selectedMask, corner,
                          paddedDestination, sphereWidth)) {
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

  const cv::Rect centralRegion(
      periodicPadding, 0, std::min(sphereWidth, blended.cols - periodicPadding),
      std::min(sphereHeight, blended.rows));
  if (centralRegion.width != sphereWidth ||
      centralRegion.height != sphereHeight) {
    throw std::runtime_error(
        "The blended panorama did not have the requested 2:1 dimensions");
  }

  cv::Mat panorama16 = blended(centralRegion).clone();
  cv::Mat panoramaMask = blendedMask(centralRegion).clone();
  cv::Mat panorama;
  panorama16.convertTo(panorama, CV_8U);
  panorama.setTo(cv::Scalar::all(0), panoramaMask == 0);
  makeLongitudeBoundaryContinuous(panorama, panoramaMask);

  const std::filesystem::path panoramaPath =
      request.outputDirectory / "panorama_equirectangular.jpg";
  if (!cv::imwrite(panoramaPath.string(), panorama,
                   {cv::IMWRITE_JPEG_QUALITY, 95})) {
    throw std::runtime_error("Could not write the equirectangular panorama");
  }
  return panoramaPath;
}

void writeReport(const StitchRequest &request,
                 const std::vector<FrameGeometry> &geometry,
                 const RefinementSummary &refinement, const SeamProducts &seams,
                 const std::filesystem::path &panoramaPath,
                 const std::filesystem::path &reportPath,
                 double elapsedSeconds) {
  cv::FileStorage report(reportPath.string(),
                         cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  if (!report.isOpened()) {
    throw std::runtime_error("Could not create the native engine report");
  }

  double maximumAppliedCorrection = 0;
  for (const FrameGeometry &frame : geometry) {
    maximumAppliedCorrection =
        std::max(maximumAppliedCorrection, frame.appliedPoseCorrectionDegrees);
  }

  report << "engine" << "sphera-ios-native";
  report << "engine_contract_version" << 1;
  report << "opencv_version" << CV_VERSION;
  report << "status" << "success";
  report << "elapsed_seconds" << elapsedSeconds;
  report << "initial_layout" << "{";
  report << "source" << "frames[].pose.cameraToCaptureReferenceRotationMatrix";
  report << "global_arrangement_rediscovery" << false;
  report << "coordinate_conversion"
         << "capture gravity-up frame to OpenCV spherical frame by pi rotation "
            "about X";
  report << "}";
  report << "pose_refinement" << "{";
  report << "status" << refinement.status;
  report << "attempted" << refinement.attempted;
  report << "applied" << refinement.applied;
  report << "maximum_allowed_degrees" << request.maximumPoseRefinementDegrees;
  report << "maximum_applied_degrees" << maximumAppliedCorrection;
  report << "approved_topology_pair_count" << refinement.approvedPairCount;
  report << "confident_pair_count" << refinement.confidentPairCount;
  report << "rejected_outlier_count" << refinement.rejectedOutlierCount;
  report << "}";
  report << "pipeline" << "["
         << "pose-initialized-layout"
         << "sift-feature-matching"
         << "topology-masked-edge-alignment"
         << "bounded-ray-bundle-adjustment"
         << "spherical-warp"
         << "gain-block-exposure-correction"
         << "graphcut-seam-optimization"
         << "periodic-multiband-blending"
         << "]";
  report << "exposure_correction" << seams.exposureStatus;
  report << "seam_optimization" << seams.seamStatus;
  report << "seam_contributing_frame_count" << seams.contributingFrameCount;
  report << "output" << "{";
  report << "panorama_equirectangular" << panoramaPath.string();
  report << "width" << request.outputWidth;
  report << "height" << request.outputWidth / 2;
  report << "}";
  report << "frames" << "[";
  for (std::size_t index = 0; index < request.frames.size(); ++index) {
    report << "{";
    report << "sequence_index" << request.frames[index].sequenceIndex;
    report << "image" << request.frames[index].imageFilename;
    report << "ring" << ringName(request.frames[index].ring);
    report << "feature_count" << geometry[index].featureCount;
    report << "pose_correction_degrees"
           << geometry[index].appliedPoseCorrectionDegrees;
    report << "intrinsics" << "{";
    report << "fx" << geometry[index].intrinsics.fx;
    report << "fy" << geometry[index].intrinsics.fy;
    report << "cx" << geometry[index].intrinsics.cx;
    report << "cy" << geometry[index].intrinsics.cy;
    report << "reference_width" << geometry[index].intrinsics.referenceWidth;
    report << "reference_height" << geometry[index].intrinsics.referenceHeight;
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

  std::vector<FrameGeometry> geometry(request.frames.size());
  std::vector<ImageFeatures> features(request.frames.size());
  std::vector<CameraParams> priorCameras;
  priorCameras.reserve(request.frames.size());
  cv::Ptr<cv::SIFT> featureFinder =
      cv::SIFT::create(kMaximumFeatures, 3, 0.006, 15, 1.6);

  for (std::size_t index = 0; index < request.frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(request.frames[index]);
    geometry[index].intrinsics = intrinsicsAdjustedToDecodedSize(
        request.frames[index], fullImage.size());
    geometry[index].workScale =
        megapixelScale(fullImage.size(), kWorkMegapixels);
    geometry[index].priorRotation =
        openCVRotationFromCapturePose(request.frames[index]);

    cv::Mat workImage = resizedImage(fullImage, geometry[index].workScale);
    cv::detail::computeImageFeatures(featureFinder, workImage, features[index]);
    features[index].img_idx = static_cast<int>(index);
    geometry[index].featureCount =
        static_cast<int>(features[index].keypoints.size());
    priorCameras.push_back(cameraAtWorkScale(geometry[index]));
  }

  RefinementSummary refinement;
  cv::Mat topologyMask =
      makeTopologyMask(request, geometry, refinement.approvedPairCount);
  cv::Ptr<cv::detail::BestOf2NearestMatcher> matcher =
      cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, 0.3f, 6, 6);
  std::vector<MatchesInfo> pairwiseMatches;
  (*matcher)(features, pairwiseMatches, topologyMask.getUMat(cv::ACCESS_READ));
  matcher->collectGarbage();

  std::vector<cv::Mat> rotations = refinePoses(
      request, geometry, features, pairwiseMatches, priorCameras, refinement);

  // Feature descriptors are the largest retained work-stage allocation. They
  // are no longer needed once bounded camera refinement is finished.
  features.clear();
  pairwiseMatches.clear();
  priorCameras.clear();

  SeamProducts seams = buildSeamsAndExposure(request, geometry, rotations);
  const std::filesystem::path panoramaPath =
      composePanorama(request, geometry, rotations, seams);
  const std::filesystem::path reportPath =
      request.outputDirectory / "report.json";
  const double elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  writeReport(request, geometry, refinement, seams, panoramaPath, reportPath,
              elapsedSeconds);
  return StitchArtifacts{panoramaPath, reportPath};
}

} // namespace sphera
