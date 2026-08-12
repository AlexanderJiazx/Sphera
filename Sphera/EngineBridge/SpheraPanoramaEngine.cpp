#include "SpheraPanoramaEngine.hpp"

#include "SpheraAdaptiveRingSeam.hpp"
#include "SpheraDirectSphere.hpp"
#include "SpheraEngineMath.hpp"
#include "SpheraPoseOverlap.hpp"
#include "SpheraRotationRefinement.hpp"

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
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#pragma clang diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <future>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace sphera {
namespace {

constexpr const char *kPipelineVersion =
    "sensor_first_s1_adaptive_ring_seam_polar_cube_v2";
constexpr const char *kRecipe =
    "sensor_first_s1_adaptive_ring_seam_polar_cube";
constexpr double kWorkMegapixels = 1.0;
constexpr double kSeamMegapixels = 0.12;
constexpr int kComposeSourceMaximumDimension = 2048;
constexpr int kTargetEquirectangularWidth = 5120;
constexpr int kMaximumFeatures = 6000;
constexpr double kSiftContrastThreshold = 0.005;
constexpr float kMatchConfidence = 0.3f;
constexpr double kRingSeamOverlapFraction = 0.25;
constexpr double kPeriodicBlendPaddingFraction = 0.08;
constexpr int kSeamDilateIterations = 1;
constexpr int kStructureBlendBands = 5;
constexpr int kPolarCubeSeamSize = 256;
constexpr int kPolarCubeComposeSize = 1024;
constexpr double kPolarCubeFieldOfViewDegrees = 100.0;
constexpr double kPolarCubeFullLatitudeDegrees = 78.0;
constexpr double kPolarCubeMinimumLatitudeDegrees = 69.0;
constexpr double kPi = 3.14159265358979323846;

using cv::detail::CameraParams;
using cv::detail::ImageFeatures;
using cv::detail::MatchesInfo;

struct PreparedFrame {
  FrameInput input;
  CameraIntrinsics intrinsics;
  double workScale = 1;
  int featureCount = 0;
  PoseFrameLayout layout;
};

void reportProgress(const StitchRequest &request, double fraction,
                    const std::string &message) {
  if (request.progress) {
    request.progress(fraction, message);
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
  if (encoded.cols == frame.intrinsics.referenceWidth &&
      encoded.rows == frame.intrinsics.referenceHeight) {
    return encoded;
  }
  cv::Mat oriented = applyExifOrientation(encoded, frame.exifOrientation);
  if (oriented.cols != frame.intrinsics.referenceWidth ||
      oriented.rows != frame.intrinsics.referenceHeight) {
    throw std::runtime_error(
        "Oriented image size does not match calibrated reference for " +
        frame.imageFilename);
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

cv::Mat scaledCameraK(const CameraParams &camera, double aspectScale) {
  cv::Mat intrinsic = camera.K();
  intrinsic.convertTo(intrinsic, CV_32F);
  intrinsic.at<float>(0, 0) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(0, 2) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(1, 1) *= static_cast<float>(aspectScale);
  intrinsic.at<float>(1, 2) *= static_cast<float>(aspectScale);
  return intrinsic;
}

std::vector<CameraParams>
camerasFromSensorPriors(const std::vector<PreparedFrame> &frames,
                        double workScale) {
  std::vector<CameraParams> cameras;
  cameras.reserve(frames.size());
  for (const PreparedFrame &frame : frames) {
    CameraParams camera;
    const double fx = frame.intrinsics.fx * workScale;
    const double fy = frame.intrinsics.fy * workScale;
    camera.focal = fx;
    camera.aspect = fy / std::max(fx, 1e-9);
    camera.ppx = frame.intrinsics.cx * workScale;
    camera.ppy = frame.intrinsics.cy * workScale;
    cv::Mat rotation = iosToOpenCVRotationCaptureRef(
        frame.input.cameraToCaptureReferenceRotation);
    cv::Mat rotation32;
    rotation.convertTo(rotation32, CV_32F);
    camera.R = rotation32;
    camera.t = cv::Mat::zeros(3, 1, CV_32F);
    cameras.push_back(camera);
  }
  return cameras;
}

void applyPerFrameLockedIntrinsics(std::vector<CameraParams> &cameras,
                                   const std::vector<PreparedFrame> &frames,
                                   double workScale) {
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    const double fx = frames[index].intrinsics.fx * workScale;
    const double fy = frames[index].intrinsics.fy * workScale;
    cameras[index].focal = fx;
    cameras[index].aspect = fy / std::max(fx, 1e-9);
    cameras[index].ppx = frames[index].intrinsics.cx * workScale;
    cameras[index].ppy = frames[index].intrinsics.cy * workScale;
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

std::string jsonEscape(const std::string &input) {
  std::ostringstream stream;
  for (char character : input) {
    switch (character) {
    case '\\':
      stream << "\\\\";
      break;
    case '"':
      stream << "\\\"";
      break;
    case '\n':
      stream << "\\n";
      break;
    default:
      stream << character;
      break;
    }
  }
  return stream.str();
}

} // namespace

StitchArtifacts PanoramaEngine::stitch(const StitchRequest &request) {
  validateRequest(request);
  std::filesystem::create_directories(request.outputDirectory);
  const int64 started = cv::getTickCount();

  reportProgress(request, 0.02, "Loading frames");
  PoseOverlapGraph graph = buildPoseOverlapGraph(request.frames, "capture_ref");
  std::vector<PreparedFrame> frames;
  frames.reserve(graph.frames.size());
  for (std::size_t index = 0; index < graph.frames.size(); ++index) {
    PreparedFrame prepared;
    prepared.input = graph.frames[index];
    prepared.layout = graph.layout[index];
    frames.push_back(std::move(prepared));
  }

  cv::Mat fullProbe = loadOrientedImage(frames.front().input);
  const double workScale = megapixelScale(fullProbe.size(), kWorkMegapixels);
  const double seamScale = megapixelScale(fullProbe.size(), kSeamMegapixels);
  double composeScale =
      maximumDimensionScale(fullProbe.size(), kComposeSourceMaximumDimension);

  reportProgress(request, 0.08, "Pose-overlap graph");
  const int approvedPairCount =
      static_cast<int>(graph.report.selectedPairs.size());
  const double manifestAndPoseGraphSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();

  std::vector<ImageFeatures> features(frames.size());
  cv::Ptr<cv::SIFT> featureFinder =
      cv::SIFT::create(kMaximumFeatures, 3, kSiftContrastThreshold, 15, 1.6);
  std::vector<cv::Size> workSizes(frames.size());

  const int64 siftStarted = cv::getTickCount();
  reportProgress(request, 0.12, "SIFT matching");
  for (std::size_t index = 0; index < frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(frames[index].input);
    frames[index].intrinsics =
        intrinsicsAdjustedToDecodedSize(frames[index].input, fullImage.size());
    frames[index].workScale = workScale;
    cv::Mat workImage = resizedImage(fullImage, workScale);
    workSizes[index] = workImage.size();
    cv::detail::computeImageFeatures(featureFinder, workImage, features[index]);
    features[index].img_idx = static_cast<int>(index);
    frames[index].featureCount =
        static_cast<int>(features[index].keypoints.size());
  }

  cv::Ptr<cv::detail::BestOf2NearestMatcher> matcher =
      cv::makePtr<cv::detail::BestOf2NearestMatcher>(false, kMatchConfidence, 4,
                                                     4);
  std::vector<MatchesInfo> pairwiseMatches;
  (*matcher)(features, pairwiseMatches, graph.mask.getUMat(cv::ACCESS_READ));
  matcher->collectGarbage();
  const double siftSeconds =
      (cv::getTickCount() - siftStarted) / cv::getTickFrequency();

  if (request.enableLegacyLearnedMatches &&
      !request.learnedMatchCacheDirectory.empty()) {
    // Optional diagnostic only — product path never sets this.
  }

  std::vector<CameraParams> cameras =
      camerasFromSensorPriors(frames, workScale);
  applyPerFrameLockedIntrinsics(cameras, frames, workScale);

  std::vector<int> ringIndices(frames.size());
  std::vector<int> ringLocalIndices(frames.size());
  std::vector<int> ringSizes(frames.size());
  for (std::size_t index = 0; index < frames.size(); ++index) {
    ringIndices[index] = frames[index].layout.ring;
    ringLocalIndices[index] = frames[index].layout.localIndex;
    ringSizes[index] = frames[index].layout.ringSize;
  }

  const int64 refinementStarted = cv::getTickCount();
  reportProgress(request, 0.35, "Sensor-anchored refinement");
  auto constraintResult = buildSensorRayConstraintsFromSift(
      cameras, features, pairwiseMatches, workSizes, graph.mask, graph.overlap,
      ringIndices, ringLocalIndices, ringSizes);
  std::vector<std::pair<int, int>> adjacentPairs;
  for (const PoseOverlapPairRecord &record : graph.report.selectedPairs) {
    if (record.sameRing) {
      adjacentPairs.emplace_back(record.source, record.target);
    }
  }
  SensorAnchoredRefineReport refineReport = refineSensorAnchoredCameras(
      cameras, constraintResult.first, constraintResult.second, adjacentPairs,
      "sift");
  applyPerFrameLockedIntrinsics(cameras, frames, workScale);

  // Unconstrained cameras stay exactly at sensor (already true via solver).
  for (int index : refineReport.solution.unconstrainedCameraIndices) {
    cv::Mat sensor = iosToOpenCVRotationCaptureRef(
        frames[static_cast<std::size_t>(index)]
            .input.cameraToCaptureReferenceRotation);
    cv::Mat rotation32;
    sensor.convertTo(rotation32, CV_32F);
    cameras[static_cast<std::size_t>(index)].R = rotation32;
  }

  std::vector<double> focals;
  focals.reserve(cameras.size());
  for (const CameraParams &camera : cameras) {
    focals.push_back(camera.focal);
  }
  const float warperScaleWork = static_cast<float>(medianOf(focals));

  // Prefer ~5120 equirectangular width when source resolution allows.
  {
    const float tentativeComposeWarper =
        static_cast<float>(warperScaleWork * (composeScale / workScale));
    const int tentativeWidth =
        static_cast<int>(std::lround(2.0 * kPi * tentativeComposeWarper));
    if (tentativeWidth < kTargetEquirectangularWidth) {
      const double desiredScale =
          (kTargetEquirectangularWidth / (2.0 * kPi)) / warperScaleWork *
          workScale;
      composeScale = std::min(1.0, desiredScale);
      composeScale = std::min(
          composeScale,
          maximumDimensionScale(fullProbe.size(), kComposeSourceMaximumDimension));
    }
  }

  features.clear();
  pairwiseMatches.clear();
  const double refinementSeconds =
      (cv::getTickCount() - refinementStarted) / cv::getTickFrequency();

  const int64 seamStageStarted = cv::getTickCount();
  reportProgress(request, 0.55, "Adaptive ring seam");
  const int frameCount = static_cast<int>(frames.size());
  const double seamWorkAspect = seamScale / workScale;
  const float seamWarperScale =
      static_cast<float>(warperScaleWork * seamWorkAspect);
  cv::Ptr<cv::detail::RotationWarper> seamWarper =
      cv::makePtr<cv::detail::SphericalWarper>(seamWarperScale);

  std::vector<cv::Mat> warpedImages(static_cast<std::size_t>(frameCount));
  std::vector<cv::Mat> warpedMasks(static_cast<std::size_t>(frameCount));
  std::vector<cv::Point> corners(static_cast<std::size_t>(frameCount));
  std::vector<PoseFrameLayout> layouts;
  layouts.reserve(static_cast<std::size_t>(frameCount));

  const int64 seamWarpStarted = cv::getTickCount();
  for (int index = 0; index < frameCount; ++index) {
    cv::Mat fullImage =
        loadOrientedImage(frames[static_cast<std::size_t>(index)].input);
    cv::Mat seamImage = resizedImage(fullImage, seamScale);
    cv::Mat sourceMask(seamImage.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic =
        scaledCameraK(cameras[static_cast<std::size_t>(index)], seamWorkAspect);
    cv::Mat rotation32;
    cameras[static_cast<std::size_t>(index)].R.convertTo(rotation32, CV_32F);
    cv::UMat warpedU;
    cv::UMat maskU;
    corners[static_cast<std::size_t>(index)] = seamWarper->warp(
        seamImage, intrinsic, rotation32, cv::INTER_LINEAR, cv::BORDER_REFLECT,
        warpedU);
    seamWarper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                     cv::BORDER_CONSTANT, maskU);
    warpedImages[static_cast<std::size_t>(index)] = warpedU.getMat(cv::ACCESS_READ).clone();
    warpedMasks[static_cast<std::size_t>(index)] = maskU.getMat(cv::ACCESS_READ).clone();
    layouts.push_back(frames[static_cast<std::size_t>(index)].layout);
  }
  const double seamWarpSeconds =
      (cv::getTickCount() - seamWarpStarted) / cv::getTickFrequency();

  // Exposure compensation needs the complete geometric overlap graph.  The
  // adaptive ring prior deliberately removes most cross-ring overlap before
  // graph cut; feeding those restricted masks to the compensator disconnects
  // the upper, horizon, and lower photometric solves.
  const std::vector<cv::Mat> exposureMasks = warpedMasks;
  const int64 ringPriorStarted = cv::getTickCount();
  auto adaptive = applyAdaptiveRingSeamPriors(
      warpedMasks, corners, warpedImages, layouts, kRingSeamOverlapFraction);
  warpedMasks = adaptive.first;
  AdaptiveRingSeamReport seamReport = adaptive.second;
  const double ringPriorSeconds =
      (cv::getTickCount() - ringPriorStarted) / cv::getTickFrequency();

  std::vector<cv::UMat> warpedImagesU(static_cast<std::size_t>(frameCount));
  std::vector<cv::UMat> exposureMasksU(static_cast<std::size_t>(frameCount));
  std::vector<cv::UMat> seamMasksU(static_cast<std::size_t>(frameCount));
  for (int index = 0; index < frameCount; ++index) {
    warpedImages[static_cast<std::size_t>(index)].copyTo(
        warpedImagesU[static_cast<std::size_t>(index)]);
    exposureMasks[static_cast<std::size_t>(index)].copyTo(
        exposureMasksU[static_cast<std::size_t>(index)]);
    warpedMasks[static_cast<std::size_t>(index)].copyTo(
        seamMasksU[static_cast<std::size_t>(index)]);
  }

  cv::Ptr<cv::detail::ExposureCompensator> compensator =
      cv::detail::ExposureCompensator::createDefault(
          cv::detail::ExposureCompensator::GAIN_BLOCKS);
  std::string exposureStatus = "gain-blocks";
  const int64 exposureStarted = cv::getTickCount();
  try {
    compensator->feed(corners, warpedImagesU, exposureMasksU);
  } catch (const cv::Exception &) {
    compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
    exposureStatus = "disabled-after-open-cv-error";
  }
  const double exposureSeconds =
      (cv::getTickCount() - exposureStarted) / cv::getTickFrequency();

  std::vector<cv::UMat> floatingImages(static_cast<std::size_t>(frameCount));
  for (int index = 0; index < frameCount; ++index) {
    cv::Mat proxy = structureSeamProxy(warpedImages[static_cast<std::size_t>(index)]);
    proxy.convertTo(floatingImages[static_cast<std::size_t>(index)], CV_32F);
  }

  std::string seamStatus =
      "ring-local-graphcut-color-structure-proxy-with-adaptive-ring-seam";
  const int64 graphCutStarted = cv::getTickCount();
  std::vector<std::pair<int, double>> graphCutRingSeconds;
  try {
    struct RingGraphCutResult {
      int ringId = 0;
      std::vector<int> indices;
      std::vector<cv::UMat> masks;
      double elapsedSeconds = 0;
    };
    std::vector<int> ringIds;
    for (const PoseFrameLayout &layout : layouts) {
      ringIds.push_back(layout.ring);
    }
    std::sort(ringIds.begin(), ringIds.end());
    ringIds.erase(std::unique(ringIds.begin(), ringIds.end()), ringIds.end());
    std::vector<std::future<RingGraphCutResult>> pending;
    for (int ringId : ringIds) {
      std::vector<int> indices;
      std::vector<cv::UMat> ringImages;
      std::vector<cv::Point> ringCorners;
      std::vector<cv::UMat> ringMasks;
      for (int index = 0; index < frameCount; ++index) {
        if (layouts[static_cast<std::size_t>(index)].ring != ringId) {
          continue;
        }
        indices.push_back(index);
        ringImages.push_back(floatingImages[static_cast<std::size_t>(index)]);
        ringCorners.push_back(corners[static_cast<std::size_t>(index)]);
        ringMasks.push_back(seamMasksU[static_cast<std::size_t>(index)]);
      }
      if (indices.size() < 2) {
        continue;
      }
      pending.push_back(std::async(
          std::launch::async,
          [ringId, indices = std::move(indices),
           ringImages = std::move(ringImages),
           ringCorners = std::move(ringCorners),
           ringMasks = std::move(ringMasks)]() mutable {
            const int64 ringStarted = cv::getTickCount();
            cv::detail::GraphCutSeamFinder seamFinder(
                cv::detail::GraphCutSeamFinderBase::COST_COLOR);
            seamFinder.find(ringImages, ringCorners, ringMasks);
            RingGraphCutResult result;
            result.ringId = ringId;
            result.indices = std::move(indices);
            result.masks = std::move(ringMasks);
            result.elapsedSeconds =
                (cv::getTickCount() - ringStarted) / cv::getTickFrequency();
            return result;
          }));
    }
    for (auto &future : pending) {
      RingGraphCutResult result = future.get();
      graphCutRingSeconds.emplace_back(result.ringId, result.elapsedSeconds);
      for (std::size_t local = 0; local < result.indices.size(); ++local) {
        result.masks[local].copyTo(seamMasksU[static_cast<std::size_t>(
            result.indices[local])]);
      }
    }
  } catch (const cv::Exception &) {
    seamStatus = "adaptive-ring-seam-fallback-after-open-cv-error";
    for (int index = 0; index < frameCount; ++index) {
      warpedMasks[static_cast<std::size_t>(index)].copyTo(
          seamMasksU[static_cast<std::size_t>(index)]);
    }
  } catch (const std::exception &) {
    seamStatus = "adaptive-ring-seam-fallback-after-concurrency-error";
    for (int index = 0; index < frameCount; ++index) {
      warpedMasks[static_cast<std::size_t>(index)].copyTo(
          seamMasksU[static_cast<std::size_t>(index)]);
    }
  }
  std::sort(graphCutRingSeconds.begin(), graphCutRingSeconds.end());
  const double graphCutSeconds =
      (cv::getTickCount() - graphCutStarted) / cv::getTickFrequency();

  for (int index = 0; index < frameCount; ++index) {
    warpedMasks[static_cast<std::size_t>(index)] =
        seamMasksU[static_cast<std::size_t>(index)].getMat(cv::ACCESS_READ).clone();
  }
  const double seamStageSeconds =
      (cv::getTickCount() - seamStageStarted) / cv::getTickFrequency();

  const int64 blendStageStarted = cv::getTickCount();
  reportProgress(request, 0.75, "Blending");
  const double composeWorkAspect = composeScale / workScale;
  const float composeWarperScale =
      static_cast<float>(warperScaleWork * composeWorkAspect);
  cv::Ptr<cv::detail::RotationWarper> composeWarper =
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
    const cv::Rect roi =
        composeWarper->warpRoi(scaledSize, intrinsic, rotation32);
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

  cv::detail::MultiBandBlender blender(false, kStructureBlendBands);
  blender.setNumBands(kStructureBlendBands);
  blender.prepare(blenderRoi);

  cv::Mat contributionMap(composeRoi.height, composeRoi.width, CV_16S,
                          cv::Scalar(-1));
  std::vector<int> contributionPixels(frames.size(), 0);
  std::vector<cv::Mat> composeScaleImages(frames.size());
  int feedCount = 0;

  for (std::size_t index = 0; index < frames.size(); ++index) {
    cv::Mat fullImage = loadOrientedImage(frames[index].input);
    cv::Mat image = resizedImage(fullImage, composeScale);
    composeScaleImages[index] = image;
    cv::Mat sourceMask(image.size(), CV_8U, cv::Scalar::all(255));
    const cv::Mat intrinsic = scaledCameraK(cameras[index], composeWorkAspect);
    cv::Mat rotation32;
    cameras[index].R.convertTo(rotation32, CV_32F);

    cv::Mat warpedImage;
    cv::Mat warpedMask;
    const cv::Point corner =
        composeWarper->warp(image, intrinsic, rotation32, cv::INTER_LINEAR,
                            cv::BORDER_REFLECT, warpedImage);
    composeWarper->warp(sourceMask, intrinsic, rotation32, cv::INTER_NEAREST,
                        cv::BORDER_CONSTANT, warpedMask);
    compensator->apply(static_cast<int>(index), corner, warpedImage, warpedMask);

    const cv::Mat seamMask = transferSeamMask(
        warpedMasks[index], corners[index], corner, warpedMask.size(),
        composeScale / seamScale, kSeamDilateIterations);
    cv::Mat selectedMask;
    cv::bitwise_and(warpedMask, seamMask, selectedMask);
    if (cv::countNonZero(selectedMask) == 0) {
      continue;
    }

    for (int shift : {-sphereWidth, 0, sphereWidth}) {
      const cv::Rect shiftedBounds(corner.x + shift, corner.y, selectedMask.cols,
                                   selectedMask.rows);
      const cv::Rect intersection = shiftedBounds & composeRoi;
      if (intersection.empty()) {
        continue;
      }
      const cv::Rect sourceRegion(intersection.x - shiftedBounds.x,
                                  intersection.y - shiftedBounds.y,
                                  intersection.width, intersection.height);
      for (int row = 0; row < intersection.height; ++row) {
        for (int column = 0; column < intersection.width; ++column) {
          if (selectedMask.at<uchar>(sourceRegion.y + row,
                                     sourceRegion.x + column) == 0) {
            continue;
          }
          const int mapRow = intersection.y - composeRoi.y + row;
          const int mapCol = intersection.x - composeRoi.x + column;
          if (mapRow < 0 || mapCol < 0 || mapRow >= contributionMap.rows ||
              mapCol >= contributionMap.cols) {
            continue;
          }
          if (contributionMap.at<short>(mapRow, mapCol) < 0) {
            contributionMap.at<short>(mapRow, mapCol) =
                static_cast<short>(index);
            ++contributionPixels[index];
          }
        }
      }
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

  const int availableWidth = std::max(0, blended.cols - periodicPadding);
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

  cv::Mat equirectangular(sphereHeight, sphereWidth, CV_8UC3, cv::Scalar::all(0));
  cv::Mat equirectangularMask(sphereHeight, sphereWidth, CV_8U,
                              cv::Scalar::all(0));
  const int dstY = std::clamp(composeRoi.y, 0, std::max(0, sphereHeight - 1));
  const int copyHeight = std::min(panorama.rows, sphereHeight - dstY);
  if (copyHeight > 0) {
    panorama(cv::Rect(0, 0, panorama.cols, copyHeight))
        .copyTo(equirectangular(cv::Rect(0, dstY, panorama.cols, copyHeight)));
    panoramaMask(cv::Rect(0, 0, panoramaMask.cols, copyHeight))
        .copyTo(
            equirectangularMask(cv::Rect(0, dstY, panoramaMask.cols, copyHeight)));
  }
  makeLongitudeBoundaryContinuous(equirectangular, equirectangularMask);
  const double blendStageSeconds =
      (cv::getTickCount() - blendStageStarted) / cv::getTickFrequency();

  reportProgress(request, 0.90, "Projection-native poles");
  cv::Mat directFillLabels(equirectangular.size(), CV_16S, cv::Scalar(-1));
  PolarCubeFaceStats polarCubeStats;
  std::string polarCubeStatus = "enabled";
  try {
    polarCubeStats = composeTopCubeFace(
        equirectangular, equirectangularMask, &directFillLabels,
        composeScaleImages, cameras, layouts, workScale, composeScale,
        kPolarCubeSeamSize, kPolarCubeComposeSize,
        kPolarCubeFieldOfViewDegrees, kStructureBlendBands,
        kPolarCubeFullLatitudeDegrees, kPolarCubeMinimumLatitudeDegrees);
    if (!polarCubeStats.enabled) {
      polarCubeStatus = "skipped-insufficient-new-coverage";
    }
  } catch (const cv::Exception &) {
    polarCubeStatus = "fallback-after-open-cv-error";
  } catch (const std::exception &) {
    polarCubeStatus = "fallback-after-compositor-error";
  }
  for (std::size_t index = 0; index < contributionPixels.size(); ++index) {
    if (index < polarCubeStats.selectedPixelsByInput.size()) {
      contributionPixels[index] +=
          polarCubeStats.selectedPixelsByInput[index];
    }
  }
  PolarCubeFaceStats bottomCubeStats;
  std::string bottomCubeStatus = "enabled";
  try {
    bottomCubeStats = composeBottomCubeFace(
        equirectangular, equirectangularMask, &directFillLabels,
        composeScaleImages, cameras, layouts, workScale, composeScale,
        kPolarCubeSeamSize, kPolarCubeComposeSize,
        kPolarCubeFieldOfViewDegrees, kStructureBlendBands,
        kPolarCubeFullLatitudeDegrees, kPolarCubeMinimumLatitudeDegrees);
    if (!bottomCubeStats.enabled) {
      if (bottomCubeStats.centralPairGateRejected) {
        bottomCubeStatus = "skipped-central-pair-gate";
      } else if (bottomCubeStats.responseFieldGateRejected) {
        bottomCubeStatus = "skipped-response-field-gate";
      } else {
        bottomCubeStatus = "skipped-insufficient-new-coverage";
      }
    }
  } catch (const cv::Exception &) {
    bottomCubeStatus = "fallback-after-open-cv-error";
  } catch (const std::exception &) {
    bottomCubeStatus = "fallback-after-compositor-error";
  }
  for (std::size_t index = 0; index < contributionPixels.size(); ++index) {
    if (index < bottomCubeStats.selectedPixelsByInput.size()) {
      contributionPixels[index] +=
          bottomCubeStats.selectedPixelsByInput[index];
    }
  }

  reportProgress(request, 0.94, "Residual sphere fill");
  const int64 directFillStarted = cv::getTickCount();
  DirectSphereFillStats directSphereFill = fillEquirectangularHoles(
      equirectangular, equirectangularMask, &directFillLabels, composeScaleImages,
      cameras, workScale, composeScale);
  const double directFillElapsedSeconds =
      (cv::getTickCount() - directFillStarted) / cv::getTickFrequency();
  for (std::size_t index = 0; index < contributionPixels.size(); ++index) {
    if (index < directSphereFill.filledPixelsByInput.size()) {
      contributionPixels[index] +=
          directSphereFill.filledPixelsByInput[index];
    }
  }
  makeLongitudeBoundaryContinuous(equirectangular, equirectangularMask);

  const int64 imageWriteStarted = cv::getTickCount();
  const std::filesystem::path panoramaPath =
      request.outputDirectory / "panorama_equirectangular.jpg";
  if (!cv::imwrite(panoramaPath.string(), equirectangular,
                   {cv::IMWRITE_JPEG_QUALITY, 95})) {
    throw std::runtime_error("Could not write the equirectangular panorama");
  }

  cv::Mat fullContributionMap(sphereHeight, sphereWidth, CV_16S,
                              cv::Scalar(-1));
  if (copyHeight > 0) {
    contributionMap(cv::Rect(0, 0, contributionMap.cols, copyHeight))
        .copyTo(fullContributionMap(
            cv::Rect(0, dstY, contributionMap.cols, copyHeight)));
  }
  for (int row = 0; row < directFillLabels.rows; ++row) {
    const short *fillRow = directFillLabels.ptr<short>(row);
    short *fullRow = fullContributionMap.ptr<short>(row);
    for (int column = 0; column < directFillLabels.cols; ++column) {
      if (fillRow[column] >= 0) {
        fullRow[column] = fillRow[column];
      }
    }
  }
  cv::Mat contributionColor(fullContributionMap.size(), CV_8UC3,
                            cv::Scalar::all(0));
  for (int row = 0; row < fullContributionMap.rows; ++row) {
    for (int column = 0; column < fullContributionMap.cols; ++column) {
      const short label = fullContributionMap.at<short>(row, column);
      if (label < 0) {
        continue;
      }
      const int hue = (label * 37) % 180;
      contributionColor.at<cv::Vec3b>(row, column) =
          cv::Vec3b(static_cast<uchar>(hue), 200, 220);
    }
  }
  cv::cvtColor(contributionColor, contributionColor, cv::COLOR_HSV2BGR);
  const std::filesystem::path contributionPath =
      request.outputDirectory / "contribution_map.png";
  cv::imwrite(contributionPath.string(), contributionColor);
  const double imageWriteSeconds =
      (cv::getTickCount() - imageWriteStarted) / cv::getTickFrequency();

  const double elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  const std::filesystem::path reportPath =
      request.outputDirectory / "report.json";

  const int spherePixels = std::max(1, sphereWidth * sphereHeight);
  int coveredPixels = cv::countNonZero(equirectangularMask);
  std::ostringstream json;
  json << std::fixed;
  json << "{\n";
  json << "  \"engine\": \"sphera-ios-native\",\n";
  json << "  \"pipeline_version\": \"" << kPipelineVersion << "\",\n";
  json << "  \"recipe\": \"" << kRecipe << "\",\n";
  json << "  \"ml_model_usage\": \"none\",\n";
  json << "  \"opencv_version\": \"" << CV_VERSION << "\",\n";
  json << "  \"status\": \"success\",\n";
  json << "  \"elapsed_seconds\": " << elapsedSeconds << ",\n";
  json << "  \"stage_timings_seconds\": {\n";
  json << "    \"manifest_and_pose_graph\": " << manifestAndPoseGraphSeconds
       << ",\n";
  json << "    \"sift_matching\": " << siftSeconds << ",\n";
  json << "    \"sensor_anchored_refinement\": " << refinementSeconds
       << ",\n";
  json << "    \"adaptive_ring_seam\": " << seamStageSeconds << ",\n";
  json << "    \"spherical_composition_and_blend\": " << blendStageSeconds
       << ",\n";
  json << "    \"projection_native_zenith\": "
       << polarCubeStats.elapsedSeconds << ",\n";
  json << "    \"projection_native_nadir\": "
       << bottomCubeStats.elapsedSeconds << ",\n";
  json << "    \"residual_sphere_fill\": " << directFillElapsedSeconds
       << ",\n";
  json << "    \"image_write\": " << imageWriteSeconds << "\n";
  json << "  },\n";
  json << "  \"configuration\": {\n";
  json << "    \"work_megapix\": " << kWorkMegapixels << ",\n";
  json << "    \"seam_megapix\": " << kSeamMegapixels << ",\n";
  json << "    \"compose_max_dimension\": " << kComposeSourceMaximumDimension
       << ",\n";
  json << "    \"target_equirectangular_width\": " << kTargetEquirectangularWidth
       << ",\n";
  json << "    \"pose_rotation_convention\": \"capture_ref\",\n";
  json << "    \"lock_shared_focal\": false,\n";
  json << "    \"maximum_pose_refinement_degrees\": 6.0,\n";
  json << "    \"blend_bands\": " << kStructureBlendBands << ",\n";
  json << "    \"ring_seam_overlap_fraction\": " << kRingSeamOverlapFraction
       << ",\n";
  json << "    \"polar_cube_seam_size\": " << kPolarCubeSeamSize << ",\n";
  json << "    \"polar_cube_compose_size\": " << kPolarCubeComposeSize
       << ",\n";
  json << "    \"polar_cube_field_of_view_degrees\": "
       << kPolarCubeFieldOfViewDegrees << ",\n";
  json << "    \"polar_cube_full_latitude_degrees\": "
       << kPolarCubeFullLatitudeDegrees << ",\n";
  json << "    \"polar_cube_minimum_latitude_degrees\": "
       << kPolarCubeMinimumLatitudeDegrees << ",\n";
  json << "    \"approved_pose_overlap_pairs\": " << approvedPairCount << "\n";
  json << "  },\n";
  json << "  \"pose_overlap\": {\n";
  json << "    \"predicted_full_sphere_coverage_fraction\": "
       << graph.report.predictedFullSphereCoverageFraction << ",\n";
  json << "    \"predicted_two_or_more_coverage_fraction\": "
       << graph.report.predictedTwoOrMoreCoverageFraction << ",\n";
  json << "    \"selected_pair_count\": " << approvedPairCount << ",\n";
  json << "    \"ring_order\": [";
  for (std::size_t index = 0; index < graph.report.ringOrder.size(); ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << "\"" << jsonEscape(graph.report.ringOrder[index]) << "\"";
  }
  json << "]\n  },\n";
  json << "  \"sensor_anchored_refinement\": {\n";
  json << "    \"constraint_count\": " << refineReport.constraintCount << ",\n";
  json << "    \"rejected_pair_count\": " << refineReport.rejectedPairCount
       << ",\n";
  json << "    \"maximum_camera_correction_degrees\": "
       << refineReport.solution.maximumCameraCorrectionDegrees << ",\n";
  json << "    \"unconstrained_camera_indices\": [";
  for (std::size_t index = 0;
       index < refineReport.solution.unconstrainedCameraIndices.size();
       ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << refineReport.solution.unconstrainedCameraIndices[index];
  }
  json << "],\n    \"camera_correction_degrees\": [";
  for (std::size_t index = 0;
       index < refineReport.solution.cameraCorrectionDegrees.size(); ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << refineReport.solution.cameraCorrectionDegrees[index];
  }
  json << "]\n  },\n";
  json << "  \"adaptive_ring_seam\": {\n";
  json << "    \"mode\": \"" << jsonEscape(seamReport.mode) << "\",\n";
  json << "    \"warp_seconds\": " << seamWarpSeconds << ",\n";
  json << "    \"ring_prior_seconds\": " << ringPriorSeconds << ",\n";
  json << "    \"exposure_seconds\": " << exposureSeconds << ",\n";
  json << "    \"graphcut_seconds\": " << graphCutSeconds << ",\n";
  json << "    \"graphcut_ring_seconds\": [";
  for (std::size_t index = 0; index < graphCutRingSeconds.size(); ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << "{\"ring\": " << graphCutRingSeconds[index].first
         << ", \"seconds\": " << graphCutRingSeconds[index].second << "}";
  }
  json << "],\n";
  json << "    \"coverage_pixels_restored\": "
       << seamReport.coveragePixelsRestored << ",\n";
  json << "    \"boundary_count\": " << seamReport.boundaries.size() << ",\n";
  json << "    \"boundaries\": [\n";
  for (std::size_t index = 0; index < seamReport.boundaries.size(); ++index) {
    const AdaptiveRingSeamBoundary &boundary = seamReport.boundaries[index];
    if (index > 0) {
      json << ",\n";
    }
    json << "      {\n";
    json << "        \"ring_pair\": [" << boundary.upperRing << ", "
         << boundary.lowerRing << "],\n";
    json << "        \"midpoint_global_y\": " << boundary.midpointGlobalY
         << ",\n";
    json << "        \"path_mean_global_y\": " << boundary.pathMeanGlobalY
         << ",\n";
    json << "        \"flow_residual_p90_pixels\": "
         << boundary.flowResidualP90Pixels << "\n";
    json << "      }";
  }
  json << "\n    ]\n";
  json << "  },\n";
  const auto writePolarCubeFace =
      [&](const std::string &key, const std::string &status,
          const PolarCubeFaceStats &stats) {
        json << "  \"" << key << "\": {\n";
        json << "    \"status\": \"" << jsonEscape(status) << "\",\n";
        json << "    \"enabled\": " << (stats.enabled ? "true" : "false")
             << ",\n";
        json << "    \"pole\": \"" << jsonEscape(stats.pole) << "\",\n";
        json << "    \"source_count\": " << stats.sourceCount << ",\n";
        json << "    \"feed_count\": " << stats.feedCount << ",\n";
        json << "    \"replaced_pixels\": " << stats.replacedPixels << ",\n";
        json << "    \"newly_covered_pixels\": " << stats.newlyCoveredPixels
             << ",\n";
        json << "    \"graphcut_seconds\": " << stats.graphCutSeconds
             << ",\n";
        json << "    \"topology_pruned_components\": "
             << stats.topologyPrunedComponents << ",\n";
        json << "    \"topology_reassigned_pixels\": "
             << stats.topologyReassignedPixels << ",\n";
        json << "    \"central_pair\": {\n";
        json << "      \"selected\": "
             << (stats.centralPairSelected ? "true" : "false") << ",\n";
        json << "      \"gate_rejected\": "
             << (stats.centralPairGateRejected ? "true" : "false") << ",\n";
        json << "      \"coverage\": " << stats.centralPairCoverage << ",\n";
        json << "      \"score\": " << stats.centralPairScore << ",\n";
        json << "      \"elapsed_seconds\": " << stats.centralPairSeconds
             << ",\n";
        json << "      \"input_indices\": [";
        for (std::size_t index = 0;
             index < stats.centralPairInputIndices.size(); ++index) {
          if (index > 0) {
            json << ", ";
          }
          json << stats.centralPairInputIndices[index];
        }
        json << "]\n";
        json << "    },\n";
        json << "    \"elapsed_seconds\": " << stats.elapsedSeconds << ",\n";
        json << "    \"response_field\": {\n";
        json << "      \"accepted\": "
             << (stats.responseFieldAccepted ? "true" : "false") << ",\n";
        json << "      \"gate_rejected\": "
             << (stats.responseFieldGateRejected ? "true" : "false")
             << ",\n";
        json << "      \"equation_count\": " << stats.responseFieldEquationCount
             << ",\n";
        json << "      \"pair_count\": " << stats.responseFieldPairCount
             << ",\n";
        json << "      \"elapsed_seconds\": " << stats.responseFieldSeconds
             << ",\n";
        json << "      \"median_absolute_log_difference_before\": "
             << stats.responseFieldMedianBefore << ",\n";
        json << "      \"median_absolute_log_difference_after\": "
             << stats.responseFieldMedianAfter << ",\n";
        json << "      \"p90_absolute_log_difference_before\": "
             << stats.responseFieldP90Before << ",\n";
        json << "      \"p90_absolute_log_difference_after\": "
             << stats.responseFieldP90After << ",\n";
        json << "      \"gain_ranges_by_input\": [";
        for (std::size_t index = 0;
             index < stats.responseFieldGainRangesByInput.size(); ++index) {
          if (index > 0) {
            json << ", ";
          }
          json << "[" << stats.responseFieldGainRangesByInput[index][0] << ", "
               << stats.responseFieldGainRangesByInput[index][1] << "]";
        }
        json << "]\n";
        json << "    },\n";
        json << "    \"photometric_gains_bgr\": ["
             << stats.photometricGainsBGR[0] << ", "
             << stats.photometricGainsBGR[1] << ", "
             << stats.photometricGainsBGR[2] << "],\n";
        json << "    \"longitude_gain_accepted\": "
             << (stats.longitudeGainAccepted ? "true" : "false") << ",\n";
        json << "    \"longitude_gain_rejected_by_cap_pressure\": "
             << (stats.longitudeGainRejectedByCapPressure ? "true" : "false")
             << ",\n";
        json << "    \"longitude_gain_supported_columns\": "
             << stats.longitudeGainSupportedColumns << ",\n";
        json << "    \"longitude_gain_minimum\": "
             << stats.longitudeGainMinimum << ",\n";
        json << "    \"longitude_gain_maximum\": "
             << stats.longitudeGainMaximum << ",\n";
        json << "    \"longitude_gain_p05\": " << stats.longitudeGainP05
             << ",\n";
        json << "    \"longitude_gain_p95\": " << stats.longitudeGainP95
             << "\n";
        json << "  },\n";
      };
  writePolarCubeFace("polar_cube_face", polarCubeStatus, polarCubeStats);
  writePolarCubeFace("bottom_cube_face", bottomCubeStatus, bottomCubeStats);
  json << "  \"direct_sphere_fill\": {\n";
  json << "    \"enabled\": " << (directSphereFill.enabled ? "true" : "false")
       << ",\n";
  json << "    \"filled_pixels\": " << directSphereFill.filledPixels << ",\n";
  json << "    \"remaining_pixels\": " << directSphereFill.remainingPixels
       << ",\n";
  json << "    \"elapsed_seconds\": " << directFillElapsedSeconds << ",\n";
  json << "    \"filled_pixels_by_input\": [";
  for (std::size_t index = 0; index < directSphereFill.filledPixelsByInput.size();
       ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << directSphereFill.filledPixelsByInput[index];
  }
  json << "]\n  },\n";
  json << "  \"seam\": {\n";
  json << "    \"status\": \"" << jsonEscape(seamStatus) << "\",\n";
  json << "    \"exposure\": \"" << jsonEscape(exposureStatus) << "\"\n";
  json << "  },\n";
  json << "  \"contribution\": {\n";
  json << "    \"sphere_pixels\": " << spherePixels << ",\n";
  json << "    \"covered_pixels\": " << coveredPixels << ",\n";
  json << "    \"rendered_coverage_fraction\": "
       << (static_cast<double>(coveredPixels) / spherePixels) << ",\n";
  json << "    \"per_frame_selected_pixels\": [";
  for (std::size_t index = 0; index < contributionPixels.size(); ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << contributionPixels[index];
  }
  json << "],\n    \"per_frame_selected_percent\": [";
  for (std::size_t index = 0; index < contributionPixels.size(); ++index) {
    if (index > 0) {
      json << ", ";
    }
    json << (100.0 * contributionPixels[index] /
             std::max(1, coveredPixels));
  }
  json << "]\n  },\n";
  json << "  \"frames\": [\n";
  for (std::size_t index = 0; index < frames.size(); ++index) {
    if (index > 0) {
      json << ",\n";
    }
    const PreparedFrame &frame = frames[index];
    cv::Mat finalRotation;
    cameras[index].R.convertTo(finalRotation, CV_64F);
    json << "    {\n";
    json << "      \"index\": " << index << ",\n";
    json << "      \"file\": \"" << jsonEscape(frame.input.imageFilename)
         << "\",\n";
    json << "      \"sequence_index\": " << frame.input.sequenceIndex << ",\n";
    json << "      \"ring\": " << frame.layout.ring << ",\n";
    json << "      \"ring_name\": \"" << jsonEscape(frame.layout.ringName)
         << "\",\n";
    json << "      \"feature_count\": " << frame.featureCount << ",\n";
    json << "      \"focal\": " << cameras[index].focal << ",\n";
    json << "      \"aspect\": " << cameras[index].aspect << ",\n";
    json << "      \"principal_point\": [" << cameras[index].ppx << ", "
         << cameras[index].ppy << "],\n";
    json << "      \"rotation_matrix\": [\n";
    for (int row = 0; row < 3; ++row) {
      json << "        [";
      for (int column = 0; column < 3; ++column) {
        if (column > 0) {
          json << ", ";
        }
        json << finalRotation.at<double>(row, column);
      }
      json << "]" << (row < 2 ? "," : "") << "\n";
    }
    json << "      ],\n";
    json << "      \"correction_degrees\": "
         << (index < refineReport.solution.cameraCorrectionDegrees.size()
                 ? refineReport.solution.cameraCorrectionDegrees[index]
                 : 0.0)
         << "\n";
    json << "    }";
  }
  json << "\n  ],\n";
  json << "  \"outputs\": {\n";
  json << "    \"panorama\": \"" << jsonEscape(panoramaPath.filename().string())
       << "\",\n";
  json << "    \"contribution_map\": \""
       << jsonEscape(contributionPath.filename().string()) << "\",\n";
  json << "    \"equirectangular_size\": [" << sphereWidth << ", " << sphereHeight
       << "]\n";
  json << "  }\n";
  json << "}\n";

  std::ofstream reportStream(reportPath);
  if (!reportStream) {
    throw std::runtime_error("Could not create the native engine report");
  }
  reportStream << json.str();

  reportProgress(request, 1.0, "Panorama complete");
  return StitchArtifacts{panoramaPath, reportPath, contributionPath};
}

} // namespace sphera
