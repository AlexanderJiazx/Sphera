#include "SpheraRotationRefinement.hpp"

#include "SpheraEngineMath.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/calib3d.hpp>
#pragma clang diagnostic pop

namespace sphera {
namespace {

cv::Mat cameraMatrix(const cv::detail::CameraParams &camera) {
  cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
  matrix.at<double>(0, 0) = camera.focal;
  matrix.at<double>(1, 1) = camera.focal * camera.aspect;
  matrix.at<double>(0, 2) = camera.ppx;
  matrix.at<double>(1, 2) = camera.ppy;
  return matrix;
}

cv::Mat localRays(const cv::Mat &points, const cv::detail::CameraParams &camera) {
  CV_Assert(points.type() == CV_32FC2 || points.type() == CV_64FC2 ||
            (points.cols == 2 && (points.type() == CV_32F || points.type() == CV_64F)));
  const int count = points.rows * (points.channels() == 2 ? 1 : 1);
  cv::Mat points2;
  if (points.channels() == 2) {
    points2 = points.reshape(1, points.total());
    if (points2.type() != CV_64F) {
      points2.convertTo(points2, CV_64F);
    }
  } else {
    points.convertTo(points2, CV_64F);
  }
  const int n = points2.rows;
  cv::Mat homogeneous(n, 3, CV_64F);
  for (int index = 0; index < n; ++index) {
    homogeneous.at<double>(index, 0) = points2.at<double>(index, 0);
    homogeneous.at<double>(index, 1) = points2.at<double>(index, 1);
    homogeneous.at<double>(index, 2) = 1.0;
  }
  cv::Mat rays = (cameraMatrix(camera).inv() * homogeneous.t()).t();
  for (int index = 0; index < n; ++index) {
    cv::Vec3d ray(rays.at<double>(index, 0), rays.at<double>(index, 1),
                  rays.at<double>(index, 2));
    const double norm = std::max(std::sqrt(ray.dot(ray)), 1e-12);
    rays.at<double>(index, 0) = ray[0] / norm;
    rays.at<double>(index, 1) = ray[1] / norm;
    rays.at<double>(index, 2) = ray[2] / norm;
  }
  return rays;
}

cv::Mat pointsFromKeypoints(const std::vector<cv::KeyPoint> &keypoints,
                            const std::vector<cv::DMatch> &matches, bool query) {
  cv::Mat points(static_cast<int>(matches.size()), 1, CV_32FC2);
  for (int index = 0; index < static_cast<int>(matches.size()); ++index) {
    const int keyIndex =
        query ? matches[static_cast<std::size_t>(index)].queryIdx
              : matches[static_cast<std::size_t>(index)].trainIdx;
    points.at<cv::Point2f>(index) = keypoints[static_cast<std::size_t>(keyIndex)].pt;
  }
  return points;
}

void rayPairState(const std::vector<cv::Mat> &rotations,
                  const RayPairConstraint &edge, double switchScaleDegrees,
                  cv::Mat &sourceWorld, cv::Mat &targetWorld, cv::Mat &angles,
                  cv::Mat &confidence, double &switchValue) {
  sourceWorld = (rotations[static_cast<std::size_t>(edge.source)] * edge.sourceRays.t()).t();
  targetWorld = (rotations[static_cast<std::size_t>(edge.target)] * edge.targetRays.t()).t();
  const int count = sourceWorld.rows;
  angles.create(count, 1, CV_64F);
  confidence.create(count, 1, CV_64F);
  double confidenceSum = 0;
  for (int index = 0; index < count; ++index) {
    const cv::Vec3d source(sourceWorld.at<double>(index, 0),
                           sourceWorld.at<double>(index, 1),
                           sourceWorld.at<double>(index, 2));
    const cv::Vec3d target(targetWorld.at<double>(index, 0),
                           targetWorld.at<double>(index, 1),
                           targetWorld.at<double>(index, 2));
    const double cosine = std::clamp(source.dot(target), -1.0, 1.0);
    angles.at<double>(index) = std::acos(cosine);
    const double conf =
        std::sqrt(std::clamp(edge.confidence.at<double>(index), 1e-5, 1.0));
    confidence.at<double>(index) = conf;
    confidenceSum += conf;
  }
  confidenceSum = std::max(confidenceSum, 1e-12);
  double meanSquared = 0;
  for (int index = 0; index < count; ++index) {
    confidence.at<double>(index) /= confidenceSum;
    const double angle = angles.at<double>(index);
    meanSquared += confidence.at<double>(index) * angle * angle;
  }
  const double switchRegularizer =
      std::pow(switchScaleDegrees * CV_PI / 180.0, 2.0);
  switchValue = switchRegularizer / (switchRegularizer + meanSquared);
  switchValue = std::max(edge.switchFloor, switchValue);
}

double sensorBundleObjective(const std::vector<cv::Mat> &rotations,
                             const std::vector<cv::Mat> &seeds,
                             const std::vector<RayPairConstraint> &constraints,
                             const std::vector<std::pair<int, int>> &adjacentPairs,
                             const SensorAnchoredSolverSettings &settings) {
  const double robustScale =
      std::max(settings.robustDegrees, 1e-6) * CV_PI / 180.0;
  const double sensorSigma =
      std::max(settings.sensorPriorSigmaDegrees, 1e-6) * CV_PI / 180.0;
  const double neighborSigma =
      std::max(settings.neighborSigmaDegrees, 1e-6) * CV_PI / 180.0;
  const double switchRegularizer =
      std::pow(std::max(settings.switchScaleDegrees, 1e-6) * CV_PI / 180.0, 2.0);
  double value = 0;
  for (const RayPairConstraint &edge : constraints) {
    cv::Mat sourceWorld;
    cv::Mat targetWorld;
    cv::Mat angles;
    cv::Mat confidence;
    double switchValue = 0;
    rayPairState(rotations, edge, settings.switchScaleDegrees, sourceWorld,
                 targetWorld, angles, confidence, switchValue);
    double robustError = 0;
    for (int index = 0; index < angles.rows; ++index) {
      const double ratio = angles.at<double>(index) / robustScale;
      robustError +=
          confidence.at<double>(index) * std::log1p(ratio * ratio);
    }
    value += edge.baseWeight *
             (switchValue * switchValue * robustError +
              (switchRegularizer / (robustScale * robustScale)) *
                  std::pow(1.0 - switchValue, 2.0));
  }
  std::vector<cv::Vec3d> corrections(rotations.size());
  for (std::size_t index = 0; index < rotations.size(); ++index) {
    corrections[index] =
        rotationVector(rotations[index] * seeds[index].t());
    value += corrections[index].dot(corrections[index]) /
             (sensorSigma * sensorSigma);
  }
  for (const auto &pair : adjacentPairs) {
    const cv::Vec3d difference =
        corrections[static_cast<std::size_t>(pair.first)] -
        corrections[static_cast<std::size_t>(pair.second)];
    value += difference.dot(difference) / (neighborSigma * neighborSigma);
  }
  return value;
}

} // namespace

void spatialTrainEvaluationSplit(const cv::Mat &sourcePoints,
                                 const cv::Mat &targetPoints,
                                 cv::Size sourceSize, cv::Size targetSize,
                                 int sourceIndex, int targetIndex,
                                 cv::Mat &trainingMask, cv::Mat &evaluationMask) {
  const int count = sourcePoints.rows;
  trainingMask = cv::Mat::ones(count, 1, CV_8U);
  evaluationMask = cv::Mat::zeros(count, 1, CV_8U);
  if (count < 10) {
    return;
  }

  auto cell = [](double coordinate, int limit, int bins) {
    return std::clamp(static_cast<int>(coordinate / std::max(1, limit) * bins),
                      0, bins - 1);
  };

  int evaluationCount = 0;
  for (int index = 0; index < count; ++index) {
    const cv::Point2f source = sourcePoints.at<cv::Point2f>(index);
    const cv::Point2f target = targetPoints.at<cv::Point2f>(index);
    const int64_t sx = cell(source.x, sourceSize.width, 12);
    const int64_t sy = cell(source.y, sourceSize.height, 9);
    const int64_t tx = cell(target.x, targetSize.width, 12);
    const int64_t ty = cell(target.y, targetSize.height, 9);
    const int64_t hashed =
        (sx * 73856093) ^ (sy * 19349663) ^ (tx * 83492791) ^
        (ty * 2654435761LL) ^
        static_cast<int64_t>(sourceIndex * 97531 + targetIndex * 314159);
    if ((hashed % 5) == 0) {
      evaluationMask.at<uchar>(index) = 1;
      trainingMask.at<uchar>(index) = 0;
      ++evaluationCount;
    }
  }

  const int minimumEvaluation = std::max(1, static_cast<int>(std::lround(0.2 * count)));
  if (evaluationCount < minimumEvaluation) {
    std::vector<int> rank(static_cast<std::size_t>(count));
    std::iota(rank.begin(), rank.end(), 0);
    std::sort(rank.begin(), rank.end(), [&](int left, int right) {
      const cv::Point2f ls = sourcePoints.at<cv::Point2f>(left);
      const cv::Point2f rs = sourcePoints.at<cv::Point2f>(right);
      const cv::Point2f lt = targetPoints.at<cv::Point2f>(left);
      const cv::Point2f rt = targetPoints.at<cv::Point2f>(right);
      if (ls.x != rs.x) {
        return ls.x < rs.x;
      }
      if (ls.y != rs.y) {
        return ls.y < rs.y;
      }
      if (lt.x != rt.x) {
        return lt.x < rt.x;
      }
      return lt.y < rt.y;
    });
    evaluationMask.setTo(0);
    trainingMask.setTo(1);
    for (std::size_t index = 0; index < rank.size(); index += 5) {
      evaluationMask.at<uchar>(rank[index]) = 1;
      trainingMask.at<uchar>(rank[index]) = 0;
    }
  }

  if (cv::countNonZero(trainingMask) < 4) {
    trainingMask.setTo(1);
    evaluationMask.setTo(0);
  }
}

std::vector<int> spatiallyBalancedSubset(const cv::Mat &sourcePoints,
                                         const cv::Mat &targetPoints,
                                         const cv::Mat &confidence,
                                         cv::Size sourceSize, cv::Size targetSize,
                                         int maximum, int gridColumns,
                                         int gridRows) {
  const int count = sourcePoints.rows;
  if (count <= maximum) {
    std::vector<int> all(static_cast<std::size_t>(count));
    std::iota(all.begin(), all.end(), 0);
    return all;
  }
  const int cells = gridColumns * gridRows;
  const int quota = std::max(1, static_cast<int>(std::ceil(
                                    static_cast<double>(maximum) / cells)));
  auto cellIndex = [&](const cv::Point2f &point, cv::Size size) {
    const int x = std::clamp(
        static_cast<int>(point.x / std::max(1, size.width) * gridColumns), 0,
        gridColumns - 1);
    const int y = std::clamp(
        static_cast<int>(point.y / std::max(1, size.height) * gridRows), 0,
        gridRows - 1);
    return y * gridColumns + x;
  };
  std::vector<int> order(static_cast<std::size_t>(count));
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int left, int right) {
    return confidence.at<double>(left) > confidence.at<double>(right);
  });
  std::vector<int> sourceCounts(static_cast<std::size_t>(cells), 0);
  std::vector<int> targetCounts(static_cast<std::size_t>(cells), 0);
  std::vector<int> selected;
  std::vector<int> deferred;
  selected.reserve(static_cast<std::size_t>(maximum));
  for (int index : order) {
    const int sourceCell =
        cellIndex(sourcePoints.at<cv::Point2f>(index), sourceSize);
    const int targetCell =
        cellIndex(targetPoints.at<cv::Point2f>(index), targetSize);
    if (sourceCounts[static_cast<std::size_t>(sourceCell)] < quota &&
        targetCounts[static_cast<std::size_t>(targetCell)] < quota) {
      selected.push_back(index);
      ++sourceCounts[static_cast<std::size_t>(sourceCell)];
      ++targetCounts[static_cast<std::size_t>(targetCell)];
    } else {
      deferred.push_back(index);
    }
    if (static_cast<int>(selected.size()) == maximum) {
      break;
    }
  }
  for (int index : deferred) {
    if (static_cast<int>(selected.size()) >= maximum) {
      break;
    }
    selected.push_back(index);
  }
  return selected;
}

double gridCoverage(const cv::Mat &points, cv::Size size, int columns,
                    int rows) {
  if (points.empty()) {
    return 0.0;
  }
  std::set<int> occupied;
  for (int index = 0; index < points.rows; ++index) {
    const cv::Point2f point = points.at<cv::Point2f>(index);
    const int x = std::clamp(
        static_cast<int>(point.x / std::max(1, size.width) * columns), 0,
        columns - 1);
    const int y = std::clamp(
        static_cast<int>(point.y / std::max(1, size.height) * rows), 0,
        rows - 1);
    occupied.insert(y * columns + x);
  }
  return static_cast<double>(occupied.size()) /
         static_cast<double>(columns * rows);
}

cv::Mat estimateRelativeRotation(const cv::Mat &sourceRays,
                                 const cv::Mat &targetRays,
                                 const cv::Mat &confidence,
                                 cv::Mat *errorsDegrees) {
  CV_Assert(sourceRays.rows == targetRays.rows && sourceRays.cols == 3);
  const int count = sourceRays.rows;
  if (count < 3) {
    throw std::runtime_error("Need at least three rays for relative rotation");
  }
  cv::Mat baseWeight(count, 1, CV_64F);
  if (confidence.empty()) {
    baseWeight.setTo(1.0);
  } else {
    for (int index = 0; index < count; ++index) {
      baseWeight.at<double>(index) =
          std::sqrt(std::clamp(confidence.at<double>(index), 1e-4, 1.0));
    }
  }
  cv::Mat weight = baseWeight.clone();
  cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat errors(count, 1, CV_64F);
  for (int iteration = 0; iteration < 4; ++iteration) {
    cv::Mat cross = cv::Mat::zeros(3, 3, CV_64F);
    for (int index = 0; index < count; ++index) {
      const double w = weight.at<double>(index);
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          cross.at<double>(row, column) +=
              w * sourceRays.at<double>(index, row) *
              targetRays.at<double>(index, column);
        }
      }
    }
    rotation = properRotation(cross);
    std::vector<double> absoluteErrors;
    absoluteErrors.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      cv::Mat projected =
          rotation * (cv::Mat_<double>(3, 1)
                      << targetRays.at<double>(index, 0),
                      targetRays.at<double>(index, 1),
                      targetRays.at<double>(index, 2));
      const cv::Vec3d source(sourceRays.at<double>(index, 0),
                             sourceRays.at<double>(index, 1),
                             sourceRays.at<double>(index, 2));
      const cv::Vec3d proj(projected.at<double>(0), projected.at<double>(1),
                           projected.at<double>(2));
      const double cosine = std::clamp(source.dot(proj), -1.0, 1.0);
      const double angle = std::acos(cosine);
      errors.at<double>(index) = angle;
      absoluteErrors.push_back(angle);
    }
    const double scale =
        std::max(0.35 * CV_PI / 180.0, 2.5 * medianOf(absoluteErrors));
    for (int index = 0; index < count; ++index) {
      const double ratio = errors.at<double>(index) / scale;
      weight.at<double>(index) =
          baseWeight.at<double>(index) / (1.0 + ratio * ratio);
    }
  }
  if (errorsDegrees != nullptr) {
    *errorsDegrees = errors * (180.0 / CV_PI);
  }
  return rotation;
}

cv::Mat limitTotalCorrection(const cv::Mat &candidate, const cv::Mat &seed,
                             double maximumDegrees) {
  const cv::Mat difference = properRotation(candidate * seed.t());
  const cv::Vec3d vector = rotationVector(difference);
  const double norm = std::sqrt(vector.dot(vector));
  const double maximum = maximumDegrees * CV_PI / 180.0;
  if (norm <= maximum || norm < 1e-12) {
    return properRotation(candidate);
  }
  cv::Mat limited;
  cv::Rodrigues(cv::Mat(vector * (maximum / norm)), limited);
  return properRotation(limited * seed);
}

namespace {

std::pair<RayPairConstraint, bool>
sensorConstraintFromPoints(const std::vector<cv::detail::CameraParams> &cameras,
                           int sourceIndex, int targetIndex, cv::Mat source,
                           cv::Mat target, cv::Mat confidence,
                           const std::vector<cv::Size> &workSizes,
                           const std::vector<int> &ringIndices,
                           const std::vector<int> &ringLocalIndices,
                           const std::vector<int> &ringSizes,
                           double predictedOverlap,
                           const SensorRayGateSettings &settings,
                           RejectedPairRecord &rejection) {
  rejection = {};
  rejection.source = sourceIndex;
  rejection.target = targetIndex;
  if (source.rows < settings.minimumInliers) {
    rejection.reason = "too few confident matches";
    rejection.matches = source.rows;
    return {{}, false};
  }

  cv::Mat centeredSource(source.rows, 1, CV_32FC2);
  cv::Mat centeredTarget(target.rows, 1, CV_32FC2);
  const cv::Point2f sourceCenter(
      static_cast<float>(workSizes[static_cast<std::size_t>(sourceIndex)].width) /
          2.0f,
      static_cast<float>(
          workSizes[static_cast<std::size_t>(sourceIndex)].height) /
          2.0f);
  const cv::Point2f targetCenter(
      static_cast<float>(workSizes[static_cast<std::size_t>(targetIndex)].width) /
          2.0f,
      static_cast<float>(
          workSizes[static_cast<std::size_t>(targetIndex)].height) /
          2.0f);
  for (int index = 0; index < source.rows; ++index) {
    centeredSource.at<cv::Point2f>(index) =
        source.at<cv::Point2f>(index) - sourceCenter;
    centeredTarget.at<cv::Point2f>(index) =
        target.at<cv::Point2f>(index) - targetCenter;
  }

  cv::Mat inlierMask;
  const cv::Mat homography = cv::findHomography(
      centeredSource, centeredTarget, cv::USAC_MAGSAC,
      settings.reprojectionThreshold, inlierMask, 10000, 0.999);
  if (homography.empty() || inlierMask.empty()) {
    rejection.reason = "homography failed";
    return {{}, false};
  }
  int inlierCount = 0;
  for (int index = 0; index < inlierMask.rows; ++index) {
    if (inlierMask.at<uchar>(index) != 0) {
      ++inlierCount;
    }
  }
  const double inlierRatio =
      static_cast<double>(inlierCount) / std::max(1, source.rows);
  if (inlierCount < settings.minimumInliers ||
      inlierRatio < settings.minimumInlierRatio) {
    rejection.reason = "geometric consensus rejected";
    rejection.matches = source.rows;
    rejection.inliers = inlierCount;
    rejection.inlierRatio = inlierRatio;
    return {{}, false};
  }

  cv::Mat filteredSource(inlierCount, 1, CV_32FC2);
  cv::Mat filteredTarget(inlierCount, 1, CV_32FC2);
  cv::Mat filteredConfidence(inlierCount, 1, CV_64F);
  int write = 0;
  for (int index = 0; index < source.rows; ++index) {
    if (inlierMask.at<uchar>(index) == 0) {
      continue;
    }
    filteredSource.at<cv::Point2f>(write) = source.at<cv::Point2f>(index);
    filteredTarget.at<cv::Point2f>(write) = target.at<cv::Point2f>(index);
    filteredConfidence.at<double>(write) = confidence.at<double>(index);
    ++write;
  }
  source = filteredSource;
  target = filteredTarget;
  confidence = filteredConfidence;

  const double sourceCoverage =
      gridCoverage(source, workSizes[static_cast<std::size_t>(sourceIndex)]);
  const double targetCoverage =
      gridCoverage(target, workSizes[static_cast<std::size_t>(targetIndex)]);
  const double coverage = std::sqrt(sourceCoverage * targetCoverage);
  if (coverage < settings.minimumSpatialCoverage) {
    rejection.reason = "insufficient spatial coverage";
    rejection.inliers = inlierCount;
    rejection.spatialCoverage = coverage;
    return {{}, false};
  }

  const cv::Mat sourceRaysAll =
      localRays(source, cameras[static_cast<std::size_t>(sourceIndex)]);
  const cv::Mat targetRaysAll =
      localRays(target, cameras[static_cast<std::size_t>(targetIndex)]);
  cv::Mat sourceRotation;
  cv::Mat targetRotation;
  cameras[static_cast<std::size_t>(sourceIndex)].R.convertTo(sourceRotation,
                                                             CV_64F);
  cameras[static_cast<std::size_t>(targetIndex)].R.convertTo(targetRotation,
                                                             CV_64F);
  const cv::Mat sourceWorld = (sourceRotation * sourceRaysAll.t()).t();
  const cv::Mat targetWorld = (targetRotation * targetRaysAll.t()).t();
  std::vector<double> sensorErrors;
  sensorErrors.reserve(static_cast<std::size_t>(sourceWorld.rows));
  for (int index = 0; index < sourceWorld.rows; ++index) {
    const cv::Vec3d left(sourceWorld.at<double>(index, 0),
                         sourceWorld.at<double>(index, 1),
                         sourceWorld.at<double>(index, 2));
    const cv::Vec3d right(targetWorld.at<double>(index, 0),
                          targetWorld.at<double>(index, 1),
                          targetWorld.at<double>(index, 2));
    sensorErrors.push_back(
        std::acos(std::clamp(left.dot(right), -1.0, 1.0)) * 180.0 / CV_PI);
  }
  const double sensorMedian = medianOf(sensorErrors);
  const double sensorP90 = percentileOf(sensorErrors, 90.0);
  if (sensorMedian > settings.maximumSensorMedianDegrees ||
      sensorP90 > settings.maximumSensorP90Degrees) {
    rejection.reason = "sensor ray residual rejected";
    rejection.sensorMedianResidualDegrees = sensorMedian;
    rejection.sensorP90ResidualDegrees = sensorP90;
    return {{}, false};
  }

  cv::Mat internalErrors;
  const cv::Mat relative =
      estimateRelativeRotation(sourceRaysAll, targetRaysAll, confidence,
                               &internalErrors);
  const cv::Mat sensorRelative = sourceRotation.t() * targetRotation;
  const double relativeDisagreement =
      rotationAngleDegrees(sensorRelative * relative.t());
  if (relativeDisagreement > settings.maximumRelativeDisagreementDegrees) {
    rejection.reason = "relative rotation disagrees with sensor";
    rejection.relativeDisagreementDegrees = relativeDisagreement;
    return {{}, false};
  }

  if (source.rows > settings.maximumMatchesPerPair) {
    const std::vector<int> chosen = spatiallyBalancedSubset(
        source, target, confidence,
        workSizes[static_cast<std::size_t>(sourceIndex)],
        workSizes[static_cast<std::size_t>(targetIndex)],
        settings.maximumMatchesPerPair);
    cv::Mat subsetSource(static_cast<int>(chosen.size()), 1, CV_32FC2);
    cv::Mat subsetTarget(static_cast<int>(chosen.size()), 1, CV_32FC2);
    cv::Mat subsetConfidence(static_cast<int>(chosen.size()), 1, CV_64F);
    for (std::size_t index = 0; index < chosen.size(); ++index) {
      subsetSource.at<cv::Point2f>(static_cast<int>(index)) =
          source.at<cv::Point2f>(chosen[index]);
      subsetTarget.at<cv::Point2f>(static_cast<int>(index)) =
          target.at<cv::Point2f>(chosen[index]);
      subsetConfidence.at<double>(static_cast<int>(index)) =
          confidence.at<double>(chosen[index]);
    }
    source = subsetSource;
    target = subsetTarget;
    confidence = subsetConfidence;
  }

  cv::Mat trainingMask;
  cv::Mat evaluationMask;
  spatialTrainEvaluationSplit(
      source, target, workSizes[static_cast<std::size_t>(sourceIndex)],
      workSizes[static_cast<std::size_t>(targetIndex)], sourceIndex, targetIndex,
      trainingMask, evaluationMask);

  const cv::Mat sourceRays =
      localRays(source, cameras[static_cast<std::size_t>(sourceIndex)]);
  const cv::Mat targetRays =
      localRays(target, cameras[static_cast<std::size_t>(targetIndex)]);

  auto gather = [](const cv::Mat &matrix, const cv::Mat &mask) {
    const int keep = cv::countNonZero(mask);
    cv::Mat result(keep, matrix.cols, matrix.type());
    int writeIndex = 0;
    for (int index = 0; index < mask.rows; ++index) {
      if (mask.at<uchar>(index) == 0) {
        continue;
      }
      matrix.row(index).copyTo(result.row(writeIndex++));
    }
    return result;
  };
  auto gatherConfidence = [](const cv::Mat &matrix, const cv::Mat &mask) {
    const int keep = cv::countNonZero(mask);
    cv::Mat result(keep, 1, CV_64F);
    int writeIndex = 0;
    for (int index = 0; index < mask.rows; ++index) {
      if (mask.at<uchar>(index) == 0) {
        continue;
      }
      result.at<double>(writeIndex++) = matrix.at<double>(index);
    }
    return result;
  };

  RayPairConstraint constraint;
  constraint.source = sourceIndex;
  constraint.target = targetIndex;
  constraint.sourceRays = gather(sourceRays, trainingMask);
  constraint.targetRays = gather(targetRays, trainingMask);
  constraint.confidence = gatherConfidence(confidence, trainingMask);
  constraint.matches = inlierMask.rows;
  constraint.inliers = inlierCount;
  constraint.inlierRatio = inlierRatio;
  constraint.spatialCoverage = coverage;
  std::vector<double> internalDegrees;
  for (int index = 0; index < internalErrors.rows; ++index) {
    internalDegrees.push_back(internalErrors.at<double>(index));
  }
  constraint.internalMedianErrorDegrees = medianOf(internalDegrees);
  constraint.sameRing = ringIndices[static_cast<std::size_t>(sourceIndex)] ==
                        ringIndices[static_cast<std::size_t>(targetIndex)];
  constraint.ringDistance = -1;
  constraint.wrapEdge = false;
  if (constraint.sameRing) {
    const int ringSize = ringSizes[static_cast<std::size_t>(sourceIndex)];
    const int rawDistance = std::abs(
        ringLocalIndices[static_cast<std::size_t>(sourceIndex)] -
        ringLocalIndices[static_cast<std::size_t>(targetIndex)]);
    constraint.ringDistance = std::min(rawDistance, ringSize - rawDistance);
    constraint.wrapEdge =
        constraint.ringDistance == 1 && rawDistance == ringSize - 1;
  }
  constraint.baseWeight = std::clamp(
      (predictedOverlap / 0.30) * (inlierRatio / 0.50) * (coverage / 0.20), 0.10,
      3.0);
  constraint.predictedOverlap = predictedOverlap;
  constraint.sensorMedianResidualDegrees = sensorMedian;
  constraint.sensorP90ResidualDegrees = sensorP90;
  constraint.relativeDisagreementDegrees = relativeDisagreement;
  if (cv::countNonZero(evaluationMask) > 0) {
    constraint.evaluationSourceRays = gather(sourceRays, evaluationMask);
    constraint.evaluationTargetRays = gather(targetRays, evaluationMask);
    constraint.evaluationConfidence =
        gatherConfidence(confidence, evaluationMask);
  }
  return {constraint, true};
}

} // namespace

std::pair<std::vector<RayPairConstraint>, std::vector<RejectedPairRecord>>
buildSensorRayConstraintsFromSift(
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<cv::detail::ImageFeatures> &features,
    const std::vector<cv::detail::MatchesInfo> &pairwiseMatches,
    const std::vector<cv::Size> &workSizes, const cv::Mat &matchMask,
    const cv::Mat &overlap, const std::vector<int> &ringIndices,
    const std::vector<int> &ringLocalIndices, const std::vector<int> &ringSizes,
    const SensorRayGateSettings &settings) {
  std::vector<RayPairConstraint> constraints;
  std::vector<RejectedPairRecord> rejected;
  for (const cv::detail::MatchesInfo &match : pairwiseMatches) {
    const int sourceIndex = match.src_img_idx;
    const int targetIndex = match.dst_img_idx;
    if (sourceIndex < 0 || targetIndex < 0 || sourceIndex >= targetIndex) {
      continue;
    }
    const double predicted = overlap.at<double>(sourceIndex, targetIndex);
    if (matchMask.at<uchar>(sourceIndex, targetIndex) == 0 ||
        predicted < settings.minimumPredictedOverlap) {
      continue;
    }
    if (match.matches.empty()) {
      continue;
    }
    cv::Mat source = pointsFromKeypoints(features[static_cast<std::size_t>(sourceIndex)].keypoints,
                                         match.matches, true);
    cv::Mat target = pointsFromKeypoints(features[static_cast<std::size_t>(targetIndex)].keypoints,
                                         match.matches, false);
    cv::Mat confidence(source.rows, 1, CV_64F, cv::Scalar(1.0));
    RejectedPairRecord rejection;
    auto result = sensorConstraintFromPoints(
        cameras, sourceIndex, targetIndex, source, target, confidence, workSizes,
        ringIndices, ringLocalIndices, ringSizes, predicted, settings,
        rejection);
    if (result.second) {
      constraints.push_back(std::move(result.first));
    } else if (!rejection.reason.empty()) {
      rejected.push_back(std::move(rejection));
    }
  }
  return {constraints, rejected};
}

std::pair<std::vector<cv::Mat>, SensorAnchoredSolution>
solveSensorAnchoredRayBundle(
    const std::vector<cv::Mat> &sensorRotations,
    const std::vector<RayPairConstraint> &constraints,
    const std::vector<std::pair<int, int>> &adjacentPairs,
    const SensorAnchoredSolverSettings &settings) {
  const int count = static_cast<int>(sensorRotations.size());
  std::vector<cv::Mat> seeds(static_cast<std::size_t>(count));
  std::vector<cv::Mat> rotations(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    seeds[static_cast<std::size_t>(index)] =
        properRotation(sensorRotations[static_cast<std::size_t>(index)]);
    rotations[static_cast<std::size_t>(index)] =
        seeds[static_cast<std::size_t>(index)].clone();
  }
  std::set<int> active;
  for (const RayPairConstraint &edge : constraints) {
    active.insert(edge.source);
    active.insert(edge.target);
  }

  SensorAnchoredSolution solution;
  solution.initialObjective = sensorBundleObjective(
      rotations, seeds, constraints, adjacentPairs, settings);
  const double robustScale =
      std::max(settings.robustDegrees, 1e-6) * CV_PI / 180.0;
  const double sensorSigma =
      std::max(settings.sensorPriorSigmaDegrees, 1e-6) * CV_PI / 180.0;
  const double neighborSigma =
      std::max(settings.neighborSigmaDegrees, 1e-6) * CV_PI / 180.0;

  for (int iteration = 0; iteration < settings.iterations; ++iteration) {
    cv::Mat hessian = cv::Mat::zeros(3 * count, 3 * count, CV_64F);
    cv::Mat rhs = cv::Mat::zeros(3 * count, 1, CV_64F);
    std::vector<double> switches;
    std::vector<double> pairMedians;
    for (const RayPairConstraint &edge : constraints) {
      cv::Mat sourceWorld;
      cv::Mat targetWorld;
      cv::Mat angles;
      cv::Mat confidence;
      double switchValue = 0;
      rayPairState(rotations, edge, settings.switchScaleDegrees, sourceWorld,
                   targetWorld, angles, confidence, switchValue);
      switches.push_back(switchValue);
      std::vector<double> angleDegrees;
      for (int index = 0; index < angles.rows; ++index) {
        angleDegrees.push_back(angles.at<double>(index) * 180.0 / CV_PI);
      }
      pairMedians.push_back(medianOf(angleDegrees));
      for (int index = 0; index < angles.rows; ++index) {
        const double robust =
            1.0 /
            (1.0 + std::pow(angles.at<double>(index) / robustScale, 2.0));
        const double weight =
            edge.baseWeight * switchValue * switchValue *
            confidence.at<double>(index) * robust /
            (robustScale * robustScale);
        const cv::Vec3d sourceRay(sourceWorld.at<double>(index, 0),
                                  sourceWorld.at<double>(index, 1),
                                  sourceWorld.at<double>(index, 2));
        const cv::Vec3d targetRay(targetWorld.at<double>(index, 0),
                                  targetWorld.at<double>(index, 1),
                                  targetWorld.at<double>(index, 2));
        const cv::Vec3d residual = sourceRay - targetRay;
        const cv::Matx33d sourceJacobian = -skew(sourceRay);
        const cv::Matx33d targetJacobian = skew(targetRay);
        const int sourceOffset = 3 * edge.source;
        const int targetOffset = 3 * edge.target;
        const cv::Mat sourceJ = cv::Mat(sourceJacobian);
        const cv::Mat targetJ = cv::Mat(targetJacobian);
        hessian(cv::Rect(sourceOffset, sourceOffset, 3, 3)) +=
            weight * (sourceJ.t() * sourceJ);
        hessian(cv::Rect(targetOffset, targetOffset, 3, 3)) +=
            weight * (targetJ.t() * targetJ);
        const cv::Mat cross = weight * (sourceJ.t() * targetJ);
        hessian(cv::Rect(targetOffset, sourceOffset, 3, 3)) += cross;
        hessian(cv::Rect(sourceOffset, targetOffset, 3, 3)) += cross.t();
        rhs(cv::Rect(0, sourceOffset, 1, 3)) -=
            weight * (sourceJ.t() * cv::Mat(residual));
        rhs(cv::Rect(0, targetOffset, 1, 3)) -=
            weight * (targetJ.t() * cv::Mat(residual));
      }
    }

    const double priorWeight = 1.0 / (sensorSigma * sensorSigma);
    std::vector<cv::Vec3d> corrections(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      corrections[static_cast<std::size_t>(index)] = rotationVector(
          rotations[static_cast<std::size_t>(index)] *
          seeds[static_cast<std::size_t>(index)].t());
      const int offset = 3 * index;
      hessian(cv::Rect(offset, offset, 3, 3)) +=
          priorWeight * cv::Mat::eye(3, 3, CV_64F);
      rhs(cv::Rect(0, offset, 1, 3)) -=
          priorWeight * cv::Mat(corrections[static_cast<std::size_t>(index)]);
    }
    const double smoothWeight = 1.0 / (neighborSigma * neighborSigma);
    for (const auto &pair : adjacentPairs) {
      const int left = pair.first;
      const int right = pair.second;
      const cv::Vec3d error = corrections[static_cast<std::size_t>(left)] -
                              corrections[static_cast<std::size_t>(right)];
      const cv::Mat block = smoothWeight * cv::Mat::eye(3, 3, CV_64F);
      hessian(cv::Rect(3 * left, 3 * left, 3, 3)) += block;
      hessian(cv::Rect(3 * right, 3 * right, 3, 3)) += block;
      hessian(cv::Rect(3 * right, 3 * left, 3, 3)) -= block;
      hessian(cv::Rect(3 * left, 3 * right, 3, 3)) -= block;
      rhs(cv::Rect(0, 3 * left, 1, 3)) -= smoothWeight * cv::Mat(error);
      rhs(cv::Rect(0, 3 * right, 1, 3)) += smoothWeight * cv::Mat(error);
    }

    cv::Mat step;
    if (!cv::solve(hessian + 1e-10 * cv::Mat::eye(3 * count, 3 * count, CV_64F),
                   rhs, step, cv::DECOMP_SVD)) {
      break;
    }
    const double maximumStep = settings.maximumStepDegrees * CV_PI / 180.0;
    double rawMaximum = 0;
    for (int index = 0; index < count; ++index) {
      cv::Mat block = step(cv::Rect(0, 3 * index, 1, 3));
      if (active.count(index) == 0) {
        block.setTo(0);
        continue;
      }
      double norm = std::sqrt(block.dot(block));
      if (norm > maximumStep && maximumStep > 0) {
        block *= maximumStep / norm;
        norm = maximumStep;
      }
      rawMaximum = std::max(rawMaximum, norm);
    }

    const double before = sensorBundleObjective(rotations, seeds, constraints,
                                                adjacentPairs, settings);
    std::vector<cv::Mat> accepted = rotations;
    double acceptedObjective = before;
    double acceptedScale = 0;
    double stepScale = 1.0;
    for (int lineSearch = 0; lineSearch < 8; ++lineSearch) {
      std::vector<cv::Mat> candidate(static_cast<std::size_t>(count));
      for (int index = 0; index < count; ++index) {
        if (active.count(index) == 0) {
          candidate[static_cast<std::size_t>(index)] =
              seeds[static_cast<std::size_t>(index)].clone();
          continue;
        }
        cv::Mat delta;
        cv::Rodrigues(step(cv::Rect(0, 3 * index, 1, 3)) * stepScale, delta);
        const cv::Mat proposed =
            properRotation(delta * rotations[static_cast<std::size_t>(index)]);
        candidate[static_cast<std::size_t>(index)] = limitTotalCorrection(
            proposed, seeds[static_cast<std::size_t>(index)],
            settings.maximumTotalCorrectionDegrees);
      }
      const double objective = sensorBundleObjective(
          candidate, seeds, constraints, adjacentPairs, settings);
      if (objective <= acceptedObjective + 1e-12) {
        accepted = candidate;
        acceptedObjective = objective;
        acceptedScale = stepScale;
        break;
      }
      stepScale *= 0.5;
    }
    rotations = accepted;
    ++solution.iterationsRun;
    const double appliedDegrees = rawMaximum * acceptedScale * 180.0 / CV_PI;
    if (acceptedScale == 0.0 || appliedDegrees < settings.convergenceDegrees) {
      break;
    }
  }

  solution.finalObjective = sensorBundleObjective(rotations, seeds, constraints,
                                                  adjacentPairs, settings);
  solution.activeCameraIndices.assign(active.begin(), active.end());
  for (int index = 0; index < count; ++index) {
    if (active.count(index) == 0) {
      solution.unconstrainedCameraIndices.push_back(index);
    }
  }
  solution.cameraCorrectionDegrees.resize(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    solution.cameraCorrectionDegrees[static_cast<std::size_t>(index)] =
        rotationAngleDegrees(rotations[static_cast<std::size_t>(index)] *
                             seeds[static_cast<std::size_t>(index)].t());
  }
  solution.maximumCameraCorrectionDegrees =
      solution.cameraCorrectionDegrees.empty()
          ? 0.0
          : *std::max_element(solution.cameraCorrectionDegrees.begin(),
                              solution.cameraCorrectionDegrees.end());
  solution.p90CameraCorrectionDegrees =
      percentileOf(solution.cameraCorrectionDegrees, 90.0);
  for (const RayPairConstraint &edge : constraints) {
    cv::Mat sourceWorld;
    cv::Mat targetWorld;
    cv::Mat angles;
    cv::Mat confidence;
    double switchValue = 0;
    rayPairState(rotations, edge, settings.switchScaleDegrees, sourceWorld,
                 targetWorld, angles, confidence, switchValue);
    std::vector<double> angleDegrees;
    for (int index = 0; index < angles.rows; ++index) {
      angleDegrees.push_back(angles.at<double>(index) * 180.0 / CV_PI);
    }
    solution.pairIndices.emplace_back(edge.source, edge.target);
    solution.pairSwitches.push_back(switchValue);
    solution.pairMedianResidualsDegrees.push_back(medianOf(angleDegrees));
  }
  return {rotations, solution};
}

SensorAnchoredRefineReport refineSensorAnchoredCameras(
    std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<RayPairConstraint> &constraints,
    const std::vector<RejectedPairRecord> &rejected,
    const std::vector<std::pair<int, int>> &adjacentPairs,
    const std::string &source,
    const SensorAnchoredSolverSettings &settings) {
  SensorAnchoredRefineReport report;
  report.enabled = true;
  report.source = source;
  report.constraintCount = static_cast<int>(constraints.size());
  report.rejectedPairCount = static_cast<int>(rejected.size());
  report.constraints = constraints;
  report.rejectedPairs = rejected;
  if (constraints.empty()) {
    report.solution.unconstrainedCameraIndices.resize(cameras.size());
    std::iota(report.solution.unconstrainedCameraIndices.begin(),
              report.solution.unconstrainedCameraIndices.end(), 0);
    report.solution.cameraCorrectionDegrees.assign(cameras.size(), 0.0);
    return report;
  }
  std::vector<cv::Mat> sensorRotations(cameras.size());
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    cameras[index].R.convertTo(sensorRotations[index], CV_64F);
  }
  auto solved = solveSensorAnchoredRayBundle(sensorRotations, constraints,
                                             adjacentPairs, settings);
  for (std::size_t index = 0; index < cameras.size(); ++index) {
    cv::Mat rotation32;
    solved.first[index].convertTo(rotation32, CV_32F);
    cameras[index].R = rotation32;
  }
  report.solution = solved.second;
  return report;
}

} // namespace sphera
