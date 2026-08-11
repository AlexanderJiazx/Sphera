#pragma once

#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#pragma clang diagnostic pop

namespace sphera {

struct RayPairConstraint {
  int source = 0;
  int target = 0;
  cv::Mat sourceRays; // Nx3 CV_64F
  cv::Mat targetRays;
  cv::Mat confidence; // Nx1 CV_64F
  double baseWeight = 1.0;
  int matches = 0;
  int inliers = 0;
  double inlierRatio = 0;
  double spatialCoverage = 0;
  double internalMedianErrorDegrees = 0;
  bool sameRing = false;
  int ringDistance = -1;
  bool wrapEdge = false;
  double switchFloor = 0;
  double predictedOverlap = 0;
  double sensorMedianResidualDegrees = 0;
  double sensorP90ResidualDegrees = 0;
  double relativeDisagreementDegrees = 0;
  cv::Mat evaluationSourceRays;
  cv::Mat evaluationTargetRays;
  cv::Mat evaluationConfidence;
};

struct RejectedPairRecord {
  int source = 0;
  int target = 0;
  std::string reason;
  int matches = 0;
  int inliers = 0;
  double inlierRatio = 0;
  double spatialCoverage = 0;
  double sensorMedianResidualDegrees = 0;
  double sensorP90ResidualDegrees = 0;
  double relativeDisagreementDegrees = 0;
  double predictedOverlap = 0;
};

struct SensorRayGateSettings {
  double minimumPredictedOverlap = 0.15;
  int minimumInliers = 30;
  double minimumInlierRatio = 0.35;
  double minimumSpatialCoverage = 0.12;
  double maximumSensorMedianDegrees = 2.0;
  double maximumSensorP90Degrees = 5.0;
  double maximumRelativeDisagreementDegrees = 8.0;
  double reprojectionThreshold = 3.0;
  int maximumMatchesPerPair = 256;
};

struct SensorAnchoredSolverSettings {
  int iterations = 30;
  double sensorPriorSigmaDegrees = 1.5;
  double neighborSigmaDegrees = 1.0;
  double robustDegrees = 2.0;
  double switchScaleDegrees = 3.0;
  double maximumStepDegrees = 0.5;
  double maximumTotalCorrectionDegrees = 6.0;
  double convergenceDegrees = 0.01;
};

struct SensorAnchoredSolution {
  std::string mode = "sensor_anchored_switchable_spherical";
  int iterationsRun = 0;
  double initialObjective = 0;
  double finalObjective = 0;
  std::vector<int> activeCameraIndices;
  std::vector<int> unconstrainedCameraIndices;
  std::vector<double> cameraCorrectionDegrees;
  double maximumCameraCorrectionDegrees = 0;
  double p90CameraCorrectionDegrees = 0;
  std::vector<std::pair<int, int>> pairIndices;
  std::vector<double> pairSwitches;
  std::vector<double> pairMedianResidualsDegrees;
};

struct SensorAnchoredRefineReport {
  bool enabled = true;
  std::string source;
  int constraintCount = 0;
  int rejectedPairCount = 0;
  SensorAnchoredSolution solution;
  std::vector<RayPairConstraint> constraints;
  std::vector<RejectedPairRecord> rejectedPairs;
};

/// Deterministic spatial 80/20 train/evaluation split (hashed cells).
void spatialTrainEvaluationSplit(const cv::Mat &sourcePoints,
                                 const cv::Mat &targetPoints,
                                 cv::Size sourceSize, cv::Size targetSize,
                                 int sourceIndex, int targetIndex,
                                 cv::Mat &trainingMask, cv::Mat &evaluationMask);

/// Spatially balanced subset (matches learned_matches.spatially_balanced_subset).
std::vector<int> spatiallyBalancedSubset(const cv::Mat &sourcePoints,
                                         const cv::Mat &targetPoints,
                                         const cv::Mat &confidence,
                                         cv::Size sourceSize, cv::Size targetSize,
                                         int maximum, int gridColumns = 12,
                                         int gridRows = 9);

double gridCoverage(const cv::Mat &points, cv::Size size, int columns = 6,
                    int rows = 4);

cv::Mat estimateRelativeRotation(const cv::Mat &sourceRays,
                                 const cv::Mat &targetRays,
                                 const cv::Mat &confidence,
                                 cv::Mat *errorsDegrees = nullptr);

std::pair<std::vector<RayPairConstraint>, std::vector<RejectedPairRecord>>
buildSensorRayConstraintsFromSift(
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<cv::detail::ImageFeatures> &features,
    const std::vector<cv::detail::MatchesInfo> &pairwiseMatches,
    const std::vector<cv::Size> &workSizes, const cv::Mat &matchMask,
    const cv::Mat &overlap, const std::vector<int> &ringIndices,
    const std::vector<int> &ringLocalIndices, const std::vector<int> &ringSizes,
    const SensorRayGateSettings &settings = {});

std::pair<std::vector<cv::Mat>, SensorAnchoredSolution>
solveSensorAnchoredRayBundle(
    const std::vector<cv::Mat> &sensorRotations,
    const std::vector<RayPairConstraint> &constraints,
    const std::vector<std::pair<int, int>> &adjacentPairs,
    const SensorAnchoredSolverSettings &settings = {});

SensorAnchoredRefineReport refineSensorAnchoredCameras(
    std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<RayPairConstraint> &constraints,
    const std::vector<RejectedPairRecord> &rejected,
    const std::vector<std::pair<int, int>> &adjacentPairs,
    const std::string &source,
    const SensorAnchoredSolverSettings &settings = {});

cv::Mat limitTotalCorrection(const cv::Mat &candidate, const cv::Mat &seed,
                             double maximumDegrees);

} // namespace sphera
