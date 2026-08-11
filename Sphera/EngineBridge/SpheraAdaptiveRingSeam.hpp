#pragma once

#include "SpheraPoseOverlap.hpp"

#include <string>
#include <unordered_map>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#pragma clang diagnostic pop

namespace sphera {

struct AdaptiveRingSeamBoundary {
  int upperRing = 0;
  int lowerRing = 0;
  double upperCenterGlobalY = 0;
  double lowerCenterGlobalY = 0;
  double midpointGlobalY = 0;
  int searchBandTopGlobalY = 0;
  int searchBandBottomGlobalY = 0;
  int pathMinimumGlobalY = 0;
  int pathMaximumGlobalY = 0;
  double pathMeanGlobalY = 0;
  double pathCostMean = 0;
  cv::Vec2d robustGlobalFlowPixels;
  double flowResidualP90Pixels = 0;
  int unsupportedLongitudeColumns = 0;
  std::vector<int> pathSamplesGlobalY;
  int pathSampleStep = 1;
};

struct AdaptiveRingSeamReport {
  bool enabled = true;
  std::string mode = "adaptive-periodic-structure-path";
  double overlapFraction = 0.25;
  double edgeWeight = 4.0;
  double geometryWeight = 2.0;
  int maximumStepPixels = 2;
  double smoothness = 0.35;
  cv::Rect roi;
  std::unordered_map<int, double> ringCentersGlobalY;
  std::vector<int> ringOrderTopToBottom;
  std::vector<AdaptiveRingSeamBoundary> boundaries;
  std::vector<double> retainedMaskFractionByInput;
  int coveragePixelsRestored = 0;
};

/// Encode robust color / gradient / edge channels for COST_COLOR graph cut.
cv::Mat structureSeamProxy(const cv::Mat &bgrImage);

/// Exact periodic longitude closure for a smooth left-to-right seam path.
cv::Mat periodicMinimumCostPath(const cv::Mat &cost, int maximumStep = 2,
                                double smoothness = 0.35);

/// Separate adjacent capture rings with structure-aware periodic paths.
std::pair<std::vector<cv::Mat>, AdaptiveRingSeamReport>
applyAdaptiveRingSeamPriors(const std::vector<cv::Mat> &masks,
                            const std::vector<cv::Point> &corners,
                            const std::vector<cv::Mat> &images,
                            const std::vector<PoseFrameLayout> &layout,
                            double overlapFraction = 0.25,
                            double edgeWeight = 4.0,
                            double geometryWeight = 2.0, int maximumStep = 2,
                            double smoothness = 0.35);

} // namespace sphera
