#pragma once

#include "SpheraPanoramaEngine.hpp"

#include <string>
#include <utility>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/core.hpp>
#pragma clang diagnostic pop

namespace sphera {

struct PoseFrameLayout {
  int ring = 0;
  int localIndex = 0;
  int ringSize = 0;
  int direction = 1;
  double phase = 0;
  std::string ringName;
  double sensorYawDegrees = 0;
  double sensorPitchDegrees = 0;
};

struct PoseOverlapPairRecord {
  int source = 0;
  int target = 0;
  std::string sourceFile;
  std::string targetFile;
  bool sameRing = false;
  double predictedOverlapFraction = 0;
  std::vector<std::string> selectionReasons;
};

struct PoseOverlapReport {
  std::string mode = "pose-overlap";
  int gridWidth = 720;
  int gridHeight = 360;
  std::string rotationConvention = "capture_ref";
  double minimumCrossOverlapFraction = 0.15;
  int crossNeighbors = 2;
  double predictedFullSphereCoverageFraction = 0;
  double predictedTwoOrMoreCoverageFraction = 0;
  std::vector<std::string> ringOrder;
  std::vector<PoseOverlapPairRecord> selectedPairs;
  // Parallel to canonical frames.
  std::vector<double> visibleSphereFraction;
};

struct PoseOverlapGraph {
  std::vector<FrameInput> frames;
  std::vector<PoseFrameLayout> layout;
  cv::Mat mask;    // CV_8U NxN
  cv::Mat overlap; // CV_64F NxN
  PoseOverlapReport report;
};

/// Canonicalize by (sequenceIndex, imageFilename), infer variable rings from
/// manifest, and build the measured frustum-overlap pair graph.
PoseOverlapGraph buildPoseOverlapGraph(
    const std::vector<FrameInput> &frames,
    const std::string &rotationConvention = "capture_ref", int gridWidth = 720,
    int gridHeight = 360, double minimumCrossOverlap = 0.15,
    int crossNeighbors = 2);

/// Sort frames by (sequenceIndex, imageFilename) and reject duplicates.
std::vector<FrameInput>
canonicalizeFrames(const std::vector<FrameInput> &frames);

} // namespace sphera
