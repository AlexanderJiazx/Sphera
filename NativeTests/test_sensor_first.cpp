#include "SpheraAdaptiveRingSeam.hpp"
#include "SpheraEngineMath.hpp"
#include "SpheraPanoramaEngine.hpp"
#include "SpheraPoseOverlap.hpp"
#include "SpheraRotationRefinement.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#pragma clang diagnostic pop

namespace {

int failures = 0;

void expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    ++failures;
  } else {
    std::cout << "PASS: " << message << "\n";
  }
}

void expectNear(double actual, double expected, double tolerance,
                const char *message) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "FAIL: " << message << " actual=" << actual
              << " expected=" << expected << "\n";
    ++failures;
  } else {
    std::cout << "PASS: " << message << "\n";
  }
}

sphera::FrameInput makeSyntheticFrame(int sequenceIndex, const std::string &name,
                                      sphera::CaptureRing ring, int ringIndex,
                                      int ringCount, double yaw, double pitch) {
  sphera::FrameInput frame;
  frame.imageFilename = name;
  frame.sequenceIndex = sequenceIndex;
  frame.ring = ring;
  frame.ringIndex = ringIndex;
  frame.ringCount = ringCount;
  frame.yawDegrees = yaw;
  frame.pitchDegrees = pitch;
  frame.exifOrientation = 1;
  frame.intrinsics.fx = 800;
  frame.intrinsics.fy = 800;
  frame.intrinsics.cx = 320;
  frame.intrinsics.cy = 240;
  frame.intrinsics.referenceWidth = 640;
  frame.intrinsics.referenceHeight = 480;
  // Identity iOS camera→reference, then rotated about gravity via yaw/pitch.
  const double yawRad = yaw * CV_PI / 180.0;
  const double pitchRad = pitch * CV_PI / 180.0;
  const cv::Matx33d yawR(std::cos(yawRad), 0, std::sin(yawRad), 0, 1, 0,
                         -std::sin(yawRad), 0, std::cos(yawRad));
  const cv::Matx33d pitchR(1, 0, 0, 0, std::cos(pitchRad), -std::sin(pitchRad),
                           0, std::sin(pitchRad), std::cos(pitchRad));
  const cv::Mat rotation = cv::Mat(yawR) * cv::Mat(pitchR);
  frame.cameraToCaptureReferenceRotation =
      sphera::rowMajor9FromMat(rotation);
  frame.imagePath = name; // unused for graph-only tests
  return frame;
}

std::vector<sphera::FrameInput> makeRingLayout(const std::vector<int> &ringSizes,
                                               const std::vector<double> &pitches) {
  std::vector<sphera::FrameInput> frames;
  int sequence = 0;
  const std::vector<sphera::CaptureRing> rings = {
      sphera::CaptureRing::horizontal, sphera::CaptureRing::downward,
      sphera::CaptureRing::upward};
  for (std::size_t ring = 0; ring < ringSizes.size(); ++ring) {
    const int count = ringSizes[ring];
    for (int index = 0; index < count; ++index) {
      const double yaw = 360.0 * index / count;
      const std::string name = "ring" + std::to_string(ring) + "_" +
                               std::to_string(index) + ".jpg";
      frames.push_back(makeSyntheticFrame(sequence++, name, rings[ring], index,
                                          count, yaw, pitches[ring]));
    }
  }
  return frames;
}

void testCaptureRefGolden() {
  std::array<double, 9> identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const cv::Mat result = sphera::iosToOpenCVRotationCaptureRef(identity);
  // diag(1,-1,-1) @ I = diag(1,-1,-1)
  expectNear(result.at<double>(0, 0), 1.0, 1e-9, "capture_ref (0,0)");
  expectNear(result.at<double>(1, 1), -1.0, 1e-9, "capture_ref (1,1)");
  expectNear(result.at<double>(2, 2), -1.0, 1e-9, "capture_ref (2,2)");
  expectNear(cv::determinant(result), 1.0, 1e-9, "capture_ref det=+1");

  const cv::Mat ray = result * (cv::Mat_<double>(3, 1) << 0, 0, 1);
  expectNear(ray.at<double>(0), 0.0, 1e-9, "capture_ref forward x");
  expectNear(ray.at<double>(1), 0.0, 1e-9, "capture_ref forward y");
  expectNear(ray.at<double>(2), -1.0, 1e-9, "capture_ref forward z flipped");
}

void testPairGraphInvariance() {
  auto frames = makeRingLayout({8, 5, 5}, {0.0, -55.0, 55.0});
  auto shuffled = frames;
  std::reverse(shuffled.begin(), shuffled.end());
  const auto graphA =
      sphera::buildPoseOverlapGraph(frames, "capture_ref", 180, 90);
  const auto graphB =
      sphera::buildPoseOverlapGraph(shuffled, "capture_ref", 180, 90);
  expect(graphA.report.selectedPairs.size() ==
             graphB.report.selectedPairs.size(),
         "pair count invariant under shuffle");
  expect(cv::countNonZero(graphA.mask != graphB.mask) == 0,
         "pair mask invariant under shuffle");
  expect(graphA.frames.size() == 18, "8/5/5 total frames");
}

void testArbitraryRingSizes() {
  auto layout866 = makeRingLayout({8, 6, 6}, {0.0, -50.0, 50.0});
  const auto graph =
      sphera::buildPoseOverlapGraph(layout866, "capture_ref", 120, 60);
  expect(graph.frames.size() == 20, "8/6/6 total frames");
  expect(!graph.report.selectedPairs.empty(), "8/6/6 has pairs");
}

void testCorrectionCapsAndUnconstrained() {
  std::vector<cv::Mat> seeds = {cv::Mat::eye(3, 3, CV_64F),
                                cv::Mat::eye(3, 3, CV_64F)};
  std::vector<sphera::RayPairConstraint> empty;
  std::vector<std::pair<int, int>> adjacent;
  auto solved =
      sphera::solveSensorAnchoredRayBundle(seeds, empty, adjacent);
  expect(solved.second.unconstrainedCameraIndices.size() == 2,
         "empty constraints leave all unconstrained");
  expectNear(solved.second.cameraCorrectionDegrees[0], 0.0, 1e-12,
             "unconstrained camera zero correction");
  expectNear(solved.second.maximumCameraCorrectionDegrees, 0.0, 1e-12,
             "max correction zero without constraints");

  cv::Mat candidate = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat delta;
  cv::Mat axisAngle = (cv::Mat_<double>(3, 1) << 0.3, 0, 0); // ~17°
  cv::Rodrigues(axisAngle, delta);
  candidate = delta * candidate;
  const cv::Mat limited =
      sphera::limitTotalCorrection(candidate, seeds[0], 6.0);
  expectNear(sphera::rotationAngleDegrees(limited * seeds[0].t()), 6.0, 1e-4,
             "correction cap at 6 degrees");
}

void testCorruptedPairSwitchRejection() {
  // Two cameras with a deliberately inconsistent ray pair should drive switch
  // toward zero without large camera motion when disagreement is huge.
  std::vector<cv::Mat> seeds = {cv::Mat::eye(3, 3, CV_64F),
                                cv::Mat::eye(3, 3, CV_64F)};
  sphera::RayPairConstraint edge;
  edge.source = 0;
  edge.target = 1;
  edge.sourceRays = (cv::Mat_<double>(4, 3) << 0, 0, 1, 0.1, 0, 0.995, -0.1, 0,
                     0.995, 0, 0.1, 0.995);
  for (int row = 0; row < edge.sourceRays.rows; ++row) {
    cv::Vec3d ray(edge.sourceRays.at<double>(row, 0),
                  edge.sourceRays.at<double>(row, 1),
                  edge.sourceRays.at<double>(row, 2));
    ray *= 1.0 / std::sqrt(ray.dot(ray));
    edge.sourceRays.at<double>(row, 0) = ray[0];
    edge.sourceRays.at<double>(row, 1) = ray[1];
    edge.sourceRays.at<double>(row, 2) = ray[2];
  }
  // Target rays point the opposite way → corrupt pair.
  edge.targetRays = -edge.sourceRays;
  edge.confidence = cv::Mat::ones(4, 1, CV_64F);
  edge.baseWeight = 1.0;
  edge.inliers = 4;
  edge.matches = 4;
  edge.inlierRatio = 1.0;
  edge.spatialCoverage = 0.5;
  edge.predictedOverlap = 0.4;
  auto solved = sphera::solveSensorAnchoredRayBundle(
      seeds, {edge}, {{0, 1}});
  expect(!solved.second.pairSwitches.empty(), "corrupt pair produces switch");
  expect(solved.second.pairSwitches.front() < 0.25,
         "corrupt pair switch rejects pair");
  expect(solved.second.maximumCameraCorrectionDegrees < 1.0,
         "corrupt pair does not drag cameras globally");
}

void testPeriodicSeamClosure() {
  cv::Mat cost(9, 40, CV_32F, cv::Scalar(8.0f));
  cost.row(4).setTo(0.0f);
  cost(cv::Rect(18, 4, 5, 1)).setTo(20.0f);
  cost(cv::Rect(18, 3, 5, 1)).setTo(0.2f);
  const cv::Mat path =
      sphera::periodicMinimumCostPath(cost, 1, 0.25);
  expect(path.rows == 40, "path width");
  expect(std::abs(path.at<int>(0) - path.at<int>(39)) <= 1,
         "periodic closure");
  bool smooth = true;
  for (int column = 1; column < path.rows; ++column) {
    if (std::abs(path.at<int>(column) - path.at<int>(column - 1)) > 1) {
      smooth = false;
    }
  }
  expect(smooth, "path step limit honored");
}

void testCoverageRestoration() {
  std::vector<cv::Mat> masks = {
      cv::Mat(60, 80, CV_8U, cv::Scalar(255)),
      cv::Mat(60, 80, CV_8U, cv::Scalar(255)),
      cv::Mat(60, 80, CV_8U, cv::Scalar(255)),
      cv::Mat(60, 80, CV_8U, cv::Scalar(255)),
  };
  std::vector<cv::Mat> images;
  for (int index = 0; index < 4; ++index) {
    images.emplace_back(60, 80, CV_8UC3, cv::Scalar(70 + 20 * index, 80, 90));
  }
  std::vector<cv::Point> corners = {{0, 0}, {0, 0}, {0, 20}, {0, 20}};
  std::vector<sphera::PoseFrameLayout> layout(4);
  layout[0].ring = 0;
  layout[1].ring = 0;
  layout[2].ring = 1;
  layout[3].ring = 1;
  auto result = sphera::applyAdaptiveRingSeamPriors(masks, corners, images,
                                                    layout, 0.5);
  cv::Mat coverage = cv::Mat::zeros(80, 80, CV_8U);
  for (std::size_t index = 0; index < result.first.size(); ++index) {
    const cv::Mat &mask = result.first[index];
    const int top = corners[index].y;
    for (int row = 0; row < mask.rows; ++row) {
      for (int column = 0; column < mask.cols; ++column) {
        if (mask.at<uchar>(row, column) != 0) {
          coverage.at<uchar>(top + row, column) = 255;
        }
      }
    }
  }
  expect(cv::countNonZero(coverage == 0) == 0, "adaptive seam restores coverage");
  expect(result.second.mode == "adaptive-periodic-structure-path",
         "adaptive seam mode");
}

void testGateHelpers() {
  cv::Mat points(10, 1, CV_32FC2);
  for (int index = 0; index < 10; ++index) {
    points.at<cv::Point2f>(index) = cv::Point2f(float(index * 20), float(index * 10));
  }
  const double coverage = sphera::gridCoverage(points, cv::Size(200, 100));
  expect(coverage > 0.1, "grid coverage helper");
  sphera::SensorRayGateSettings settings;
  expectNear(settings.minimumPredictedOverlap, 0.15, 1e-12, "overlap gate");
  expect(settings.minimumInliers == 30, "inlier gate");
  expectNear(settings.minimumInlierRatio, 0.35, 1e-12, "inlier ratio gate");
  expectNear(settings.minimumSpatialCoverage, 0.12, 1e-12, "spatial gate");
  expectNear(settings.maximumSensorMedianDegrees, 2.0, 1e-12, "median gate");
  expectNear(settings.maximumSensorP90Degrees, 5.0, 1e-12, "p90 gate");
  expectNear(settings.maximumRelativeDisagreementDegrees, 8.0, 1e-12,
             "relative gate");
}

} // namespace

int main() {
  try {
    testCaptureRefGolden();
    testPairGraphInvariance();
    testArbitraryRingSizes();
    testCorrectionCapsAndUnconstrained();
    testCorruptedPairSwitchRejection();
    testPeriodicSeamClosure();
    testCoverageRestoration();
    testGateHelpers();
  } catch (const std::exception &exception) {
    std::cerr << "EXCEPTION: " << exception.what() << "\n";
    return 1;
  }
  if (failures > 0) {
    std::cerr << failures << " failure(s)\n";
    return 1;
  }
  std::cout << "All sensor-first native tests passed.\n";
  return 0;
}
