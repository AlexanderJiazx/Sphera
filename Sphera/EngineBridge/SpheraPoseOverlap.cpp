#include "SpheraPoseOverlap.hpp"

#include "SpheraEngineMath.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace sphera {
namespace {

std::string ringNameOf(const FrameInput &frame) {
  switch (frame.ring) {
  case CaptureRing::horizontal:
    return "horizontal";
  case CaptureRing::downward:
    return "downward";
  case CaptureRing::upward:
    return "upward";
  }
}

void sphereGrid(int width, int height, cv::Mat &rays, cv::Mat &weights) {
  rays.create(height * width, 3, CV_64F);
  weights.create(height * width, 1, CV_64F);
  int index = 0;
  for (int row = 0; row < height; ++row) {
    const double latitude =
        (0.5 - (static_cast<double>(row) + 0.5) / height) * CV_PI;
    const double cosLat = std::cos(latitude);
    const double sinLat = std::sin(latitude);
    for (int column = 0; column < width; ++column) {
      const double longitude =
          ((static_cast<double>(column) + 0.5) / width - 0.5) * (2.0 * CV_PI);
      rays.at<double>(index, 0) = std::sin(longitude) * cosLat;
      rays.at<double>(index, 1) = -sinLat;
      rays.at<double>(index, 2) = std::cos(longitude) * cosLat;
      weights.at<double>(index, 0) = cosLat;
      ++index;
    }
  }
}

cv::Mat visibleMask(const FrameInput &frame, const cv::Mat &worldRays,
                    const std::string &convention) {
  if (convention != "capture_ref") {
    throw std::runtime_error("Unsupported rotation convention: " + convention);
  }
  const cv::Mat cameraToWorld =
      iosToOpenCVRotationCaptureRef(frame.cameraToCaptureReferenceRotation);
  const cv::Mat worldToCamera = cameraToWorld.t();
  const int count = worldRays.rows;
  cv::Mat mask(count, 1, CV_8U);
  for (int index = 0; index < count; ++index) {
    const cv::Vec3d world(worldRays.at<double>(index, 0),
                          worldRays.at<double>(index, 1),
                          worldRays.at<double>(index, 2));
    const cv::Mat localMat = worldToCamera * cv::Mat(world);
    const double z = localMat.at<double>(2);
    const double safeZ = std::abs(z) > 1e-12 ? z : 1.0;
    const double x =
        frame.intrinsics.fx * localMat.at<double>(0) / safeZ +
        frame.intrinsics.cx;
    const double y =
        frame.intrinsics.fy * localMat.at<double>(1) / safeZ +
        frame.intrinsics.cy;
    const bool visible =
        z > 0.0 && x >= -0.5 &&
        x <= frame.intrinsics.referenceWidth - 0.5 && y >= -0.5 &&
        y <= frame.intrinsics.referenceHeight - 0.5;
    mask.at<uchar>(index) = visible ? 1 : 0;
  }
  return mask;
}

double weightedFraction(const cv::Mat &mask, const cv::Mat &weights) {
  double numerator = 0;
  double denominator = 0;
  for (int index = 0; index < mask.rows; ++index) {
    const double weight = weights.at<double>(index, 0);
    denominator += weight;
    if (mask.at<uchar>(index) != 0) {
      numerator += weight;
    }
  }
  return numerator / std::max(denominator, 1e-12);
}

} // namespace

std::vector<FrameInput>
canonicalizeFrames(const std::vector<FrameInput> &frames) {
  std::map<std::string, FrameInput> keyed;
  for (const FrameInput &frame : frames) {
    if (keyed.count(frame.imageFilename) != 0) {
      throw std::runtime_error(
          "Multiple input images resolve to metadata frame " +
          frame.imageFilename);
    }
    keyed.emplace(frame.imageFilename, frame);
  }
  std::vector<FrameInput> ordered;
  ordered.reserve(keyed.size());
  for (auto &entry : keyed) {
    ordered.push_back(std::move(entry.second));
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const FrameInput &left, const FrameInput &right) {
              if (left.sequenceIndex != right.sequenceIndex) {
                return left.sequenceIndex < right.sequenceIndex;
              }
              return left.imageFilename < right.imageFilename;
            });
  return ordered;
}

PoseOverlapGraph buildPoseOverlapGraph(const std::vector<FrameInput> &frames,
                                       const std::string &rotationConvention,
                                       int gridWidth, int gridHeight,
                                       double minimumCrossOverlap,
                                       int crossNeighbors) {
  PoseOverlapGraph graph;
  graph.frames = canonicalizeFrames(frames);
  const int count = static_cast<int>(graph.frames.size());
  if (count < 2) {
    throw std::runtime_error("Pose-overlap graph needs at least two frames");
  }

  cv::Mat worldRays;
  cv::Mat weights;
  sphereGrid(gridWidth, gridHeight, worldRays, weights);

  std::vector<cv::Mat> visibility(static_cast<std::size_t>(count));
  std::vector<double> visibleWeight(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    visibility[static_cast<std::size_t>(index)] =
        visibleMask(graph.frames[static_cast<std::size_t>(index)], worldRays,
                    rotationConvention);
    double sum = 0;
    for (int pixel = 0; pixel < visibility[static_cast<std::size_t>(index)].rows;
         ++pixel) {
      if (visibility[static_cast<std::size_t>(index)].at<uchar>(pixel) != 0) {
        sum += weights.at<double>(pixel, 0);
      }
    }
    visibleWeight[static_cast<std::size_t>(index)] = sum;
  }

  graph.overlap = cv::Mat::zeros(count, count, CV_64F);
  for (int left = 0; left < count; ++left) {
    graph.overlap.at<double>(left, left) = 1.0;
    for (int right = left + 1; right < count; ++right) {
      double intersection = 0;
      for (int pixel = 0; pixel < weights.rows; ++pixel) {
        if (visibility[static_cast<std::size_t>(left)].at<uchar>(pixel) != 0 &&
            visibility[static_cast<std::size_t>(right)].at<uchar>(pixel) != 0) {
          intersection += weights.at<double>(pixel, 0);
        }
      }
      const double denominator = std::max(
          std::min(visibleWeight[static_cast<std::size_t>(left)],
                   visibleWeight[static_cast<std::size_t>(right)]),
          1e-12);
      const double value = intersection / denominator;
      graph.overlap.at<double>(left, right) = value;
      graph.overlap.at<double>(right, left) = value;
    }
  }

  std::vector<std::string> ringNames(static_cast<std::size_t>(count));
  std::map<std::string, int> firstSequence;
  std::map<std::string, std::vector<double>> targetPitch;
  for (int index = 0; index < count; ++index) {
    const FrameInput &frame = graph.frames[static_cast<std::size_t>(index)];
    const std::string name = ringNameOf(frame);
    ringNames[static_cast<std::size_t>(index)] = name;
    if (firstSequence.count(name) == 0) {
      firstSequence[name] = frame.sequenceIndex;
    } else {
      firstSequence[name] = std::min(firstSequence[name], frame.sequenceIndex);
    }
    targetPitch[name].push_back(frame.pitchDegrees);
  }

  std::vector<std::string> orderedRingNames;
  for (const auto &entry : firstSequence) {
    orderedRingNames.push_back(entry.first);
  }
  std::sort(orderedRingNames.begin(), orderedRingNames.end(),
            [&](const std::string &left, const std::string &right) {
              return firstSequence[left] < firstSequence[right];
            });

  std::unordered_map<std::string, int> ringNumber;
  for (int index = 0; index < static_cast<int>(orderedRingNames.size());
       ++index) {
    ringNumber[orderedRingNames[static_cast<std::size_t>(index)]] = index;
  }

  std::map<std::string, std::vector<int>> ringMembers;
  for (int index = 0; index < count; ++index) {
    ringMembers[ringNames[static_cast<std::size_t>(index)]].push_back(index);
  }

  std::vector<std::pair<double, double>> yawPitch(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const cv::Mat rotation = iosToOpenCVRotationCaptureRef(
        graph.frames[static_cast<std::size_t>(index)]
            .cameraToCaptureReferenceRotation);
    yawPitch[static_cast<std::size_t>(index)] =
        yawPitchFromCameraToWorld(rotation);
  }

  std::unordered_map<int, int> localRank;
  for (const auto &entry : ringMembers) {
    std::vector<int> ordered = entry.second;
    std::sort(ordered.begin(), ordered.end(), [&](int left, int right) {
      const double leftYaw =
          std::fmod(yawPitch[static_cast<std::size_t>(left)].first, 360.0);
      const double rightYaw =
          std::fmod(yawPitch[static_cast<std::size_t>(right)].first, 360.0);
      const double leftNorm = leftYaw < 0 ? leftYaw + 360.0 : leftYaw;
      const double rightNorm = rightYaw < 0 ? rightYaw + 360.0 : rightYaw;
      if (leftNorm != rightNorm) {
        return leftNorm < rightNorm;
      }
      return graph.frames[static_cast<std::size_t>(left)].sequenceIndex <
             graph.frames[static_cast<std::size_t>(right)].sequenceIndex;
    });
    for (int rank = 0; rank < static_cast<int>(ordered.size()); ++rank) {
      localRank[ordered[static_cast<std::size_t>(rank)]] = rank;
    }
  }

  graph.layout.resize(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const std::string &name = ringNames[static_cast<std::size_t>(index)];
    PoseFrameLayout layout;
    layout.ring = ringNumber[name];
    layout.localIndex = localRank[index];
    layout.ringSize = static_cast<int>(ringMembers[name].size());
    layout.direction = 1;
    const double yaw =
        std::fmod(yawPitch[static_cast<std::size_t>(index)].first, 360.0);
    const double yawNorm = yaw < 0 ? yaw + 360.0 : yaw;
    layout.phase = yawNorm / 360.0;
    layout.ringName = name;
    layout.sensorYawDegrees = yawPitch[static_cast<std::size_t>(index)].first;
    layout.sensorPitchDegrees = yawPitch[static_cast<std::size_t>(index)].second;
    graph.layout[static_cast<std::size_t>(index)] = layout;
  }

  std::map<std::pair<int, int>, std::set<std::string>> selected;
  for (const auto &entry : ringMembers) {
    std::vector<int> ordered = entry.second;
    std::sort(ordered.begin(), ordered.end(),
              [&](int left, int right) { return localRank[left] < localRank[right]; });
    if (ordered.size() < 2) {
      continue;
    }
    for (std::size_t offset = 0; offset < ordered.size(); ++offset) {
      const int source = ordered[offset];
      const int target = ordered[(offset + 1) % ordered.size()];
      const auto key = std::make_pair(std::min(source, target),
                                      std::max(source, target));
      selected[key].insert("same-ring:" + entry.first);
    }
  }

  std::vector<std::string> pitchOrder = orderedRingNames;
  std::sort(pitchOrder.begin(), pitchOrder.end(),
            [&](const std::string &left, const std::string &right) {
              const double leftMedian = medianOf(targetPitch[left]);
              const double rightMedian = medianOf(targetPitch[right]);
              return leftMedian < rightMedian;
            });

  for (std::size_t index = 0; index + 1 < pitchOrder.size(); ++index) {
    const std::string &firstName = pitchOrder[index];
    const std::string &secondName = pitchOrder[index + 1];
    const std::vector<int> &firstMembers = ringMembers[firstName];
    const std::vector<int> &secondMembers = ringMembers[secondName];
    const std::string reason =
        "cross-ring:" + firstName + "<->" + secondName;
    for (const auto &pair :
         {std::make_pair(firstMembers, secondMembers),
          std::make_pair(secondMembers, firstMembers)}) {
      for (int source : pair.first) {
        std::vector<std::pair<double, int>> candidates;
        for (int target : pair.second) {
          const double score = graph.overlap.at<double>(source, target);
          if (score >= minimumCrossOverlap) {
            candidates.emplace_back(score, target);
          }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [&](const auto &left, const auto &right) {
                    if (left.first != right.first) {
                      return left.first > right.first;
                    }
                    return graph.frames[static_cast<std::size_t>(left.second)]
                               .sequenceIndex <
                           graph.frames[static_cast<std::size_t>(right.second)]
                               .sequenceIndex;
                  });
        const int take =
            std::min(crossNeighbors, static_cast<int>(candidates.size()));
        for (int candidateIndex = 0; candidateIndex < take; ++candidateIndex) {
          const int target = candidates[static_cast<std::size_t>(candidateIndex)].second;
          const auto key = std::make_pair(std::min(source, target),
                                          std::max(source, target));
          selected[key].insert(reason);
        }
      }
    }
  }

  graph.mask = cv::Mat::zeros(count, count, CV_8U);
  for (const auto &entry : selected) {
    const int left = entry.first.first;
    const int right = entry.first.second;
    graph.mask.at<uchar>(left, right) = 1;
    graph.mask.at<uchar>(right, left) = 1;
    PoseOverlapPairRecord record;
    record.source = left;
    record.target = right;
    record.sourceFile = graph.frames[static_cast<std::size_t>(left)].imageFilename;
    record.targetFile =
        graph.frames[static_cast<std::size_t>(right)].imageFilename;
    record.sameRing = graph.layout[static_cast<std::size_t>(left)].ring ==
                      graph.layout[static_cast<std::size_t>(right)].ring;
    record.predictedOverlapFraction = graph.overlap.at<double>(left, right);
    record.selectionReasons.assign(entry.second.begin(), entry.second.end());
    std::sort(record.selectionReasons.begin(), record.selectionReasons.end());
    graph.report.selectedPairs.push_back(std::move(record));
  }
  std::sort(graph.report.selectedPairs.begin(), graph.report.selectedPairs.end(),
            [](const PoseOverlapPairRecord &left,
               const PoseOverlapPairRecord &right) {
              if (left.source != right.source) {
                return left.source < right.source;
              }
              return left.target < right.target;
            });

  cv::Mat coverageCount = cv::Mat::zeros(weights.rows, 1, CV_32S);
  for (int index = 0; index < count; ++index) {
    for (int pixel = 0; pixel < weights.rows; ++pixel) {
      if (visibility[static_cast<std::size_t>(index)].at<uchar>(pixel) != 0) {
        coverageCount.at<int>(pixel) += 1;
      }
    }
  }
  cv::Mat covered = cv::Mat::zeros(weights.rows, 1, CV_8U);
  cv::Mat multi = cv::Mat::zeros(weights.rows, 1, CV_8U);
  for (int pixel = 0; pixel < weights.rows; ++pixel) {
    if (coverageCount.at<int>(pixel) > 0) {
      covered.at<uchar>(pixel) = 1;
    }
    if (coverageCount.at<int>(pixel) >= 2) {
      multi.at<uchar>(pixel) = 1;
    }
  }

  graph.report.mode = "pose-overlap";
  graph.report.gridWidth = gridWidth;
  graph.report.gridHeight = gridHeight;
  graph.report.rotationConvention = rotationConvention;
  graph.report.minimumCrossOverlapFraction = minimumCrossOverlap;
  graph.report.crossNeighbors = crossNeighbors;
  graph.report.predictedFullSphereCoverageFraction =
      weightedFraction(covered, weights);
  graph.report.predictedTwoOrMoreCoverageFraction =
      weightedFraction(multi, weights);
  graph.report.ringOrder = orderedRingNames;
  graph.report.visibleSphereFraction.resize(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    graph.report.visibleSphereFraction[static_cast<std::size_t>(index)] =
        weightedFraction(visibility[static_cast<std::size_t>(index)], weights);
  }
  return graph;
}

} // namespace sphera
