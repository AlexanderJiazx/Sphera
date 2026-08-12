#include "SpheraDirectSphere.hpp"

#include "SpheraAdaptiveRingSeam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdocumentation-deprecated-sync"
#pragma clang diagnostic ignored "-Wquoted-include-in-framework-header"
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#pragma clang diagnostic pop

namespace sphera {
namespace {

constexpr double kPi = 3.14159265358979323846;

double medianOfDoubles(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
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

double percentileOfDoubles(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position =
      std::clamp(percentile, 0.0, 1.0) * (values.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - lower;
  return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

cv::Mat photographicLuminance(const cv::Mat &image) {
  cv::Mat result(image.rows, image.cols, CV_32F);
  for (int row = 0; row < image.rows; ++row) {
    const cv::Vec3b *sourceRow = image.ptr<cv::Vec3b>(row);
    float *resultRow = result.ptr<float>(row);
    for (int column = 0; column < image.cols; ++column) {
      const cv::Vec3b pixel = sourceRow[column];
      resultRow[column] = static_cast<float>(
          0.0722 * pixel[0] + 0.7152 * pixel[1] + 0.2126 * pixel[2]);
    }
  }
  return result;
}

struct LongitudeGainEstimate {
  bool accepted = false;
  bool rejectedByCapPressure = false;
  int supportedColumns = 0;
  std::vector<float> logGain;
  double minimum = 1.0;
  double maximum = 1.0;
  double p05 = 1.0;
  double p95 = 1.0;
};

struct PairwiseResponseFieldEstimate {
  bool accepted = false;
  int equationCount = 0;
  int pairCount = 0;
  double elapsedSeconds = 0.0;
  double medianBefore = 0.0;
  double medianAfter = 0.0;
  double p90Before = 0.0;
  double p90After = 0.0;
  std::vector<cv::Vec2d> gainRanges;
};

struct ResponsePairPlan {
  int first = 0;
  int second = 0;
  int reliablePixels = 0;
  int sampleStep = 1;
  int sampleCount = 0;
};

struct OwnerTopologyPruneStats {
  int removedComponents = 0;
  int reassignedPixels = 0;
};

struct CentralPairSelection {
  bool accepted = false;
  int first = -1;
  int second = -1;
  double coverage = 0.0;
  double score = std::numeric_limits<double>::infinity();
  double elapsedSeconds = 0.0;
};

CentralPairSelection selectConnectedCentralPair(
    const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &validMasks,
    std::vector<cv::UMat> &selectedMasks, double radiusFraction) {
  CentralPairSelection selection;
  const int64 started = cv::getTickCount();
  if (images.size() < 2 || images.size() != validMasks.size() ||
      images.size() != selectedMasks.size() || !(radiusFraction > 0.0) ||
      !(radiusFraction < 0.5)) {
    return selection;
  }
  const int height = images[0].rows;
  const int width = images[0].cols;
  const double centerX = (width - 1) * 0.5;
  const double centerY = (height - 1) * 0.5;
  const double radius = std::min(width, height) * radiusFraction;
  cv::Mat disk(height, width, CV_8U, cv::Scalar(0));
  cv::circle(disk, cv::Point(static_cast<int>(std::lround(centerX)),
                             static_cast<int>(std::lround(centerY))),
             static_cast<int>(std::lround(radius)), cv::Scalar(255), -1,
             cv::LINE_8);
  const int diskPixels = cv::countNonZero(disk);
  std::vector<cv::Mat> lumas;
  std::vector<cv::Mat> textures;
  std::vector<cv::UMat> proxies;
  lumas.reserve(images.size());
  textures.reserve(images.size());
  proxies.resize(images.size());
  for (std::size_t index = 0; index < images.size(); ++index) {
    cv::Mat luma = photographicLuminance(images[index]);
    cv::Mat gradientX;
    cv::Mat gradientY;
    cv::Mat texture;
    cv::Sobel(luma, gradientX, CV_32F, 1, 0, 3);
    cv::Sobel(luma, gradientY, CV_32F, 0, 1, 3);
    cv::magnitude(gradientX, gradientY, texture);
    lumas.push_back(std::move(luma));
    textures.push_back(std::move(texture));
    cv::Mat proxy = structureSeamProxy(images[index]);
    proxy.convertTo(proxies[index], CV_32F);
  }

  std::vector<cv::Mat> bestPairMasks;
  for (int first = 0; first < static_cast<int>(images.size()); ++first) {
    for (int second = first + 1; second < static_cast<int>(images.size());
         ++second) {
      cv::Mat unionValid;
      cv::bitwise_or(validMasks[static_cast<std::size_t>(first)],
                     validMasks[static_cast<std::size_t>(second)], unionValid);
      cv::Mat covered;
      cv::bitwise_and(unionValid, disk, covered);
      const double coverage =
          cv::countNonZero(covered) / std::max(static_cast<double>(diskPixels), 1.0);
      if (coverage < 0.985) {
        continue;
      }
      std::vector<cv::UMat> pairProxies = {
          proxies[static_cast<std::size_t>(first)].clone(),
          proxies[static_cast<std::size_t>(second)].clone()};
      std::vector<cv::UMat> pairMasks(2);
      validMasks[static_cast<std::size_t>(first)].copyTo(pairMasks[0]);
      validMasks[static_cast<std::size_t>(second)].copyTo(pairMasks[1]);
      try {
        cv::detail::GraphCutSeamFinder pairFinder(
            cv::detail::GraphCutSeamFinderBase::COST_COLOR);
        pairFinder.find(pairProxies, {cv::Point(), cv::Point()}, pairMasks);
      } catch (const cv::Exception &) {
        continue;
      }
      const cv::Mat firstSelected =
          pairMasks[0].getMat(cv::ACCESS_READ).clone();
      const cv::Mat secondSelected =
          pairMasks[1].getMat(cv::ACCESS_READ).clone();
      cv::Mat owner(height, width, CV_8S, cv::Scalar(-1));
      owner.setTo(0, firstSelected);
      owner.setTo(1, secondSelected);

      int componentCount = 0;
      for (int localOwner = 0; localOwner < 2; ++localOwner) {
        cv::Mat region = owner == localOwner;
        cv::bitwise_and(region, disk, region);
        cv::Mat labels;
        componentCount +=
            std::max(0, cv::connectedComponents(region, labels, 8) - 1);
      }
      cv::Mat boundary(height, width, CV_8U, cv::Scalar(0));
      int seamPixels = 0;
      for (int row = 0; row < height; ++row) {
        const signed char *ownerRow = owner.ptr<signed char>(row);
        const signed char *previousOwnerRow =
            row > 0 ? owner.ptr<signed char>(row - 1) : nullptr;
        const uchar *diskRow = disk.ptr<uchar>(row);
        uchar *boundaryRow = boundary.ptr<uchar>(row);
        for (int column = 0; column < width; ++column) {
          if (diskRow[column] == 0 || ownerRow[column] < 0) {
            continue;
          }
          const bool horizontal =
              column > 0 && ownerRow[column - 1] >= 0 &&
              ownerRow[column] != ownerRow[column - 1];
          const bool vertical =
              row > 0 && previousOwnerRow[column] >= 0 &&
              ownerRow[column] != previousOwnerRow[column];
          if (horizontal || vertical) {
            boundaryRow[column] = 255;
            ++seamPixels;
          }
        }
      }
      cv::dilate(boundary, boundary, cv::Mat::ones(3, 3, CV_8U));
      cv::bitwise_and(boundary, disk, boundary);
      cv::bitwise_and(
          boundary, validMasks[static_cast<std::size_t>(first)], boundary);
      cv::bitwise_and(
          boundary, validMasks[static_cast<std::size_t>(second)], boundary);
      std::vector<double> logDifferences;
      std::vector<double> gradientDifferences;
      for (int row = 0; row < height; ++row) {
        const uchar *sampleRow = boundary.ptr<uchar>(row);
        const float *firstLumaRow =
            lumas[static_cast<std::size_t>(first)].ptr<float>(row);
        const float *secondLumaRow =
            lumas[static_cast<std::size_t>(second)].ptr<float>(row);
        const float *firstTextureRow =
            textures[static_cast<std::size_t>(first)].ptr<float>(row);
        const float *secondTextureRow =
            textures[static_cast<std::size_t>(second)].ptr<float>(row);
        for (int column = 0; column < width; ++column) {
          if (sampleRow[column] == 0) {
            continue;
          }
          logDifferences.push_back(std::abs(
              std::log(std::max(firstLumaRow[column], 4.0f)) -
              std::log(std::max(secondLumaRow[column], 4.0f))));
          gradientDifferences.push_back(
              std::abs(firstTextureRow[column] - secondTextureRow[column]));
        }
      }
      double score = 0.0;
      if (logDifferences.size() < 32) {
        cv::Mat firstDisk;
        cv::Mat secondDisk;
        cv::bitwise_and(firstSelected, disk, firstDisk);
        cv::bitwise_and(secondSelected, disk, secondDisk);
        const double singleOwnerCoverage =
            std::max(cv::countNonZero(firstDisk), cv::countNonZero(secondDisk)) /
            std::max(static_cast<double>(diskPixels), 1.0);
        if (singleOwnerCoverage < 0.985) {
          continue;
        }
        // If GraphCut can give one source the complete disk, that is even more
        // coherent than a two-source partition and requires no seam penalty.
        score = 0.02;
      } else {
        score = percentileOfDoubles(logDifferences, 0.90) +
                0.004 * percentileOfDoubles(gradientDifferences, 0.75) +
                0.0008 * seamPixels +
                0.08 * std::max(0, componentCount - 2);
      }
      if (score < selection.score) {
        selection.first = first;
        selection.second = second;
        selection.coverage = coverage;
        selection.score = score;
        bestPairMasks = {firstSelected, secondSelected};
      }
    }
  }
  if (selection.first >= 0 && selection.second >= 0 &&
      selection.score < 1.2 && bestPairMasks.size() == 2) {
    std::vector<cv::Mat> updated;
    updated.reserve(selectedMasks.size());
    for (cv::UMat &selectedMask : selectedMasks) {
      updated.push_back(selectedMask.getMat(cv::ACCESS_READ).clone());
      updated.back().setTo(0, disk);
    }
    bestPairMasks[0].copyTo(
        updated[static_cast<std::size_t>(selection.first)], disk);
    bestPairMasks[1].copyTo(
        updated[static_cast<std::size_t>(selection.second)], disk);
    for (std::size_t index = 0; index < selectedMasks.size(); ++index) {
      updated[index].copyTo(selectedMasks[index]);
    }
    selection.accepted = true;
  }
  selection.elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  return selection;
}

OwnerTopologyPruneStats pruneLowTextureOwnerIslands(
    std::vector<cv::UMat> &selectedMasks, const std::vector<cv::Mat> &validMasks,
    const std::vector<cv::Mat> &images) {
  OwnerTopologyPruneStats stats;
  if (selectedMasks.empty() || selectedMasks.size() != validMasks.size() ||
      selectedMasks.size() != images.size()) {
    return stats;
  }
  const int height = images[0].rows;
  const int width = images[0].cols;
  cv::Mat owner(height, width, CV_16S, cv::Scalar(-1));
  std::vector<cv::Mat> selected;
  selected.reserve(selectedMasks.size());
  for (std::size_t index = 0; index < selectedMasks.size(); ++index) {
    selected.push_back(selectedMasks[index].getMat(cv::ACCESS_READ).clone());
    owner.setTo(static_cast<int>(index), selected.back());
  }
  std::vector<cv::Mat> textures;
  std::vector<cv::Mat> edges;
  textures.reserve(images.size());
  edges.reserve(images.size());
  for (const cv::Mat &image : images) {
    const cv::Mat luma = photographicLuminance(image);
    cv::Mat gradientX;
    cv::Mat gradientY;
    cv::Mat texture;
    cv::Sobel(luma, gradientX, CV_32F, 1, 0, 3);
    cv::Sobel(luma, gradientY, CV_32F, 0, 1, 3);
    cv::magnitude(gradientX, gradientY, texture);
    textures.push_back(std::move(texture));
    cv::Mat edge;
    cv::Canny(image, edge, 70.0, 150.0);
    edges.push_back(std::move(edge));
  }

  const int minimumArea =
      std::max(20, static_cast<int>(std::lround(height * width * 0.003)));
  for (int source = 0; source < static_cast<int>(selectedMasks.size()); ++source) {
    cv::Mat sourceRegion = owner == source;
    cv::Mat componentLabels;
    cv::Mat componentStatistics;
    cv::Mat componentCentroids;
    const int componentCount = cv::connectedComponentsWithStats(
        sourceRegion, componentLabels, componentStatistics, componentCentroids,
        8, CV_32S);
    if (componentCount <= 2) {
      continue;
    }
    int largestArea = 0;
    for (int component = 1; component < componentCount; ++component) {
      largestArea = std::max(
          largestArea,
          componentStatistics.at<int>(component, cv::CC_STAT_AREA));
    }
    for (int component = 1; component < componentCount; ++component) {
      const int area =
          componentStatistics.at<int>(component, cv::CC_STAT_AREA);
      if (area >= largestArea ||
          area >= std::max(minimumArea,
                           static_cast<int>(std::lround(0.14 * largestArea)))) {
        continue;
      }
      cv::Mat region = componentLabels == component;
      std::vector<double> textureValues;
      textureValues.reserve(static_cast<std::size_t>(area));
      int edgePixels = 0;
      for (int row = 0; row < height; ++row) {
        const uchar *regionRow = region.ptr<uchar>(row);
        const float *textureRow =
            textures[static_cast<std::size_t>(source)].ptr<float>(row);
        const uchar *edgeRow =
            edges[static_cast<std::size_t>(source)].ptr<uchar>(row);
        for (int column = 0; column < width; ++column) {
          if (regionRow[column] == 0) {
            continue;
          }
          textureValues.push_back(textureRow[column]);
          edgePixels += edgeRow[column] != 0 ? 1 : 0;
        }
      }
      const double textureP75 = percentileOfDoubles(textureValues, 0.75);
      const double edgeFraction =
          edgePixels / std::max(static_cast<double>(area), 1.0);
      if (textureP75 > 22.0 || edgeFraction > 0.06) {
        continue;
      }

      cv::Mat distanceInput;
      region.convertTo(distanceInput, CV_8U);
      cv::Mat distance;
      cv::Mat nearestLabels;
      cv::distanceTransform(distanceInput, distance, nearestLabels, cv::DIST_L2,
                            5, cv::DIST_LABEL_PIXEL);
      double maximumLabelValue = 0.0;
      cv::minMaxLoc(nearestLabels, nullptr, &maximumLabelValue);
      std::vector<short> ownerByLabel(
          static_cast<std::size_t>(maximumLabelValue) + 1, -1);
      for (int row = 0; row < height; ++row) {
        const uchar *regionRow = region.ptr<uchar>(row);
        const int *nearestRow = nearestLabels.ptr<int>(row);
        const short *ownerRow = owner.ptr<short>(row);
        for (int column = 0; column < width; ++column) {
          if (regionRow[column] == 0 && ownerRow[column] >= 0) {
            ownerByLabel[static_cast<std::size_t>(nearestRow[column])] =
                ownerRow[column];
          }
        }
      }
      int replaceablePixels = 0;
      for (int row = 0; row < height; ++row) {
        const uchar *regionRow = region.ptr<uchar>(row);
        const int *nearestRow = nearestLabels.ptr<int>(row);
        for (int column = 0; column < width; ++column) {
          if (regionRow[column] == 0) {
            continue;
          }
          const short replacement =
              ownerByLabel[static_cast<std::size_t>(nearestRow[column])];
          if (replacement >= 0 &&
              validMasks[static_cast<std::size_t>(replacement)].at<uchar>(
                  row, column) != 0) {
            ++replaceablePixels;
          }
        }
      }
      if (replaceablePixels < static_cast<int>(std::ceil(0.985 * area))) {
        continue;
      }
      int changed = 0;
      for (int row = 0; row < height; ++row) {
        const uchar *regionRow = region.ptr<uchar>(row);
        const int *nearestRow = nearestLabels.ptr<int>(row);
        short *ownerRow = owner.ptr<short>(row);
        for (int column = 0; column < width; ++column) {
          if (regionRow[column] == 0) {
            continue;
          }
          const short replacement =
              ownerByLabel[static_cast<std::size_t>(nearestRow[column])];
          if (replacement >= 0 &&
              validMasks[static_cast<std::size_t>(replacement)].at<uchar>(
                  row, column) != 0) {
            ownerRow[column] = replacement;
            ++changed;
          }
        }
      }
      if (changed > 0) {
        ++stats.removedComponents;
        stats.reassignedPixels += changed;
      }
    }
  }
  if (stats.reassignedPixels > 0) {
    for (std::size_t index = 0; index < selectedMasks.size(); ++index) {
      cv::Mat updated = owner == static_cast<int>(index);
      updated.copyTo(selectedMasks[index]);
    }
  }
  return stats;
}

PairwiseResponseFieldEstimate equalizePairwiseResponseFields(
    std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks,
    const std::vector<cv::Mat> &normalizedSourceX,
    const std::vector<cv::Mat> &normalizedSourceY) {
  PairwiseResponseFieldEstimate estimate;
  const int64 started = cv::getTickCount();
  const int imageCount = static_cast<int>(images.size());
  estimate.gainRanges.assign(static_cast<std::size_t>(imageCount),
                             cv::Vec2d(1.0, 1.0));
  if (imageCount < 2 || masks.size() != images.size() ||
      normalizedSourceX.size() != images.size() ||
      normalizedSourceY.size() != images.size()) {
    return estimate;
  }

  std::vector<cv::Mat> lumas;
  std::vector<cv::Mat> logs;
  std::vector<cv::Mat> textures;
  std::vector<cv::Mat> interiors;
  std::vector<double> logCenters;
  lumas.reserve(images.size());
  logs.reserve(images.size());
  textures.reserve(images.size());
  interiors.reserve(images.size());
  logCenters.reserve(images.size());
  for (int index = 0; index < imageCount; ++index) {
    cv::Mat luma = photographicLuminance(images[static_cast<std::size_t>(index)]);
    cv::Mat logValue(luma.size(), CV_32F);
    std::vector<double> validLogs;
    validLogs.reserve(static_cast<std::size_t>(cv::countNonZero(
        masks[static_cast<std::size_t>(index)])));
    for (int row = 0; row < luma.rows; ++row) {
      const float *lumaRow = luma.ptr<float>(row);
      const uchar *validRow = masks[static_cast<std::size_t>(index)].ptr<uchar>(row);
      float *logRow = logValue.ptr<float>(row);
      for (int column = 0; column < luma.cols; ++column) {
        logRow[column] = std::log(std::max(lumaRow[column], 4.0f));
        if (validRow[column] != 0) {
          validLogs.push_back(logRow[column]);
        }
      }
    }
    if (validLogs.empty()) {
      estimate.elapsedSeconds =
          (cv::getTickCount() - started) / cv::getTickFrequency();
      return estimate;
    }
    cv::Mat gradientX;
    cv::Mat gradientY;
    cv::Mat texture;
    cv::Sobel(luma, gradientX, CV_32F, 1, 0, 3);
    cv::Sobel(luma, gradientY, CV_32F, 0, 1, 3);
    cv::magnitude(gradientX, gradientY, texture);
    cv::Mat interior;
    cv::erode(masks[static_cast<std::size_t>(index)], interior,
              cv::Mat::ones(9, 9, CV_8U));
    lumas.push_back(std::move(luma));
    logs.push_back(std::move(logValue));
    textures.push_back(std::move(texture));
    interiors.push_back(std::move(interior));
    logCenters.push_back(medianOfDoubles(std::move(validLogs)));
  }

  const auto usableAt = [&](int first, int second, int row, int column) {
    if (interiors[static_cast<std::size_t>(first)].at<uchar>(row, column) == 0 ||
        interiors[static_cast<std::size_t>(second)].at<uchar>(row, column) == 0) {
      return false;
    }
    const float firstLuma =
        lumas[static_cast<std::size_t>(first)].at<float>(row, column);
    const float secondLuma =
        lumas[static_cast<std::size_t>(second)].at<float>(row, column);
    const float firstLog =
        logs[static_cast<std::size_t>(first)].at<float>(row, column);
    const float secondLog =
        logs[static_cast<std::size_t>(second)].at<float>(row, column);
    return firstLuma > 12.0f && firstLuma < 238.0f && secondLuma > 12.0f &&
           secondLuma < 238.0f &&
           textures[static_cast<std::size_t>(first)].at<float>(row, column) <
               24.0f &&
           textures[static_cast<std::size_t>(second)].at<float>(row, column) <
               24.0f &&
           std::abs(firstLog - secondLog) < 0.75f;
  };

  constexpr int kMaximumPairSamples = 18000;
  std::vector<ResponsePairPlan> plans;
  std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(imageCount));
  int totalSamples = 0;
  for (int first = 0; first < imageCount; ++first) {
    for (int second = first + 1; second < imageCount; ++second) {
      int reliablePixels = 0;
      for (int row = 0; row < images[0].rows; ++row) {
        for (int column = 0; column < images[0].cols; ++column) {
          if (usableAt(first, second, row, column)) {
            ++reliablePixels;
          }
        }
      }
      if (reliablePixels < 256) {
        continue;
      }
      ResponsePairPlan plan;
      plan.first = first;
      plan.second = second;
      plan.reliablePixels = reliablePixels;
      plan.sampleStep =
          std::max(1, (reliablePixels + kMaximumPairSamples - 1) /
                          kMaximumPairSamples);
      plan.sampleCount =
          (reliablePixels + plan.sampleStep - 1) / plan.sampleStep;
      totalSamples += plan.sampleCount;
      plans.push_back(plan);
      adjacency[static_cast<std::size_t>(first)].push_back(second);
      adjacency[static_cast<std::size_t>(second)].push_back(first);
    }
  }
  if (plans.empty()) {
    estimate.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return estimate;
  }
  std::vector<bool> visited(static_cast<std::size_t>(imageCount), false);
  std::vector<int> pending(1, 0);
  visited[0] = true;
  while (!pending.empty()) {
    const int current = pending.back();
    pending.pop_back();
    for (int neighbour : adjacency[static_cast<std::size_t>(current)]) {
      if (!visited[static_cast<std::size_t>(neighbour)]) {
        visited[static_cast<std::size_t>(neighbour)] = true;
        pending.push_back(neighbour);
      }
    }
  }
  if (std::find(visited.begin(), visited.end(), false) != visited.end()) {
    estimate.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return estimate;
  }

  constexpr int kBasisCount = 7;
  const int parameterCount = imageCount * kBasisCount;
  cv::Mat design = cv::Mat::zeros(totalSamples, parameterCount, CV_64F);
  cv::Mat observed(totalSamples, 1, CV_64F);
  cv::Mat baseWeights(totalSamples, 1, CV_64F);
  int sampleIndex = 0;
  for (const ResponsePairPlan &plan : plans) {
    int usableOrdinal = 0;
    for (int row = 0; row < images[0].rows; ++row) {
      for (int column = 0; column < images[0].cols; ++column) {
        if (!usableAt(plan.first, plan.second, row, column)) {
          continue;
        }
        const bool selected = usableOrdinal % plan.sampleStep == 0;
        ++usableOrdinal;
        if (!selected) {
          continue;
        }
        double *designRow = design.ptr<double>(sampleIndex);
        const int firstOffset = plan.first * kBasisCount;
        const int secondOffset = plan.second * kBasisCount;
        const float firstX = normalizedSourceX[static_cast<std::size_t>(plan.first)]
                                 .at<float>(row, column);
        const float firstY = normalizedSourceY[static_cast<std::size_t>(plan.first)]
                                 .at<float>(row, column);
        const float secondX = normalizedSourceX[static_cast<std::size_t>(plan.second)]
                                  .at<float>(row, column);
        const float secondY = normalizedSourceY[static_cast<std::size_t>(plan.second)]
                                  .at<float>(row, column);
        const double firstTone =
            logs[static_cast<std::size_t>(plan.first)].at<float>(row, column) -
            logCenters[static_cast<std::size_t>(plan.first)];
        const double secondTone =
            logs[static_cast<std::size_t>(plan.second)].at<float>(row, column) -
            logCenters[static_cast<std::size_t>(plan.second)];
        const double firstBasis[kBasisCount] = {
            1.0, firstX, firstY, firstX * firstX, firstX * firstY,
            firstY * firstY, firstTone};
        const double secondBasis[kBasisCount] = {
            1.0, secondX, secondY, secondX * secondX, secondX * secondY,
            secondY * secondY, secondTone};
        for (int basis = 0; basis < kBasisCount; ++basis) {
          designRow[firstOffset + basis] = firstBasis[basis];
          designRow[secondOffset + basis] = -secondBasis[basis];
        }
        const float firstLog =
            logs[static_cast<std::size_t>(plan.first)].at<float>(row, column);
        const float secondLog =
            logs[static_cast<std::size_t>(plan.second)].at<float>(row, column);
        observed.at<double>(sampleIndex) = secondLog - firstLog;
        const float texture = std::max(
            textures[static_cast<std::size_t>(plan.first)].at<float>(row, column),
            textures[static_cast<std::size_t>(plan.second)].at<float>(row, column));
        const double normalizedTexture = texture / 8.0;
        baseWeights.at<double>(sampleIndex) =
            1.0 / (1.0 + normalizedTexture * normalizedTexture);
        ++sampleIndex;
      }
    }
  }
  if (sampleIndex != totalSamples) {
    design = design.rowRange(0, sampleIndex).clone();
    observed = observed.rowRange(0, sampleIndex).clone();
    baseWeights = baseWeights.rowRange(0, sampleIndex).clone();
    totalSamples = sampleIndex;
  }
  estimate.equationCount = totalSamples;
  estimate.pairCount = static_cast<int>(plans.size());

  cv::Mat robustWeights(totalSamples, 1, CV_64F, cv::Scalar(1.0));
  cv::Mat coefficients(parameterCount, 1, CV_64F, cv::Scalar(0.0));
  cv::Mat predicted;
  const double regularizationByBasis[kBasisCount] = {
      2e-6, 2e-4, 2e-4, 8e-4, 1.2e-3, 8e-4, 4e-4};
  bool solved = false;
  for (int iteration = 0; iteration < 8; ++iteration) {
    cv::Mat weightedDesign = design.clone();
    cv::Mat weightedObserved(totalSamples, 1, CV_64F);
    double weightSum = 0.0;
    for (int row = 0; row < totalSamples; ++row) {
      const double weight = baseWeights.at<double>(row) *
                            robustWeights.at<double>(row);
      const double squareRootWeight = std::sqrt(std::max(weight, 0.0));
      weightedDesign.row(row) *= squareRootWeight;
      weightedObserved.at<double>(row) =
          observed.at<double>(row) * squareRootWeight;
      weightSum += weight;
    }
    weightSum = std::max(weightSum, 1.0);
    cv::Mat normal;
    cv::Mat rhs;
    cv::gemm(weightedDesign, weightedDesign, 1.0, cv::noArray(), 0.0, normal,
             cv::GEMM_1_T);
    cv::gemm(weightedDesign, weightedObserved, 1.0, cv::noArray(), 0.0, rhs,
             cv::GEMM_1_T);
    for (int parameter = 0; parameter < parameterCount; ++parameter) {
      normal.at<double>(parameter, parameter) +=
          regularizationByBasis[parameter % kBasisCount] * weightSum;
    }
    const double gaugeValue = 1.0 / imageCount;
    for (int first = 0; first < imageCount; ++first) {
      for (int second = 0; second < imageCount; ++second) {
        normal.at<double>(first * kBasisCount, second * kBasisCount) +=
            gaugeValue * gaugeValue * weightSum;
      }
    }
    solved = cv::solve(normal, rhs, coefficients, cv::DECOMP_CHOLESKY);
    if (!solved) {
      solved = cv::solve(normal, rhs, coefficients, cv::DECOMP_SVD);
    }
    if (!solved || !cv::checkRange(coefficients)) {
      break;
    }
    cv::gemm(design, coefficients, 1.0, cv::noArray(), 0.0, predicted);
    std::vector<double> absoluteResiduals;
    absoluteResiduals.reserve(static_cast<std::size_t>(totalSamples));
    for (int row = 0; row < totalSamples; ++row) {
      absoluteResiduals.push_back(
          std::abs(observed.at<double>(row) - predicted.at<double>(row)));
    }
    const double scale =
        std::max(1.4826 * medianOfDoubles(std::move(absoluteResiduals)), 0.006);
    for (int row = 0; row < totalSamples; ++row) {
      const double residual =
          std::abs(observed.at<double>(row) - predicted.at<double>(row));
      robustWeights.at<double>(row) =
          std::min(1.0, 1.5 * scale / std::max(residual, 1e-8));
    }
  }
  if (!solved || predicted.empty()) {
    estimate.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return estimate;
  }

  std::vector<double> absoluteBefore;
  std::vector<double> absoluteAfter;
  absoluteBefore.reserve(static_cast<std::size_t>(totalSamples));
  absoluteAfter.reserve(static_cast<std::size_t>(totalSamples));
  for (int row = 0; row < totalSamples; ++row) {
    absoluteBefore.push_back(std::abs(observed.at<double>(row)));
    absoluteAfter.push_back(
        std::abs(observed.at<double>(row) - predicted.at<double>(row)));
  }
  estimate.medianBefore = percentileOfDoubles(absoluteBefore, 0.5);
  estimate.medianAfter = percentileOfDoubles(absoluteAfter, 0.5);
  estimate.p90Before = percentileOfDoubles(absoluteBefore, 0.9);
  estimate.p90After = percentileOfDoubles(absoluteAfter, 0.9);
  estimate.accepted =
      estimate.medianAfter < 0.72 * std::max(estimate.medianBefore, 1e-8) &&
      estimate.p90After < 0.86 * estimate.p90Before;
  if (estimate.accepted) {
    const double minimumField = std::log(0.68);
    const double maximumField = std::log(1.47);
    for (int index = 0; index < imageCount; ++index) {
      cv::Mat &image = images[static_cast<std::size_t>(index)];
      double minimumGain = std::numeric_limits<double>::infinity();
      double maximumGain = 0.0;
      for (int row = 0; row < image.rows; ++row) {
        cv::Vec3b *imageRow = image.ptr<cv::Vec3b>(row);
        const uchar *validRow = masks[static_cast<std::size_t>(index)].ptr<uchar>(row);
        const float *xRow =
            normalizedSourceX[static_cast<std::size_t>(index)].ptr<float>(row);
        const float *yRow =
            normalizedSourceY[static_cast<std::size_t>(index)].ptr<float>(row);
        const float *logRow = logs[static_cast<std::size_t>(index)].ptr<float>(row);
        for (int column = 0; column < image.cols; ++column) {
          if (validRow[column] == 0) {
            imageRow[column] = cv::Vec3b(0, 0, 0);
            continue;
          }
          const double x = xRow[column];
          const double y = yRow[column];
          const double tone =
              logRow[column] - logCenters[static_cast<std::size_t>(index)];
          const int offset = index * kBasisCount;
          const double field = std::clamp(
              coefficients.at<double>(offset) +
                  coefficients.at<double>(offset + 1) * x +
                  coefficients.at<double>(offset + 2) * y +
                  coefficients.at<double>(offset + 3) * x * x +
                  coefficients.at<double>(offset + 4) * x * y +
                  coefficients.at<double>(offset + 5) * y * y +
                  coefficients.at<double>(offset + 6) * tone,
              minimumField, maximumField);
          const double gain = std::exp(field);
          minimumGain = std::min(minimumGain, gain);
          maximumGain = std::max(maximumGain, gain);
          for (int channel = 0; channel < 3; ++channel) {
            imageRow[column][channel] = static_cast<uchar>(std::clamp(
                std::lround(imageRow[column][channel] * gain), 0L, 255L));
          }
        }
      }
      estimate.gainRanges[static_cast<std::size_t>(index)] =
          cv::Vec2d(std::isfinite(minimumGain) ? minimumGain : 1.0,
                    maximumGain > 0.0 ? maximumGain : 1.0);
    }
  }
  estimate.elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  return estimate;
}

LongitudeGainEstimate estimateLongitudeBoundaryGain(
    const cv::Mat &reference, const cv::Mat &candidate,
    const cv::Mat &candidateMask, const std::vector<double> &latitudes,
    double minimumReplacementLatitudeDegrees) {
  LongitudeGainEstimate estimate;
  estimate.logGain.assign(static_cast<std::size_t>(reference.cols), 0.0f);

  const cv::Mat referenceLuma = photographicLuminance(reference);
  const cv::Mat candidateLuma = photographicLuminance(candidate);
  cv::Mat referenceGradientX;
  cv::Mat referenceGradientY;
  cv::Mat candidateGradientX;
  cv::Mat candidateGradientY;
  cv::Sobel(referenceLuma, referenceGradientX, CV_32F, 1, 0, 3);
  cv::Sobel(referenceLuma, referenceGradientY, CV_32F, 0, 1, 3);
  cv::Sobel(candidateLuma, candidateGradientX, CV_32F, 1, 0, 3);
  cv::Sobel(candidateLuma, candidateGradientY, CV_32F, 0, 1, 3);
  cv::Mat referenceTexture;
  cv::Mat candidateTexture;
  cv::magnitude(referenceGradientX, referenceGradientY, referenceTexture);
  cv::magnitude(candidateGradientX, candidateGradientY, candidateTexture);

  std::vector<double> columnValues(static_cast<std::size_t>(reference.cols),
                                   std::numeric_limits<double>::quiet_NaN());
  std::vector<double> columnWeights(static_cast<std::size_t>(reference.cols),
                                    0.0);
  for (int column = 0; column < reference.cols; ++column) {
    std::vector<double> values;
    for (int row = 0; row < reference.rows; ++row) {
      const double latitude =
          std::abs(latitudes[static_cast<std::size_t>(row)]);
      if (latitude < minimumReplacementLatitudeDegrees + 0.4 ||
          latitude > minimumReplacementLatitudeDegrees + 2.5 ||
          candidateMask.at<uchar>(row, column) == 0) {
        continue;
      }
      const float target = referenceLuma.at<float>(row, column);
      const float source = candidateLuma.at<float>(row, column);
      if (target <= 12.0f || target >= 238.0f || source <= 12.0f ||
          source >= 238.0f ||
          referenceTexture.at<float>(row, column) >= 28.0f ||
          candidateTexture.at<float>(row, column) >= 28.0f) {
        continue;
      }
      values.push_back(std::log(std::max(target, 4.0f)) -
                       std::log(std::max(source, 4.0f)));
    }
    if (values.size() < 3) {
      continue;
    }
    const double median = medianOfDoubles(values);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (double value : values) {
      deviations.push_back(std::abs(value - median));
    }
    const double deviation =
        std::max(1.4826 * medianOfDoubles(std::move(deviations)), 0.01);
    std::vector<double> inliers;
    inliers.reserve(values.size());
    for (double value : values) {
      if (std::abs(value - median) <= 2.5 * deviation) {
        inliers.push_back(value);
      }
    }
    if (inliers.size() < 3) {
      continue;
    }
    columnValues[static_cast<std::size_t>(column)] =
        medianOfDoubles(inliers);
    columnWeights[static_cast<std::size_t>(column)] =
        static_cast<double>(inliers.size());
    ++estimate.supportedColumns;
  }

  if (estimate.supportedColumns < std::max(64, reference.cols / 20)) {
    return estimate;
  }
  double weightedSum = 0.0;
  double weightSum = 0.0;
  for (int column = 0; column < reference.cols; ++column) {
    const double value = columnValues[static_cast<std::size_t>(column)];
    const double weight = columnWeights[static_cast<std::size_t>(column)];
    if (std::isfinite(value) && weight > 0.0) {
      weightedSum += value * weight;
      weightSum += weight;
    }
  }
  if (weightSum <= 0.0) {
    return estimate;
  }
  const double globalLogGain = weightedSum / weightSum;
  cv::Mat tiledNumerator(1, reference.cols * 3, CV_32F, cv::Scalar(0));
  cv::Mat tiledDenominator(1, reference.cols * 3, CV_32F, cv::Scalar(0));
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (int column = 0; column < reference.cols; ++column) {
      const double value = columnValues[static_cast<std::size_t>(column)];
      const double weight = columnWeights[static_cast<std::size_t>(column)];
      if (!std::isfinite(value) || weight <= 0.0) {
        continue;
      }
      const int targetColumn = repeat * reference.cols + column;
      tiledNumerator.at<float>(0, targetColumn) =
          static_cast<float>((value - globalLogGain) * weight);
      tiledDenominator.at<float>(0, targetColumn) = static_cast<float>(weight);
    }
  }
  const double sigma = std::max(reference.cols * 0.025, 12.0);
  cv::GaussianBlur(tiledNumerator, tiledNumerator, cv::Size(), sigma, 0.0);
  cv::GaussianBlur(tiledDenominator, tiledDenominator, cv::Size(), sigma, 0.0);

  const float minimumLogGain = static_cast<float>(std::log(0.88));
  const float maximumLogGain = static_cast<float>(std::log(1.14));
  std::vector<float> smoothedLogGain(
      static_cast<std::size_t>(reference.cols), 0.0f);
  int capPressureColumns = 0;
  for (int column = 0; column < reference.cols; ++column) {
    const int middleColumn = reference.cols + column;
    const float denominator = tiledDenominator.at<float>(0, middleColumn);
    const float smoothed = denominator > 1e-5f
                               ? tiledNumerator.at<float>(0, middleColumn) /
                                     denominator
                               : 0.0f;
    smoothedLogGain[static_cast<std::size_t>(column)] = smoothed;
    if (smoothed < minimumLogGain || smoothed > maximumLogGain) {
      ++capPressureColumns;
    }
  }
  // A mild exposure or lens-shading residual should naturally fit inside the
  // safety bounds. If a broad part of the field presses against a cap, the
  // estimator is probably following real indoor lighting, cloud structure, or
  // parallax instead. Reject the whole spatial component in that case.
  if (capPressureColumns > std::max(8, reference.cols / 100)) {
    estimate.rejectedByCapPressure = true;
    return estimate;
  }

  std::vector<double> spatialGains;
  spatialGains.reserve(static_cast<std::size_t>(reference.cols));
  for (int column = 0; column < reference.cols; ++column) {
    const float bounded = std::clamp(
        smoothedLogGain[static_cast<std::size_t>(column)], minimumLogGain,
        maximumLogGain);
    estimate.logGain[static_cast<std::size_t>(column)] = bounded;
    spatialGains.push_back(std::exp(static_cast<double>(bounded)));
  }
  estimate.minimum = *std::min_element(spatialGains.begin(), spatialGains.end());
  estimate.maximum = *std::max_element(spatialGains.begin(), spatialGains.end());
  estimate.p05 = percentileOfDoubles(spatialGains, 0.05);
  estimate.p95 = percentileOfDoubles(std::move(spatialGains), 0.95);
  estimate.accepted = true;
  return estimate;
}

} // namespace

static PolarCubeFaceStats composePolarCubeFace(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<PoseFrameLayout> &layouts, double workScale,
    double composeScale, int seamSize, int composeSize,
    double fieldOfViewDegrees, int blendBands,
    double fullReplacementLatitudeDegrees,
    double minimumReplacementLatitudeDegrees, const std::string &pole,
    bool requireResponseField, bool selectCentralPair) {
  PolarCubeFaceStats stats;
  stats.pole = pole;
  const bool topPole = pole == "top";
  if (!topPole && pole != "bottom") {
    throw std::runtime_error("Polar cube pole must be top or bottom");
  }
  const int64 started = cv::getTickCount();
  if (panorama.empty() || mask.empty() || panorama.type() != CV_8UC3 ||
      mask.type() != CV_8U || panorama.size() != mask.size()) {
    throw std::runtime_error(
        "Polar cube compositor requires matching CV_8UC3 panorama and CV_8U mask");
  }
  if (composeScaleImages.size() != cameras.size() ||
      composeScaleImages.size() != layouts.size()) {
    throw std::runtime_error("Polar cube image/camera/layout counts do not match");
  }
  if (seamSize <= 0 || composeSize <= 0 || blendBands <= 0 ||
      !(fieldOfViewDegrees > 0 && fieldOfViewDegrees < 180) ||
      !(minimumReplacementLatitudeDegrees >= 0 &&
        minimumReplacementLatitudeDegrees < fullReplacementLatitudeDegrees &&
        fullReplacementLatitudeDegrees <= 90)) {
    throw std::runtime_error("Invalid polar cube compositor configuration");
  }
  if (labels != nullptr &&
      (labels->empty() || labels->type() != CV_16S ||
       labels->size() != panorama.size())) {
    *labels = cv::Mat(panorama.size(), CV_16S, cv::Scalar(-1));
  }

  std::vector<int> sourceIndices;
  for (std::size_t index = 0; index < layouts.size(); ++index) {
    if (layouts[index].ringName == (topPole ? "upward" : "downward")) {
      sourceIndices.push_back(static_cast<int>(index));
    }
  }
  stats.sourceCount = static_cast<int>(sourceIndices.size());
  stats.selectedPixelsByInput.assign(cameras.size(), 0);
  stats.responseFieldGainRangesByInput.assign(cameras.size(),
                                               cv::Vec2d(1.0, 1.0));
  if (sourceIndices.size() < 2) {
    stats.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return stats;
  }

  const double scaleRatio = composeScale / std::max(workScale, 1e-12);
  std::vector<cv::Mat> intrinsics(cameras.size());
  std::vector<cv::Matx33f> worldToCamera(cameras.size());
  for (int globalIndex : sourceIndices) {
    if (composeScaleImages[static_cast<std::size_t>(globalIndex)].empty() ||
        composeScaleImages[static_cast<std::size_t>(globalIndex)].type() !=
            CV_8UC3) {
      throw std::runtime_error(
          "Polar cube compositor requires non-empty CV_8UC3 source images");
    }
    cv::Mat intrinsic = cameras[static_cast<std::size_t>(globalIndex)].K();
    intrinsic.convertTo(intrinsic, CV_64F);
    intrinsic.at<double>(0, 0) *= scaleRatio;
    intrinsic.at<double>(0, 2) *= scaleRatio;
    intrinsic.at<double>(1, 1) *= scaleRatio;
    intrinsic.at<double>(1, 2) *= scaleRatio;
    intrinsics[static_cast<std::size_t>(globalIndex)] = intrinsic;

    cv::Mat rotation;
    cameras[static_cast<std::size_t>(globalIndex)].R.convertTo(rotation,
                                                               CV_32F);
    cv::Mat rotationT = rotation.t();
    cv::Matx33f rotationMatrix;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotationMatrix(row, column) = rotationT.at<float>(row, column);
      }
    }
    worldToCamera[static_cast<std::size_t>(globalIndex)] = rotationMatrix;
  }

  const auto renderLayers = [&](int faceSize, std::vector<cv::Mat> &images,
                                std::vector<cv::Mat> &masks,
                                std::vector<cv::Mat> *normalizedSourceX = nullptr,
                                std::vector<cv::Mat> *normalizedSourceY = nullptr) {
    images.clear();
    masks.clear();
    images.reserve(sourceIndices.size());
    masks.reserve(sourceIndices.size());
    if (normalizedSourceX != nullptr) {
      normalizedSourceX->clear();
      normalizedSourceX->reserve(sourceIndices.size());
    }
    if (normalizedSourceY != nullptr) {
      normalizedSourceY->clear();
      normalizedSourceY->reserve(sourceIndices.size());
    }
    const double extent =
        std::tan(fieldOfViewDegrees * kPi / 180.0 * 0.5);
    std::vector<cv::Vec3f> world(static_cast<std::size_t>(faceSize * faceSize));
    for (int row = 0; row < faceSize; ++row) {
      const float localY = static_cast<float>(
          (((row + 0.5) / faceSize) * 2.0 - 1.0) * extent);
      for (int column = 0; column < faceSize; ++column) {
        const float localX = static_cast<float>(
            (((column + 0.5) / faceSize) * 2.0 - 1.0) * extent);
        const float inverseLength =
            1.0f / std::sqrt(localX * localX + localY * localY + 1.0f);
        // Top: local (x,y,z) -> world (x,-z,y).
        // Bottom: local (x,y,z) -> world (x,z,-y).
        world[static_cast<std::size_t>(row * faceSize + column)] = topPole
            ? cv::Vec3f(localX * inverseLength, -inverseLength,
                        localY * inverseLength)
            : cv::Vec3f(localX * inverseLength, inverseLength,
                        -localY * inverseLength);
      }
    }

    for (int globalIndex : sourceIndices) {
      const cv::Mat &source =
          composeScaleImages[static_cast<std::size_t>(globalIndex)];
      const cv::Mat &intrinsic = intrinsics[static_cast<std::size_t>(globalIndex)];
      const cv::Matx33f &rotationT =
          worldToCamera[static_cast<std::size_t>(globalIndex)];
      const float fx = static_cast<float>(intrinsic.at<double>(0, 0));
      const float fy = static_cast<float>(intrinsic.at<double>(1, 1));
      const float cx = static_cast<float>(intrinsic.at<double>(0, 2));
      const float cy = static_cast<float>(intrinsic.at<double>(1, 2));
      cv::Mat mapX(faceSize, faceSize, CV_32F);
      cv::Mat mapY(faceSize, faceSize, CV_32F);
      cv::Mat valid(faceSize, faceSize, CV_8U, cv::Scalar(0));
      for (int row = 0; row < faceSize; ++row) {
        float *mapXRow = mapX.ptr<float>(row);
        float *mapYRow = mapY.ptr<float>(row);
        uchar *validRow = valid.ptr<uchar>(row);
        for (int column = 0; column < faceSize; ++column) {
          const cv::Vec3f local = rotationT *
              world[static_cast<std::size_t>(row * faceSize + column)];
          const float safeZ = std::abs(local[2]) > 1e-7f ? local[2] : 1.0f;
          const float sampleX = fx * local[0] / safeZ + cx;
          const float sampleY = fy * local[1] / safeZ + cy;
          mapXRow[column] = sampleX;
          mapYRow[column] = sampleY;
          if (local[2] > 0 && sampleX >= -0.5f &&
              sampleX <= source.cols - 0.5f && sampleY >= -0.5f &&
              sampleY <= source.rows - 0.5f) {
            validRow[column] = 255;
          }
        }
      }
      cv::Mat sampled;
      cv::remap(source, sampled, mapX, mapY, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
      sampled.setTo(cv::Scalar::all(0), valid == 0);
      if (normalizedSourceX != nullptr && normalizedSourceY != nullptr) {
        cv::Mat normalizedX =
            (mapX - cx) / std::max(source.cols * 0.5f, 1.0f);
        cv::Mat normalizedY =
            (mapY - cy) / std::max(source.rows * 0.5f, 1.0f);
        normalizedX.setTo(0.0f, valid == 0);
        normalizedY.setTo(0.0f, valid == 0);
        normalizedSourceX->push_back(std::move(normalizedX));
        normalizedSourceY->push_back(std::move(normalizedY));
      }
      images.push_back(std::move(sampled));
      masks.push_back(std::move(valid));
    }
  };

  std::vector<cv::Mat> seamImages;
  std::vector<cv::Mat> seamMasks;
  renderLayers(seamSize, seamImages, seamMasks);
  std::vector<cv::UMat> seamProxiesU(seamImages.size());
  std::vector<cv::UMat> seamMasksU(seamMasks.size());
  for (std::size_t index = 0; index < seamImages.size(); ++index) {
    cv::Mat proxy = structureSeamProxy(seamImages[index]);
    proxy.convertTo(seamProxiesU[index], CV_32F);
    seamMasks[index].copyTo(seamMasksU[index]);
  }
  const std::vector<cv::Point> faceCorners(seamImages.size(), cv::Point(0, 0));
  const int64 graphCutStarted = cv::getTickCount();
  try {
    cv::detail::GraphCutSeamFinder seamFinder(
        cv::detail::GraphCutSeamFinderBase::COST_COLOR);
    seamFinder.find(seamProxiesU, faceCorners, seamMasksU);
  } catch (const cv::Exception &) {
    for (std::size_t index = 0; index < seamMasks.size(); ++index) {
      seamMasks[index].copyTo(seamMasksU[index]);
    }
  }
  stats.graphCutSeconds =
      (cv::getTickCount() - graphCutStarted) / cv::getTickFrequency();
  const OwnerTopologyPruneStats topologyPrune =
      pruneLowTextureOwnerIslands(seamMasksU, seamMasks, seamImages);
  stats.topologyPrunedComponents = topologyPrune.removedComponents;
  stats.topologyReassignedPixels = topologyPrune.reassignedPixels;
  CentralPairSelection centralPair;
  if (selectCentralPair) {
    centralPair =
        selectConnectedCentralPair(seamImages, seamMasks, seamMasksU, 0.24);
    stats.centralPairSelected = centralPair.accepted;
    stats.centralPairCoverage = centralPair.coverage;
    stats.centralPairScore =
        std::isfinite(centralPair.score) ? centralPair.score : 0.0;
    stats.centralPairSeconds = centralPair.elapsedSeconds;
    if (centralPair.first >= 0 && centralPair.second >= 0) {
      stats.centralPairInputIndices = {
          sourceIndices[static_cast<std::size_t>(centralPair.first)],
          sourceIndices[static_cast<std::size_t>(centralPair.second)]};
    }
  }

  std::vector<cv::Mat> composeImages;
  std::vector<cv::Mat> composeMasks;
  std::vector<cv::Mat> composeNormalizedSourceX;
  std::vector<cv::Mat> composeNormalizedSourceY;
  renderLayers(composeSize, composeImages, composeMasks,
               &composeNormalizedSourceX, &composeNormalizedSourceY);
  const PairwiseResponseFieldEstimate responseField =
      equalizePairwiseResponseFields(composeImages, composeMasks,
                                     composeNormalizedSourceX,
                                     composeNormalizedSourceY);
  stats.responseFieldAccepted = responseField.accepted;
  stats.responseFieldEquationCount = responseField.equationCount;
  stats.responseFieldPairCount = responseField.pairCount;
  stats.responseFieldSeconds = responseField.elapsedSeconds;
  stats.responseFieldMedianBefore = responseField.medianBefore;
  stats.responseFieldMedianAfter = responseField.medianAfter;
  stats.responseFieldP90Before = responseField.p90Before;
  stats.responseFieldP90After = responseField.p90After;
  for (std::size_t localIndex = 0;
       localIndex < responseField.gainRanges.size(); ++localIndex) {
    stats.responseFieldGainRangesByInput[static_cast<std::size_t>(
        sourceIndices[localIndex])] = responseField.gainRanges[localIndex];
  }
  if ((selectCentralPair && !centralPair.accepted) ||
      (requireResponseField &&
       (!responseField.accepted || responseField.p90After >= 0.08))) {
    stats.centralPairGateRejected = selectCentralPair && !centralPair.accepted;
    stats.responseFieldGateRejected =
        requireResponseField &&
        (!responseField.accepted || responseField.p90After >= 0.08);
    stats.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return stats;
  }
  std::vector<cv::UMat> composeImagesU(composeImages.size());
  std::vector<cv::UMat> composeMasksU(composeMasks.size());
  for (std::size_t index = 0; index < composeImages.size(); ++index) {
    composeImages[index].copyTo(composeImagesU[index]);
    composeMasks[index].copyTo(composeMasksU[index]);
  }
  cv::Ptr<cv::detail::ExposureCompensator> compensator;
  if (responseField.accepted) {
    compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
  } else {
    compensator = cv::detail::ExposureCompensator::createDefault(
        cv::detail::ExposureCompensator::GAIN_BLOCKS);
    try {
      compensator->feed(
          std::vector<cv::Point>(composeImages.size(), cv::Point()),
          composeImagesU, composeMasksU);
    } catch (const cv::Exception &) {
      compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
    }
  }

  cv::detail::MultiBandBlender blender(false, blendBands);
  blender.setNumBands(blendBands);
  blender.prepare(cv::Rect(0, 0, composeSize, composeSize));
  cv::Mat ownerFace(composeSize, composeSize, CV_16S, cv::Scalar(-1));
  for (std::size_t localIndex = 0; localIndex < composeImages.size();
       ++localIndex) {
    cv::Mat corrected = composeImages[localIndex].clone();
    compensator->apply(static_cast<int>(localIndex), cv::Point(), corrected,
                       composeMasks[localIndex]);
    cv::Mat seamOwnership =
        seamMasksU[localIndex].getMat(cv::ACCESS_READ).clone();
    cv::resize(seamOwnership, seamOwnership,
               cv::Size(composeSize, composeSize), 0, 0, cv::INTER_NEAREST);
    cv::dilate(seamOwnership, seamOwnership, cv::Mat(), cv::Point(-1, -1), 1);
    cv::bitwise_and(seamOwnership, composeMasks[localIndex], seamOwnership);
    if (cv::countNonZero(seamOwnership) == 0) {
      continue;
    }
    ownerFace.setTo(sourceIndices[localIndex], seamOwnership);
    cv::Mat corrected16;
    corrected.convertTo(corrected16, CV_16S);
    blender.feed(corrected16, seamOwnership, cv::Point());
    ++stats.feedCount;
  }
  if (stats.feedCount == 0) {
    stats.elapsedSeconds =
        (cv::getTickCount() - started) / cv::getTickFrequency();
    return stats;
  }
  cv::Mat face16;
  cv::Mat faceMask;
  blender.blend(face16, faceMask);
  cv::Mat face;
  face16.convertTo(face, CV_8U);
  face.setTo(cv::Scalar::all(0), faceMask == 0);

  const int transitionRows = std::min(
      panorama.rows,
      static_cast<int>(std::ceil(
          (90.0 - minimumReplacementLatitudeDegrees) / 180.0 * panorama.rows)) +
          2);
  const int rowOffset = topPole ? 0 : panorama.rows - transitionRows;
  cv::Mat mapX(transitionRows, panorama.cols, CV_32F);
  cv::Mat mapY(transitionRows, panorama.cols, CV_32F);
  const double extent =
      std::tan(fieldOfViewDegrees * kPi / 180.0 * 0.5);
  std::vector<double> latitudes(static_cast<std::size_t>(transitionRows));
  for (int row = 0; row < transitionRows; ++row) {
    const int outputRow = rowOffset + row;
    const double latitude =
        (0.5 - (outputRow + 0.5) / panorama.rows) * kPi;
    latitudes[static_cast<std::size_t>(row)] = latitude * 180.0 / kPi;
    float *mapXRow = mapX.ptr<float>(row);
    float *mapYRow = mapY.ptr<float>(row);
    for (int column = 0; column < panorama.cols; ++column) {
      const double longitude =
          ((column + 0.5) / panorama.cols - 0.5) * (2.0 * kPi);
      const double cosLat = std::cos(latitude);
      const cv::Vec3d world(std::sin(longitude) * cosLat,
                            -std::sin(latitude),
                            std::cos(longitude) * cosLat);
      // world-to-top-face: world (x,y,z) -> local (x,z,-y).
      // world-to-bottom-face: world (x,y,z) -> local (x,-z,y).
      const cv::Vec3d local = topPole
                                  ? cv::Vec3d(world[0], world[2], -world[1])
                                  : cv::Vec3d(world[0], -world[2], world[1]);
      const double safeZ = std::max(local[2], 1e-9);
      mapXRow[column] = static_cast<float>(
          ((local[0] / safeZ / extent) + 1.0) * 0.5 * composeSize - 0.5);
      mapYRow[column] = static_cast<float>(
          ((local[1] / safeZ / extent) + 1.0) * 0.5 * composeSize - 0.5);
    }
  }
  cv::Mat sampled;
  cv::Mat sampledMask;
  cv::Mat sampledOwner;
  cv::remap(face, sampled, mapX, mapY, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  cv::remap(faceMask, sampledMask, mapX, mapY, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::remap(ownerFace, sampledOwner, mapX, mapY, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT, cv::Scalar(-1));

  cv::Vec3d gains(1, 1, 1);
  for (int channel = 0; channel < 3; ++channel) {
    std::vector<double> ratios;
    for (int row = 0; row < transitionRows; ++row) {
      const double latitude =
          std::abs(latitudes[static_cast<std::size_t>(row)]);
      if (latitude > minimumReplacementLatitudeDegrees + 2.5 ||
          latitude < minimumReplacementLatitudeDegrees + 0.4) {
        continue;
      }
      const cv::Vec3b *referenceRow =
          panorama.ptr<cv::Vec3b>(rowOffset + row);
      const cv::Vec3b *sampledRow = sampled.ptr<cv::Vec3b>(row);
      const uchar *validRow = sampledMask.ptr<uchar>(row);
      for (int column = 0; column < panorama.cols; ++column) {
        const double source = sampledRow[column][channel];
        const double target = referenceRow[column][channel];
        if (validRow[column] != 0 && source > 16.0 && target > 8.0) {
          ratios.push_back(target / source);
        }
      }
    }
    if (ratios.size() >= 64) {
      gains[channel] = std::clamp(medianOfDoubles(std::move(ratios)), 0.72, 1.38);
    }
  }
  stats.photometricGainsBGR = gains;

  cv::Mat globallyCorrected = sampled.clone();
  for (int row = 0; row < globallyCorrected.rows; ++row) {
    cv::Vec3b *correctedRow = globallyCorrected.ptr<cv::Vec3b>(row);
    const uchar *validRow = sampledMask.ptr<uchar>(row);
    for (int column = 0; column < globallyCorrected.cols; ++column) {
      if (validRow[column] == 0) {
        continue;
      }
      for (int channel = 0; channel < 3; ++channel) {
        correctedRow[column][channel] = static_cast<uchar>(std::clamp(
            std::lround(correctedRow[column][channel] * gains[channel]), 0L,
            255L));
      }
    }
  }
  const cv::Mat reference =
      panorama.rowRange(rowOffset, rowOffset + transitionRows);
  const LongitudeGainEstimate longitudeGain = estimateLongitudeBoundaryGain(
      reference, globallyCorrected, sampledMask, latitudes,
      minimumReplacementLatitudeDegrees);
  stats.longitudeGainAccepted = longitudeGain.accepted;
  stats.longitudeGainRejectedByCapPressure =
      longitudeGain.rejectedByCapPressure;
  stats.longitudeGainSupportedColumns = longitudeGain.supportedColumns;
  stats.longitudeGainMinimum = longitudeGain.minimum;
  stats.longitudeGainMaximum = longitudeGain.maximum;
  stats.longitudeGainP05 = longitudeGain.p05;
  stats.longitudeGainP95 = longitudeGain.p95;

  cv::Mat alpha(transitionRows, panorama.cols, CV_32F, cv::Scalar(0));
  for (int row = 0; row < transitionRows; ++row) {
    const double latitude =
        std::abs(latitudes[static_cast<std::size_t>(row)]);
    const double linear = std::clamp(
        (latitude - minimumReplacementLatitudeDegrees) /
            std::max(fullReplacementLatitudeDegrees -
                         minimumReplacementLatitudeDegrees,
                     1e-9),
        0.0, 1.0);
    const float smooth = static_cast<float>(linear * linear * (3.0 - 2.0 * linear));
    alpha.row(row).setTo(smooth, sampledMask.row(row));
  }
  for (int row = 0; row < transitionRows; ++row) {
    const int outputRow = rowOffset + row;
    cv::Vec3b *resultRow = panorama.ptr<cv::Vec3b>(outputRow);
    uchar *resultMaskRow = mask.ptr<uchar>(outputRow);
    short *labelRow =
        labels != nullptr ? labels->ptr<short>(outputRow) : nullptr;
    const cv::Vec3b *sampledRow = sampled.ptr<cv::Vec3b>(row);
    const uchar *sampledMaskRow = sampledMask.ptr<uchar>(row);
    const short *ownerRow = sampledOwner.ptr<short>(row);
    const float *alphaRow = alpha.ptr<float>(row);
    for (int column = 0; column < panorama.cols; ++column) {
      if (sampledMaskRow[column] == 0 || alphaRow[column] <= 1e-4f) {
        continue;
      }
      ++stats.replacedPixels;
      const bool wasMissing = resultMaskRow[column] == 0;
      const double a = alphaRow[column];
      const double longitudeDistance = std::clamp(
          (std::abs(latitudes[static_cast<std::size_t>(row)]) -
           minimumReplacementLatitudeDegrees) /
              std::max(90.0 - minimumReplacementLatitudeDegrees, 1e-9),
          0.0, 1.0);
      const double longitudeSmooth = longitudeDistance * longitudeDistance *
                                     (3.0 - 2.0 * longitudeDistance);
      const double spatialGain = std::exp(
          (1.0 - longitudeSmooth) *
          longitudeGain.logGain[static_cast<std::size_t>(column)]);
      for (int channel = 0; channel < 3; ++channel) {
        const double corrected = sampledRow[column][channel] * gains[channel] *
                                 spatialGain;
        const double blended = corrected * a + resultRow[column][channel] * (1.0 - a);
        resultRow[column][channel] = static_cast<uchar>(
            std::clamp(std::lround(blended), 0L, 255L));
      }
      if (wasMissing && a >= 0.999) {
        resultMaskRow[column] = 255;
        ++stats.newlyCoveredPixels;
        const int owner = ownerRow[column];
        if (owner >= 0 && owner < static_cast<int>(stats.selectedPixelsByInput.size())) {
          ++stats.selectedPixelsByInput[static_cast<std::size_t>(owner)];
          if (labelRow != nullptr) {
            labelRow[column] = static_cast<short>(owner);
          }
        }
      }
    }
  }
  stats.enabled = stats.replacedPixels > 0;
  stats.elapsedSeconds =
      (cv::getTickCount() - started) / cv::getTickFrequency();
  return stats;
}

PolarCubeFaceStats composeTopCubeFace(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<PoseFrameLayout> &layouts, double workScale,
    double composeScale, int seamSize, int composeSize,
    double fieldOfViewDegrees, int blendBands,
    double fullReplacementLatitudeDegrees,
    double minimumReplacementLatitudeDegrees) {
  return composePolarCubeFace(
      panorama, mask, labels, composeScaleImages, cameras, layouts, workScale,
      composeScale, seamSize, composeSize, fieldOfViewDegrees, blendBands,
      fullReplacementLatitudeDegrees, minimumReplacementLatitudeDegrees, "top",
      false, false);
}

PolarCubeFaceStats composeBottomCubeFace(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras,
    const std::vector<PoseFrameLayout> &layouts, double workScale,
    double composeScale, int seamSize, int composeSize,
    double fieldOfViewDegrees, int blendBands,
    double fullReplacementLatitudeDegrees,
    double minimumReplacementLatitudeDegrees) {
  return composePolarCubeFace(
      panorama, mask, labels, composeScaleImages, cameras, layouts, workScale,
      composeScale, seamSize, composeSize, fieldOfViewDegrees, blendBands,
      fullReplacementLatitudeDegrees, minimumReplacementLatitudeDegrees,
      "bottom", true, true);
}

DirectSphereFillStats fillEquirectangularHoles(
    cv::Mat &panorama, cv::Mat &mask, cv::Mat *labels,
    const std::vector<cv::Mat> &composeScaleImages,
    const std::vector<cv::detail::CameraParams> &cameras, double workScale,
    double composeScale, int rowChunk) {
  DirectSphereFillStats stats;
  stats.enabled = true;
  if (panorama.empty() || mask.empty() || panorama.type() != CV_8UC3 ||
      mask.type() != CV_8U || panorama.size() != mask.size()) {
    throw std::runtime_error(
        "Direct sphere fill requires matching CV_8UC3 panorama and CV_8U mask");
  }
  if (composeScaleImages.size() != cameras.size()) {
    throw std::runtime_error(
        "Direct sphere fill image/camera counts do not match");
  }
  const int height = mask.rows;
  const int width = mask.cols;
  const int cameraCount = static_cast<int>(cameras.size());
  stats.filledPixelsByInput.assign(static_cast<std::size_t>(cameraCount), 0);

  cv::Mat missing = mask == 0;
  cv::Mat labelMap;
  if (labels != nullptr && !labels->empty() && labels->type() == CV_16S &&
      labels->size() == panorama.size()) {
    labelMap = labels->clone();
  } else {
    labelMap = cv::Mat(height, width, CV_16S, cv::Scalar(-1));
  }
  if (cv::countNonZero(missing) == 0) {
    stats.filledPixels = 0;
    stats.remainingPixels = 0;
    if (labels != nullptr) {
      *labels = labelMap;
    }
    return stats;
  }

  const double scaleRatio = composeScale / std::max(workScale, 1e-12);
  std::vector<cv::Mat> scaledIntrinsics;
  std::vector<cv::Mat> rotationsT;
  scaledIntrinsics.reserve(static_cast<std::size_t>(cameraCount));
  rotationsT.reserve(static_cast<std::size_t>(cameraCount));
  for (int index = 0; index < cameraCount; ++index) {
    if (composeScaleImages[static_cast<std::size_t>(index)].empty() ||
        composeScaleImages[static_cast<std::size_t>(index)].type() != CV_8UC3) {
      throw std::runtime_error(
          "Direct sphere fill requires non-empty CV_8UC3 compose-scale images");
    }
    cv::Mat intrinsic = cameras[static_cast<std::size_t>(index)].K();
    intrinsic.convertTo(intrinsic, CV_64F);
    intrinsic.at<double>(0, 0) *= scaleRatio;
    intrinsic.at<double>(0, 2) *= scaleRatio;
    intrinsic.at<double>(1, 1) *= scaleRatio;
    intrinsic.at<double>(1, 2) *= scaleRatio;
    scaledIntrinsics.push_back(intrinsic);

    cv::Mat rotation;
    cameras[static_cast<std::size_t>(index)].R.convertTo(rotation, CV_64F);
    // camera.R is camera-to-world; world→camera uses R.t().
    rotationsT.push_back(rotation.t());
  }

  // Narrow valid-side band + missing region, then feather into graph-cut result.
  cv::Mat validMask;
  cv::bitwise_not(missing, validMask);
  cv::Mat validDistance;
  cv::distanceTransform(validMask, validDistance, cv::DIST_L2, 5);
  const int transitionPixels =
      std::max(12, static_cast<int>(std::lround(height * 0.03)));
  cv::Mat renderRegion = missing.clone();
  for (int row = 0; row < height; ++row) {
    const uchar *missingRow = missing.ptr<uchar>(row);
    const float *distanceRow = validDistance.ptr<float>(row);
    uchar *renderRow = renderRegion.ptr<uchar>(row);
    for (int column = 0; column < width; ++column) {
      if (missingRow[column] == 0 &&
          distanceRow[column] < static_cast<float>(transitionPixels)) {
        renderRow[column] = 255;
      }
    }
  }

  int firstRow = height;
  int lastRow = -1;
  for (int row = 0; row < height; ++row) {
    if (cv::countNonZero(renderRegion.row(row)) > 0) {
      firstRow = std::min(firstRow, row);
      lastRow = std::max(lastRow, row);
    }
  }
  if (lastRow < firstRow) {
    stats.filledPixels = 0;
    stats.remainingPixels = cv::countNonZero(mask == 0);
    if (labels != nullptr) {
      *labels = labelMap;
    }
    return stats;
  }
  ++lastRow; // exclusive

  std::vector<double> longitude(static_cast<std::size_t>(width));
  for (int column = 0; column < width; ++column) {
    longitude[static_cast<std::size_t>(column)] =
        ((column + 0.5) / width - 0.5) * (2.0 * kPi);
  }

  const int chunkSize = std::max(1, rowChunk);
  for (int top = firstRow; top < lastRow; top += chunkSize) {
    const int bottom = std::min(lastRow, top + chunkSize);
    const int chunkHeight = bottom - top;
    const cv::Mat chunkRegion = renderRegion.rowRange(top, bottom);
    const cv::Mat chunkMissing = missing.rowRange(top, bottom);
    if (cv::countNonZero(chunkRegion) == 0) {
      continue;
    }

    cv::Mat world(chunkHeight, width, CV_64FC3);
    for (int localRow = 0; localRow < chunkHeight; ++localRow) {
      const int globalRow = top + localRow;
      const double latitude = (0.5 - (globalRow + 0.5) / height) * kPi;
      const double cosLat = std::cos(latitude);
      const double sinLat = std::sin(latitude);
      cv::Vec3d *worldRow = world.ptr<cv::Vec3d>(localRow);
      for (int column = 0; column < width; ++column) {
        const double lon = longitude[static_cast<std::size_t>(column)];
        worldRow[column] = cv::Vec3d(std::sin(lon) * cosLat, -sinLat,
                                     std::cos(lon) * cosLat);
      }
    }

    cv::Mat bestZ(chunkHeight, width, CV_64F,
                  cv::Scalar(-std::numeric_limits<double>::infinity()));
    cv::Mat bestIndex(chunkHeight, width, CV_16S, cv::Scalar(-1));
    cv::Mat accumulated(chunkHeight, width, CV_64FC3, cv::Scalar(0, 0, 0));
    cv::Mat accumulatedWeight(chunkHeight, width, CV_64F, cv::Scalar(0));

    cv::Mat flatWorld = world.reshape(1, chunkHeight * width); // N x 3
    for (int index = 0; index < cameraCount; ++index) {
      const cv::Mat &image = composeScaleImages[static_cast<std::size_t>(index)];
      const cv::Mat &intrinsic = scaledIntrinsics[static_cast<std::size_t>(index)];
      const cv::Mat &rotationT = rotationsT[static_cast<std::size_t>(index)];

      cv::Mat localFlat = (rotationT * flatWorld.t()).t(); // N x 3
      cv::Mat local = localFlat.reshape(3, chunkHeight);   // HxW CV_64FC3

      cv::Mat mapX(chunkHeight, width, CV_32F);
      cv::Mat mapY(chunkHeight, width, CV_32F);
      cv::Mat valid(chunkHeight, width, CV_8U, cv::Scalar(0));
      const double fx = intrinsic.at<double>(0, 0);
      const double fy = intrinsic.at<double>(1, 1);
      const double cx = intrinsic.at<double>(0, 2);
      const double cy = intrinsic.at<double>(1, 2);
      const double maxX = image.cols - 0.5;
      const double maxY = image.rows - 0.5;

      for (int localRow = 0; localRow < chunkHeight; ++localRow) {
        const cv::Vec3d *localRowPtr = local.ptr<cv::Vec3d>(localRow);
        const uchar *regionRow = chunkRegion.ptr<uchar>(localRow);
        float *mapXRow = mapX.ptr<float>(localRow);
        float *mapYRow = mapY.ptr<float>(localRow);
        uchar *validRow = valid.ptr<uchar>(localRow);
        double *bestZRow = bestZ.ptr<double>(localRow);
        short *bestIndexRow = bestIndex.ptr<short>(localRow);

        for (int column = 0; column < width; ++column) {
          const double z = localRowPtr[column][2];
          const double safeZ = std::abs(z) > 1e-12 ? z : 1.0;
          const double sampleX = fx * localRowPtr[column][0] / safeZ + cx;
          const double sampleY = fy * localRowPtr[column][1] / safeZ + cy;
          mapXRow[column] = static_cast<float>(sampleX);
          mapYRow[column] = static_cast<float>(sampleY);
          const bool inFrustum =
              regionRow[column] != 0 && z > 0.0 && sampleX >= -0.5 &&
              sampleX <= maxX && sampleY >= -0.5 && sampleY <= maxY;
          if (!inFrustum) {
            continue;
          }
          validRow[column] = 255;
          if (z > bestZRow[column]) {
            bestZRow[column] = z;
            bestIndexRow[column] = static_cast<short>(index);
          }
        }
      }

      cv::Mat sampled;
      cv::remap(image, sampled, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                cv::Scalar(0, 0, 0));

      for (int localRow = 0; localRow < chunkHeight; ++localRow) {
        const cv::Vec3d *localRowPtr = local.ptr<cv::Vec3d>(localRow);
        const uchar *validRow = valid.ptr<uchar>(localRow);
        const cv::Vec3b *sampledRow = sampled.ptr<cv::Vec3b>(localRow);
        cv::Vec3d *accumRow = accumulated.ptr<cv::Vec3d>(localRow);
        double *accumWeightRow = accumulatedWeight.ptr<double>(localRow);

        for (int column = 0; column < width; ++column) {
          if (validRow[column] == 0) {
            continue;
          }
          const double blendWeight =
              std::pow(std::clamp(localRowPtr[column][2], 0.0, 1.0), 8.0);
          accumRow[column][0] += sampledRow[column][0] * blendWeight;
          accumRow[column][1] += sampledRow[column][1] * blendWeight;
          accumRow[column][2] += sampledRow[column][2] * blendWeight;
          accumWeightRow[column] += blendWeight;
        }
      }
    }

    cv::Mat direct(chunkHeight, width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat directValid(chunkHeight, width, CV_8U, cv::Scalar(0));
    for (int localRow = 0; localRow < chunkHeight; ++localRow) {
      const cv::Vec3d *accumRow = accumulated.ptr<cv::Vec3d>(localRow);
      const double *accumWeightRow = accumulatedWeight.ptr<double>(localRow);
      cv::Vec3b *directRow = direct.ptr<cv::Vec3b>(localRow);
      uchar *directValidRow = directValid.ptr<uchar>(localRow);
      for (int column = 0; column < width; ++column) {
        if (accumWeightRow[column] <= 1e-12) {
          continue;
        }
        directValidRow[column] = 255;
        for (int channel = 0; channel < 3; ++channel) {
          const double value =
              accumRow[column][channel] / accumWeightRow[column];
          directRow[column][channel] =
              static_cast<uchar>(std::clamp(std::lround(value), 0L, 255L));
        }
      }
    }

    // Exposure gain calibration on the valid-side overlap band.
    std::vector<cv::Vec3d> referencePixels;
    std::vector<cv::Vec3d> directPixels;
    referencePixels.reserve(static_cast<std::size_t>(chunkHeight * width / 8));
    directPixels.reserve(referencePixels.capacity());
    for (int localRow = 0; localRow < chunkHeight; ++localRow) {
      const uchar *directValidRow = directValid.ptr<uchar>(localRow);
      const uchar *missingRow = chunkMissing.ptr<uchar>(localRow);
      const uchar *regionRow = chunkRegion.ptr<uchar>(localRow);
      const cv::Vec3b *resultRow = panorama.ptr<cv::Vec3b>(top + localRow);
      const cv::Vec3b *directRow = direct.ptr<cv::Vec3b>(localRow);
      for (int column = 0; column < width; ++column) {
        if (directValidRow[column] != 0 && missingRow[column] == 0 &&
            regionRow[column] != 0) {
          referencePixels.emplace_back(resultRow[column][0], resultRow[column][1],
                                       resultRow[column][2]);
          directPixels.emplace_back(directRow[column][0], directRow[column][1],
                                    directRow[column][2]);
        }
      }
    }
    if (static_cast<int>(referencePixels.size()) >= 32) {
      cv::Vec3d gains(1.0, 1.0, 1.0);
      for (int channel = 0; channel < 3; ++channel) {
        std::vector<double> ratios;
        ratios.reserve(referencePixels.size());
        for (std::size_t sample = 0; sample < referencePixels.size(); ++sample) {
          if (directPixels[sample][channel] > 12.0) {
            ratios.push_back(referencePixels[sample][channel] /
                             directPixels[sample][channel]);
          }
        }
        if (static_cast<int>(ratios.size()) >= 16) {
          gains[channel] = std::clamp(medianOfDoubles(ratios), 0.72, 1.38);
        }
      }
      for (int localRow = 0; localRow < chunkHeight; ++localRow) {
        cv::Vec3b *directRow = direct.ptr<cv::Vec3b>(localRow);
        for (int column = 0; column < width; ++column) {
          for (int channel = 0; channel < 3; ++channel) {
            const double value = directRow[column][channel] * gains[channel];
            directRow[column][channel] =
                static_cast<uchar>(std::clamp(std::lround(value), 0L, 255L));
          }
        }
      }
    }

    cv::Mat alpha(chunkHeight, width, CV_64F, cv::Scalar(1.0));
    for (int localRow = 0; localRow < chunkHeight; ++localRow) {
      const uchar *missingRow = chunkMissing.ptr<uchar>(localRow);
      const float *distanceRow = validDistance.ptr<float>(top + localRow);
      double *alphaRow = alpha.ptr<double>(localRow);
      for (int column = 0; column < width; ++column) {
        if (missingRow[column] == 0) {
          alphaRow[column] = std::clamp(
              1.0 - distanceRow[column] / transitionPixels, 0.0, 1.0);
        }
      }
    }

    for (int localRow = 0; localRow < chunkHeight; ++localRow) {
      const uchar *directValidRow = directValid.ptr<uchar>(localRow);
      const uchar *regionRow = chunkRegion.ptr<uchar>(localRow);
      const uchar *missingRow = chunkMissing.ptr<uchar>(localRow);
      const short *bestIndexRow = bestIndex.ptr<short>(localRow);
      const double *alphaRow = alpha.ptr<double>(localRow);
      const cv::Vec3b *directRow = direct.ptr<cv::Vec3b>(localRow);
      cv::Vec3b *resultRow = panorama.ptr<cv::Vec3b>(top + localRow);
      uchar *maskRow = mask.ptr<uchar>(top + localRow);
      short *labelRow = labelMap.ptr<short>(top + localRow);

      for (int column = 0; column < width; ++column) {
        if (directValidRow[column] == 0 || regionRow[column] == 0) {
          continue;
        }
        const double a = alphaRow[column];
        for (int channel = 0; channel < 3; ++channel) {
          const double blended = directRow[column][channel] * a +
                                 resultRow[column][channel] * (1.0 - a);
          resultRow[column][channel] =
              static_cast<uchar>(std::clamp(std::lround(blended), 0L, 255L));
        }
        if (missingRow[column] != 0) {
          maskRow[column] = 255;
          labelRow[column] = bestIndexRow[column];
          if (bestIndexRow[column] >= 0 &&
              bestIndexRow[column] < cameraCount) {
            ++stats.filledPixelsByInput[static_cast<std::size_t>(
                bestIndexRow[column])];
          }
        }
      }
    }
  }

  stats.filledPixels = 0;
  for (int count : stats.filledPixelsByInput) {
    stats.filledPixels += count;
  }
  stats.remainingPixels = cv::countNonZero(mask == 0);
  if (labels != nullptr) {
    *labels = labelMap;
  }
  return stats;
}

} // namespace sphera
