#include "SpheraAdaptiveRingSeam.hpp"

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
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/video/tracking.hpp>
#pragma clang diagnostic pop

namespace sphera {

cv::Mat structureSeamProxy(const cv::Mat &bgrImage) {
  CV_Assert(bgrImage.type() == CV_8UC3);
  cv::Mat lab;
  cv::cvtColor(bgrImage, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> labChannels;
  cv::split(lab, labChannels);
  cv::Mat luminance;
  labChannels[0].convertTo(luminance, CV_32F);
  cv::Mat gx;
  cv::Mat gy;
  cv::Sobel(luminance, gx, CV_32F, 1, 0, 3);
  cv::Sobel(luminance, gy, CV_32F, 0, 1, 3);
  cv::Mat gradient;
  cv::magnitude(gx, gy, gradient);
  cv::Mat edges;
  cv::Canny(bgrImage, edges, 70, 150);
  cv::Mat edgeFloat;
  edges.convertTo(edgeFloat, CV_32F);
  edgeFloat *= 2.0f;

  auto robustChannel = [](const cv::Mat &values) {
    std::vector<float> flat;
    flat.reserve(static_cast<std::size_t>(values.total()));
    for (int row = 0; row < values.rows; ++row) {
      for (int column = 0; column < values.cols; ++column) {
        flat.push_back(values.at<float>(row, column));
      }
    }
    std::vector<double> doubles(flat.begin(), flat.end());
    const double median = medianOf(doubles);
    std::vector<double> deviations;
    deviations.reserve(doubles.size());
    for (double value : doubles) {
      deviations.push_back(std::abs(value - median));
    }
    const double deviation = medianOf(deviations);
    const double scale = std::max(1.4826 * deviation, 1.0);
    cv::Mat output(values.size(), CV_8U);
    for (int row = 0; row < values.rows; ++row) {
      for (int column = 0; column < values.cols; ++column) {
        const double normalized =
            (values.at<float>(row, column) - median) / scale;
        output.at<uchar>(row, column) = static_cast<uchar>(
            std::clamp(127.5 + 30.0 * normalized, 0.0, 255.0));
      }
    }
    return output;
  };

  std::vector<cv::Mat> channels = {robustChannel(luminance),
                                   robustChannel(gradient),
                                   robustChannel(edgeFloat)};
  cv::Mat proxy;
  cv::merge(channels, proxy);
  return proxy;
}

cv::Mat periodicMinimumCostPath(const cv::Mat &cost, int maximumStep,
                                double smoothness) {
  cv::Mat values;
  cost.convertTo(values, CV_64F);
  if (values.empty() || values.dims != 2) {
    throw std::runtime_error("periodic seam cost must be a non-empty 2D array");
  }
  if (maximumStep < 0) {
    throw std::runtime_error("maximum seam step must be non-negative");
  }
  if (smoothness < 0) {
    throw std::runtime_error("seam smoothness must be non-negative");
  }
  const int height = values.rows;
  const int width = values.cols;
  double bestTotal = std::numeric_limits<double>::infinity();
  int bestEnd = 0;
  cv::Mat bestBack;
  for (int start = 0; start < height; ++start) {
    cv::Mat previous(height, 1, CV_64F, cv::Scalar(std::numeric_limits<double>::infinity()));
    previous.at<double>(start) = values.at<double>(start, 0);
    cv::Mat back(width, height, CV_16S, cv::Scalar(-1));
    for (int column = 1; column < width; ++column) {
      cv::Mat current(height, 1, CV_64F,
                      cv::Scalar(std::numeric_limits<double>::infinity()));
      for (int row = 0; row < height; ++row) {
        const int low = std::max(0, row - maximumStep);
        const int high = std::min(height, row + maximumStep + 1);
        double best = std::numeric_limits<double>::infinity();
        int predecessor = low;
        for (int candidate = low; candidate < high; ++candidate) {
          const double score =
              previous.at<double>(candidate) +
              smoothness * std::pow(static_cast<double>(candidate - row), 2.0);
          if (score < best) {
            best = score;
            predecessor = candidate;
          }
        }
        current.at<double>(row) = values.at<double>(row, column) + best;
        back.at<short>(column, row) = static_cast<short>(predecessor);
      }
      previous = current;
    }
    double localBest = std::numeric_limits<double>::infinity();
    int end = 0;
    for (int row = 0; row < height; ++row) {
      if (std::abs(row - start) > maximumStep) {
        continue;
      }
      const double total =
          previous.at<double>(row) +
          smoothness * std::pow(static_cast<double>(row - start), 2.0);
      if (total < localBest) {
        localBest = total;
        end = row;
      }
    }
    if (localBest < bestTotal) {
      bestTotal = localBest;
      bestEnd = end;
      bestBack = back;
    }
  }
  if (bestBack.empty() || !std::isfinite(bestTotal)) {
    throw std::runtime_error("Could not solve a periodic seam path");
  }
  cv::Mat path(width, 1, CV_32S);
  path.at<int>(width - 1) = bestEnd;
  for (int column = width - 1; column > 0; --column) {
    path.at<int>(column - 1) =
        bestBack.at<short>(column, path.at<int>(column));
  }
  return path;
}

std::pair<std::vector<cv::Mat>, AdaptiveRingSeamReport>
applyAdaptiveRingSeamPriors(const std::vector<cv::Mat> &masks,
                            const std::vector<cv::Point> &corners,
                            const std::vector<cv::Mat> &images,
                            const std::vector<PoseFrameLayout> &layout,
                            double overlapFraction, double edgeWeight,
                            double geometryWeight, int maximumStep,
                            double smoothness) {
  if (!(overlapFraction > 0.0 && overlapFraction <= 1.0)) {
    throw std::runtime_error("adaptive ring seam overlap must be in (0, 1]");
  }
  if (edgeWeight < 0 || geometryWeight < 0) {
    throw std::runtime_error("adaptive ring seam weights must be non-negative");
  }
  if (masks.size() != corners.size() || masks.size() != images.size() ||
      masks.size() != layout.size()) {
    throw std::runtime_error("adaptive ring seam inputs must have equal length");
  }

  AdaptiveRingSeamReport report;
  report.overlapFraction = overlapFraction;
  report.edgeWeight = edgeWeight;
  report.geometryWeight = geometryWeight;
  report.maximumStepPixels = maximumStep;
  report.smoothness = smoothness;

  std::vector<cv::Size> sizes;
  sizes.reserve(masks.size());
  for (const cv::Mat &mask : masks) {
    sizes.push_back(mask.size());
  }
  const cv::Rect roi = cv::detail::resultRoi(corners, sizes);
  report.roi = roi;
  const int roiLeft = roi.x;
  const int roiTop = roi.y;
  const int roiWidth = roi.width;
  const int roiHeight = roi.height;

  std::set<int> ringSet;
  for (const PoseFrameLayout &item : layout) {
    ringSet.insert(item.ring);
  }
  std::unordered_map<int, cv::Mat> ringLayers;
  std::unordered_map<int, cv::Mat> ringValid;
  std::unordered_map<int, cv::Mat> ringScore;
  for (int ring : ringSet) {
    ringLayers[ring] = cv::Mat::zeros(roiHeight, roiWidth, CV_8UC3);
    ringValid[ring] = cv::Mat::zeros(roiHeight, roiWidth, CV_8U);
    ringScore[ring] =
        cv::Mat(roiHeight, roiWidth, CV_32F, cv::Scalar(-1.0f));
  }

  std::unordered_map<int, std::vector<double>> centersByRing;
  for (std::size_t index = 0; index < masks.size(); ++index) {
    const cv::Mat &mask = masks[index];
    const cv::Mat &image = images[index];
    const PoseFrameLayout &item = layout[index];
    cv::Mat valid = mask > 0;
    std::vector<cv::Point> nonzero;
    cv::findNonZero(valid, nonzero);
    if (!nonzero.empty()) {
      std::vector<int> rows;
      rows.reserve(nonzero.size());
      for (const cv::Point &point : nonzero) {
        rows.push_back(point.y);
      }
      std::nth_element(rows.begin(), rows.begin() + rows.size() / 2, rows.end());
      centersByRing[item.ring].push_back(corners[index].y +
                                         rows[rows.size() / 2]);
    }
    cv::Mat distance;
    cv::distanceTransform(valid, distance, cv::DIST_L2, 5);
    const int left = corners[index].x - roiLeft;
    const int top = corners[index].y - roiTop;
    cv::Mat targetScore =
        ringScore[item.ring](cv::Rect(left, top, mask.cols, mask.rows));
    cv::Mat targetLayer =
        ringLayers[item.ring](cv::Rect(left, top, mask.cols, mask.rows));
    cv::Mat targetValid =
        ringValid[item.ring](cv::Rect(left, top, mask.cols, mask.rows));
    for (int row = 0; row < mask.rows; ++row) {
      for (int column = 0; column < mask.cols; ++column) {
        if (mask.at<uchar>(row, column) == 0) {
          continue;
        }
        const float score = distance.at<float>(row, column);
        if (score > targetScore.at<float>(row, column)) {
          targetLayer.at<cv::Vec3b>(row, column) =
              image.at<cv::Vec3b>(row, column);
          targetScore.at<float>(row, column) = score;
          targetValid.at<uchar>(row, column) = 255;
        }
      }
    }
  }

  if (centersByRing.size() < 2) {
    std::vector<cv::Mat> copies;
    copies.reserve(masks.size());
    for (const cv::Mat &mask : masks) {
      copies.push_back(mask.clone());
    }
    return {copies, report};
  }

  std::unordered_map<int, double> ringCenters;
  for (auto &entry : centersByRing) {
    ringCenters[entry.first] = medianOf(entry.second);
    report.ringCentersGlobalY[entry.first] = ringCenters[entry.first];
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
  report.ringOrderTopToBottom = ordered;

  std::unordered_map<int, cv::Mat> lowerPaths;
  std::unordered_map<int, cv::Mat> upperPaths;

  for (std::size_t index = 0; index + 1 < ordered.size(); ++index) {
    const int upperRing = ordered[index];
    const int lowerRing = ordered[index + 1];
    const double upperCenter = ringCenters[upperRing];
    const double lowerCenter = ringCenters[lowerRing];
    const double gap = lowerCenter - upperCenter;
    const double midpoint = 0.5 * (upperCenter + lowerCenter);
    const double halfOverlap = gap * overlapFraction / 2.0;
    const int bandTopGlobal =
        std::max(roiTop, static_cast<int>(std::floor(midpoint - halfOverlap)));
    const int bandBottomGlobal = std::min(
        roiTop + roiHeight - 1,
        static_cast<int>(std::ceil(midpoint + halfOverlap)));
    const int bandTop = bandTopGlobal - roiTop;
    const int bandBottom = bandBottomGlobal - roiTop + 1;
    const cv::Mat upperImage =
        ringLayers[upperRing].rowRange(bandTop, bandBottom);
    const cv::Mat lowerImage =
        ringLayers[lowerRing].rowRange(bandTop, bandBottom);
    const cv::Mat upperMask =
        ringValid[upperRing].rowRange(bandTop, bandBottom) > 0;
    const cv::Mat lowerMask =
        ringValid[lowerRing].rowRange(bandTop, bandBottom) > 0;
    cv::Mat jointlyValid;
    cv::bitwise_and(upperMask, lowerMask, jointlyValid);

    cv::Mat upperLab;
    cv::Mat lowerLab;
    cv::cvtColor(upperImage, upperLab, cv::COLOR_BGR2Lab);
    cv::cvtColor(lowerImage, lowerLab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> upperChannels;
    std::vector<cv::Mat> lowerChannels;
    cv::split(upperLab, upperChannels);
    cv::split(lowerLab, lowerChannels);
    cv::Mat upperLuma;
    cv::Mat lowerLuma;
    upperChannels[0].convertTo(upperLuma, CV_32F);
    lowerChannels[0].convertTo(lowerLuma, CV_32F);
    cv::Mat lumaDelta = upperLuma - lowerLuma;
    std::vector<double> jointDeltas;
    for (int row = 0; row < jointlyValid.rows; ++row) {
      for (int column = 0; column < jointlyValid.cols; ++column) {
        if (jointlyValid.at<uchar>(row, column) != 0) {
          jointDeltas.push_back(lumaDelta.at<float>(row, column));
        }
      }
    }
    const double offset = jointDeltas.empty() ? 0.0 : medianOf(jointDeltas);
    cv::Mat colorDifference;
    cv::absdiff(lumaDelta, cv::Scalar(static_cast<float>(offset)),
                colorDifference);
    std::vector<double> jointColor;
    for (int row = 0; row < jointlyValid.rows; ++row) {
      for (int column = 0; column < jointlyValid.cols; ++column) {
        if (jointlyValid.at<uchar>(row, column) != 0) {
          jointColor.push_back(colorDifference.at<float>(row, column));
        }
      }
    }
    const double colorScale =
        jointColor.empty() ? 1.0 : percentileOf(jointColor, 90.0);
    cv::Mat colorCost;
    colorDifference.convertTo(colorCost, CV_32F);
    colorCost /= static_cast<float>(std::max(colorScale, 8.0));
    cv::threshold(colorCost, colorCost, 3.0, 3.0, cv::THRESH_TRUNC);

    auto gradientAndEdges = [](const cv::Mat &luma, const cv::Mat &valid) {
      cv::Mat gx;
      cv::Mat gy;
      cv::Sobel(luma, gx, CV_32F, 1, 0, 3);
      cv::Sobel(luma, gy, CV_32F, 0, 1, 3);
      cv::Mat gradient;
      cv::magnitude(gx, gy, gradient);
      std::vector<double> values;
      for (int row = 0; row < valid.rows; ++row) {
        for (int column = 0; column < valid.cols; ++column) {
          if (valid.at<uchar>(row, column) != 0) {
            values.push_back(gradient.at<float>(row, column));
          }
        }
      }
      const double scale = values.empty() ? 1.0 : percentileOf(values, 90.0);
      gradient /= static_cast<float>(std::max(scale, 1.0));
      cv::threshold(gradient, gradient, 3.0, 3.0, cv::THRESH_TRUNC);
      cv::Mat luma8;
      luma.convertTo(luma8, CV_8U);
      cv::Mat edges;
      cv::Canny(luma8, edges, 60, 140);
      cv::dilate(edges, edges, cv::Mat::ones(3, 3, CV_8U));
      return std::make_pair(gradient, edges > 0);
    };

    auto upperGeometry = gradientAndEdges(upperLuma, upperMask);
    auto lowerGeometry = gradientAndEdges(lowerLuma, lowerMask);
    cv::Mat structuralCost;
    cv::max(upperGeometry.first, lowerGeometry.first, structuralCost);
    cv::Mat edgeCost;
    cv::bitwise_or(upperGeometry.second, lowerGeometry.second, edgeCost);
    edgeCost.convertTo(edgeCost, CV_32F);

    cv::Mat upperFlowInput;
    cv::Mat lowerFlowInput;
    cv::Mat upperLuma8;
    cv::Mat lowerLuma8;
    upperLuma.convertTo(upperLuma8, CV_8U);
    lowerLuma.convertTo(lowerLuma8, CV_8U);
    cv::equalizeHist(upperLuma8, upperFlowInput);
    cv::equalizeHist(lowerLuma8, lowerFlowInput);
    cv::Mat forwardFlow;
    cv::calcOpticalFlowFarneback(upperFlowInput, lowerFlowInput, forwardFlow,
                                 0.5, 2, 15, 3, 5, 1.1,
                                 cv::OPTFLOW_FARNEBACK_GAUSSIAN);
    cv::Mat textured = jointlyValid & (structuralCost > 0.12f);
    cv::Mat geometryCost = cv::Mat::zeros(structuralCost.size(), CV_32F);
    cv::Vec2d medianFlow(0, 0);
    double flowScale = 0;
    if (cv::countNonZero(textured) > 0) {
      std::vector<cv::Vec2f> flowSamples;
      for (int row = 0; row < textured.rows; ++row) {
        for (int column = 0; column < textured.cols; ++column) {
          if (textured.at<uchar>(row, column) != 0) {
            flowSamples.push_back(forwardFlow.at<cv::Vec2f>(row, column));
          }
        }
      }
      std::vector<double> flowX;
      std::vector<double> flowY;
      for (const cv::Vec2f &sample : flowSamples) {
        flowX.push_back(sample[0]);
        flowY.push_back(sample[1]);
      }
      medianFlow = cv::Vec2d(medianOf(flowX), medianOf(flowY));
      std::vector<double> residuals;
      for (int row = 0; row < forwardFlow.rows; ++row) {
        for (int column = 0; column < forwardFlow.cols; ++column) {
          const cv::Vec2f flow = forwardFlow.at<cv::Vec2f>(row, column);
          const double residual = std::hypot(flow[0] - medianFlow[0],
                                             flow[1] - medianFlow[1]);
          if (textured.at<uchar>(row, column) != 0) {
            residuals.push_back(residual);
          }
          geometryCost.at<float>(row, column) = static_cast<float>(residual);
        }
      }
      flowScale = residuals.empty() ? 0.0 : percentileOf(residuals, 90.0);
      geometryCost /= static_cast<float>(std::max(flowScale, 0.35));
      cv::threshold(geometryCost, geometryCost, 3.0, 3.0, cv::THRESH_TRUNC);
      cv::Mat structuralClip;
      cv::min(structuralCost, 1.0, structuralClip);
      geometryCost = geometryCost.mul(structuralClip);
    }

    cv::Mat midpointCost(bandBottom - bandTop, roiWidth, CV_32F);
    const double midpointLocal = midpoint - bandTopGlobal;
    for (int row = 0; row < midpointCost.rows; ++row) {
      const float costValue = static_cast<float>(
          0.08 * std::pow((row - midpointLocal) / std::max(halfOverlap, 1.0),
                          2.0));
      midpointCost.row(row).setTo(costValue);
    }

    cv::Mat cost = colorCost + structuralCost + edgeWeight * edgeCost +
                   geometryWeight * geometryCost + midpointCost;
    int unsupportedColumns = 0;
    for (int column = 0; column < jointlyValid.cols; ++column) {
      bool any = false;
      for (int row = 0; row < jointlyValid.rows; ++row) {
        if (jointlyValid.at<uchar>(row, column) != 0) {
          any = true;
          break;
        }
      }
      if (!any) {
        ++unsupportedColumns;
      }
    }
    for (int row = 0; row < cost.rows; ++row) {
      for (int column = 0; column < cost.cols; ++column) {
        if (jointlyValid.at<uchar>(row, column) == 0) {
          cost.at<float>(row, column) = 1'000'000.0f;
        }
      }
    }
    cv::GaussianBlur(cost, cost, cv::Size(3, 3), 0.65);
    for (int row = 0; row < cost.rows; ++row) {
      for (int column = 0; column < cost.cols; ++column) {
        if (jointlyValid.at<uchar>(row, column) == 0) {
          cost.at<float>(row, column) = 1'000'000.0f;
        }
      }
    }

    const cv::Mat localPath =
        periodicMinimumCostPath(cost, maximumStep, smoothness);
    cv::Mat globalPath(localPath.rows, 1, CV_32S);
    for (int column = 0; column < localPath.rows; ++column) {
      globalPath.at<int>(column) = localPath.at<int>(column) + bandTopGlobal;
    }
    upperPaths[upperRing] = globalPath;
    cv::Mat lowerPath = globalPath.clone();
    lowerPath += 1;
    lowerPaths[lowerRing] = lowerPath;

    AdaptiveRingSeamBoundary boundary;
    boundary.upperRing = upperRing;
    boundary.lowerRing = lowerRing;
    boundary.upperCenterGlobalY = upperCenter;
    boundary.lowerCenterGlobalY = lowerCenter;
    boundary.midpointGlobalY = midpoint;
    boundary.searchBandTopGlobalY = bandTopGlobal;
    boundary.searchBandBottomGlobalY = bandBottomGlobal;
    int pathMin = globalPath.at<int>(0);
    int pathMax = globalPath.at<int>(0);
    double pathSum = 0;
    double pathCostSum = 0;
    for (int column = 0; column < globalPath.rows; ++column) {
      const int y = globalPath.at<int>(column);
      pathMin = std::min(pathMin, y);
      pathMax = std::max(pathMax, y);
      pathSum += y;
      pathCostSum += cost.at<float>(localPath.at<int>(column), column);
    }
    boundary.pathMinimumGlobalY = pathMin;
    boundary.pathMaximumGlobalY = pathMax;
    boundary.pathMeanGlobalY = pathSum / globalPath.rows;
    boundary.pathCostMean = pathCostSum / globalPath.rows;
    boundary.robustGlobalFlowPixels = medianFlow;
    boundary.flowResidualP90Pixels = flowScale;
    boundary.unsupportedLongitudeColumns = unsupportedColumns;
    boundary.pathSampleStep = std::max(1, roiWidth / 64);
    for (int column = 0; column < globalPath.rows;
         column += boundary.pathSampleStep) {
      boundary.pathSamplesGlobalY.push_back(globalPath.at<int>(column));
    }
    report.boundaries.push_back(std::move(boundary));
  }

  cv::Mat originalCoverage = cv::Mat::zeros(roiHeight, roiWidth, CV_16U);
  std::vector<cv::Mat> restricted;
  restricted.reserve(masks.size());
  for (std::size_t index = 0; index < masks.size(); ++index) {
    const cv::Mat &source = masks[index];
    const int left = corners[index].x - roiLeft;
    const int top = corners[index].y - roiTop;
    for (int row = 0; row < source.rows; ++row) {
      for (int column = 0; column < source.cols; ++column) {
        if (source.at<uchar>(row, column) != 0) {
          originalCoverage.at<ushort>(top + row, left + column) += 1;
        }
      }
    }
    cv::Mat result = source.clone();
    const PoseFrameLayout &item = layout[index];
    for (int row = 0; row < result.rows; ++row) {
      const int globalRow = corners[index].y + row;
      for (int column = 0; column < result.cols; ++column) {
        const int roiColumn = corners[index].x + column - roiLeft;
        bool allowed = true;
        if (lowerPaths.count(item.ring) != 0) {
          allowed = allowed &&
                    globalRow >= lowerPaths[item.ring].at<int>(roiColumn);
        }
        if (upperPaths.count(item.ring) != 0) {
          allowed = allowed &&
                    globalRow <= upperPaths[item.ring].at<int>(roiColumn);
        }
        if (!allowed) {
          result.at<uchar>(row, column) = 0;
        }
      }
    }
    const int originalPixels = cv::countNonZero(source);
    report.retainedMaskFractionByInput.push_back(
        static_cast<double>(cv::countNonZero(result)) /
        std::max(1, originalPixels));
    restricted.push_back(result);
  }

  cv::Mat restrictedCoverage = cv::Mat::zeros(roiHeight, roiWidth, CV_16U);
  for (std::size_t index = 0; index < restricted.size(); ++index) {
    const int left = corners[index].x - roiLeft;
    const int top = corners[index].y - roiTop;
    for (int row = 0; row < restricted[index].rows; ++row) {
      for (int column = 0; column < restricted[index].cols; ++column) {
        if (restricted[index].at<uchar>(row, column) != 0) {
          restrictedCoverage.at<ushort>(top + row, left + column) += 1;
        }
      }
    }
  }

  int lostBeforeRestore = 0;
  cv::Mat lost = (originalCoverage > 0) & (restrictedCoverage == 0);
  lostBeforeRestore = cv::countNonZero(lost);
  if (lostBeforeRestore > 0) {
    for (std::size_t index = 0; index < masks.size(); ++index) {
      const int left = corners[index].x - roiLeft;
      const int top = corners[index].y - roiTop;
      for (int row = 0; row < masks[index].rows; ++row) {
        for (int column = 0; column < masks[index].cols; ++column) {
          if (lost.at<uchar>(top + row, left + column) != 0 &&
              masks[index].at<uchar>(row, column) != 0) {
            restricted[index].at<uchar>(row, column) = 255;
          }
        }
      }
    }
  }
  report.coveragePixelsRestored = lostBeforeRestore;
  return {restricted, report};
}

} // namespace sphera
