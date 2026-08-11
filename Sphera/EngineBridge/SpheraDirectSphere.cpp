#include "SpheraDirectSphere.hpp"

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

} // namespace

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
  cv::Mat labelMap(height, width, CV_16S, cv::Scalar(-1));
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
