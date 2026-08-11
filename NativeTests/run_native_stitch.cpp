#include "SpheraPanoramaEngine.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#include <opencv2/core.hpp>
#pragma clang diagnostic pop

namespace {

std::string readFile(const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Could not read " + path.string());
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

sphera::CaptureRing parseRing(const std::string &value) {
  if (value == "horizontal") {
    return sphera::CaptureRing::horizontal;
  }
  if (value == "downward") {
    return sphera::CaptureRing::downward;
  }
  if (value == "upward") {
    return sphera::CaptureRing::upward;
  }
  throw std::runtime_error("Unknown ring: " + value);
}

// Minimal JSON extraction helpers for sphera-engine-request.json style files.
std::string extractString(const std::string &json, const std::string &key,
                          std::size_t from) {
  const std::string pattern = "\"" + key + "\"";
  const std::size_t keyPos = json.find(pattern, from);
  if (keyPos == std::string::npos) {
    throw std::runtime_error("Missing key " + key);
  }
  const std::size_t colon = json.find(':', keyPos);
  const std::size_t firstQuote = json.find('"', colon + 1);
  const std::size_t secondQuote = json.find('"', firstQuote + 1);
  return json.substr(firstQuote + 1, secondQuote - firstQuote - 1);
}

double extractNumber(const std::string &json, const std::string &key,
                     std::size_t from) {
  const std::string pattern = "\"" + key + "\"";
  const std::size_t keyPos = json.find(pattern, from);
  if (keyPos == std::string::npos) {
    throw std::runtime_error("Missing key " + key);
  }
  const std::size_t colon = json.find(':', keyPos);
  std::size_t cursor = colon + 1;
  while (cursor < json.size() &&
         (json[cursor] == ' ' || json[cursor] == '\n' || json[cursor] == '\t')) {
    ++cursor;
  }
  std::size_t end = cursor;
  while (end < json.size() &&
         (std::isdigit(static_cast<unsigned char>(json[end])) ||
          json[end] == '-' || json[end] == '+' || json[end] == '.' ||
          json[end] == 'e' || json[end] == 'E')) {
    ++end;
  }
  return std::stod(json.substr(cursor, end - cursor));
}

std::array<double, 9> extractMatrix9(const std::string &json, std::size_t from) {
  const std::size_t valuesPos = json.find("\"values\"", from);
  const std::size_t open = json.find('[', valuesPos);
  const std::size_t close = json.find(']', open);
  std::array<double, 9> values{};
  std::size_t cursor = open + 1;
  for (int index = 0; index < 9; ++index) {
    while (cursor < close &&
           !(std::isdigit(static_cast<unsigned char>(json[cursor])) ||
             json[cursor] == '-' || json[cursor] == '.')) {
      ++cursor;
    }
    std::size_t end = cursor;
    while (end < close &&
           (std::isdigit(static_cast<unsigned char>(json[end])) ||
            json[end] == '-' || json[end] == '+' || json[end] == '.' ||
            json[end] == 'e' || json[end] == 'E')) {
      ++end;
    }
    values[static_cast<std::size_t>(index)] =
        std::stod(json.substr(cursor, end - cursor));
    cursor = end;
  }
  return values;
}

sphera::StitchRequest loadRequest(const std::filesystem::path &captureDir,
                                  const std::filesystem::path &outputDir) {
  const std::filesystem::path metadata =
      captureDir / "sphera-engine-request.json";
  const std::filesystem::path images =
      std::filesystem::exists(captureDir / "images_oriented")
          ? captureDir / "images_oriented"
          : captureDir / "images";
  const std::string json = readFile(metadata);

  sphera::StitchRequest request;
  request.outputDirectory = outputDir;
  request.enableLegacyLearnedMatches = false;

  std::size_t cursor = 0;
  while (true) {
    const std::size_t framePos = json.find("\"imageFilename\"", cursor);
    if (framePos == std::string::npos) {
      break;
    }
    sphera::FrameInput frame;
    frame.imageFilename = extractString(json, "imageFilename", framePos);
    frame.imagePath = images / frame.imageFilename;
    frame.sequenceIndex =
        static_cast<int>(extractNumber(json, "sequenceIndex", framePos));
    const std::string ring = extractString(json, "ring", framePos);
    frame.ring = parseRing(ring);
    frame.ringIndex =
        static_cast<int>(extractNumber(json, "ringIndex", framePos));
    frame.ringCount =
        static_cast<int>(extractNumber(json, "ringCount", framePos));
    frame.yawDegrees = extractNumber(json, "yawDegrees", framePos);
    frame.pitchDegrees = extractNumber(json, "pitchDegrees", framePos);
    frame.intrinsics.fx = extractNumber(json, "photoFx", framePos);
    frame.intrinsics.fy = extractNumber(json, "photoFy", framePos);
    frame.intrinsics.cx = extractNumber(json, "photoCx", framePos);
    frame.intrinsics.cy = extractNumber(json, "photoCy", framePos);
    frame.intrinsics.referenceWidth =
        static_cast<int>(extractNumber(json, "orientedPhotoWidth", framePos));
    frame.intrinsics.referenceHeight =
        static_cast<int>(extractNumber(json, "orientedPhotoHeight", framePos));
    try {
      frame.exifOrientation =
          static_cast<int>(extractNumber(json, "exifOrientation", framePos));
    } catch (...) {
      frame.exifOrientation = 1;
    }
    frame.cameraToCaptureReferenceRotation = extractMatrix9(json, framePos);
    request.frames.push_back(std::move(frame));
    cursor = framePos + 1;
  }
  if (request.frames.size() < 2) {
    throw std::runtime_error("No frames parsed from " + metadata.string());
  }
  return request;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <capture-directory> <output-directory>\n";
    return 2;
  }
  try {
    const std::filesystem::path captureDir = argv[1];
    const std::filesystem::path outputDir = argv[2];
    std::filesystem::create_directories(outputDir);
    sphera::StitchRequest request = loadRequest(captureDir, outputDir);
    request.progress = [](double fraction, const std::string &message) {
      std::cout << "[" << static_cast<int>(fraction * 100) << "%] " << message
                << "\n";
    };
    const sphera::StitchArtifacts artifacts =
        sphera::PanoramaEngine::stitch(request);
    std::cout << "Wrote " << artifacts.panoramaPath << "\n";
    std::cout << "Wrote " << artifacts.reportPath << "\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "error: " << exception.what() << "\n";
    return 1;
  }
}
