#import "SpheraNativeEngine.h"

#if !TARGET_OS_SIMULATOR

#include "SpheraPanoramaEngine.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr NSString *kErrorDomain = @"com.sphera.capture.native-engine";

std::runtime_error invalidManifest(NSString *field) {
  return std::runtime_error([[NSString
      stringWithFormat:@"Invalid or missing manifest field: %@", field]
      UTF8String]);
}

NSDictionary *requiredDictionary(id value, NSString *field) {
  if (![value isKindOfClass:NSDictionary.class]) {
    throw invalidManifest(field);
  }
  return static_cast<NSDictionary *>(value);
}

NSArray *requiredArray(id value, NSString *field) {
  if (![value isKindOfClass:NSArray.class]) {
    throw invalidManifest(field);
  }
  return static_cast<NSArray *>(value);
}

NSString *requiredString(id value, NSString *field) {
  if (![value isKindOfClass:NSString.class] ||
      [static_cast<NSString *>(value) length] == 0) {
    throw invalidManifest(field);
  }
  return static_cast<NSString *>(value);
}

NSNumber *requiredNumber(id value, NSString *field) {
  if (![value isKindOfClass:NSNumber.class]) {
    throw invalidManifest(field);
  }
  return static_cast<NSNumber *>(value);
}

std::filesystem::path fileSystemPath(NSURL *url) {
  if (!url.isFileURL) {
    throw std::runtime_error(
        "The native Sphera engine only accepts local file URLs");
  }
  return std::filesystem::path(url.fileSystemRepresentation);
}

sphera::CaptureRing captureRing(NSString *value) {
  if ([value isEqualToString:@"horizontal"]) {
    return sphera::CaptureRing::horizontal;
  }
  if ([value isEqualToString:@"downward"]) {
    return sphera::CaptureRing::downward;
  }
  if ([value isEqualToString:@"upward"]) {
    return sphera::CaptureRing::upward;
  }
  throw invalidManifest(@"frames[].target.ring");
}

sphera::StitchRequest parseRequest(NSURL *manifestURL,
                                   NSURL *outputDirectoryURL) {
  NSError *readError = nil;
  NSData *data = [NSData dataWithContentsOfURL:manifestURL
                                       options:0
                                         error:&readError];
  if (data == nil) {
    throw std::runtime_error(readError.localizedDescription.UTF8String
                                 ?: "Could not read manifest");
  }

  NSError *jsonError = nil;
  id object = [NSJSONSerialization JSONObjectWithData:data
                                              options:0
                                                error:&jsonError];
  if (object == nil) {
    throw std::runtime_error(jsonError.localizedDescription.UTF8String
                                 ?: "Could not decode manifest");
  }

  NSDictionary *manifest = requiredDictionary(object, @"root");
  const NSInteger schemaVersion =
      [requiredNumber(manifest[@"schemaVersion"], @"schemaVersion")
          integerValue];
  if (schemaVersion < 5) {
    throw std::runtime_error(
        "The capture manifest predates the pose-initialized engine contract");
  }

  NSString *imageDirectoryName =
      requiredString(manifest[@"imageDirectory"], @"imageDirectory");
  NSArray *frames = requiredArray(manifest[@"frames"], @"frames");
  if (frames.count < 2) {
    throw std::runtime_error(
        "At least two captured frames are required for stitching");
  }

  const std::filesystem::path packageDirectory =
      fileSystemPath(manifestURL).parent_path();
  const std::filesystem::path imageDirectory =
      packageDirectory / std::string(imageDirectoryName.UTF8String);

  sphera::StitchRequest request;
  request.outputDirectory = fileSystemPath(outputDirectoryURL);
  request.frames.reserve(frames.count);

  for (NSUInteger index = 0; index < frames.count; ++index) {
    NSString *prefix =
        [NSString stringWithFormat:@"frames[%lu]", (unsigned long)index];
    NSDictionary *frame = requiredDictionary(frames[index], prefix);
    NSDictionary *target = requiredDictionary(
        frame[@"target"], [prefix stringByAppendingString:@".target"]);
    NSDictionary *pose = requiredDictionary(
        frame[@"pose"], [prefix stringByAppendingString:@".pose"]);
    NSDictionary *intrinsics = requiredDictionary(
        frame[@"intrinsics"], [prefix stringByAppendingString:@".intrinsics"]);
    NSDictionary *photo = requiredDictionary(
        frame[@"photo"], [prefix stringByAppendingString:@".photo"]);
    NSDictionary *rotation = requiredDictionary(
        pose[@"cameraToCaptureReferenceRotationMatrix"],
        [prefix stringByAppendingString:
                    @".pose.cameraToCaptureReferenceRotationMatrix"]);
    NSArray *rotationValues = requiredArray(
        rotation[@"values"],
        [prefix stringByAppendingString:
                    @".pose.cameraToCaptureReferenceRotationMatrix.values"]);
    if (rotationValues.count != 9) {
      throw invalidManifest(
          [prefix stringByAppendingString:
                      @".pose.cameraToCaptureReferenceRotationMatrix.values"]);
    }

    NSString *filename =
        requiredString(frame[@"imageFilename"],
                       [prefix stringByAppendingString:@".imageFilename"]);
    sphera::FrameInput input;
    input.imagePath = imageDirectory / std::string(filename.UTF8String);
    input.imageFilename = filename.UTF8String;
    input.sequenceIndex =
        [requiredNumber(frame[@"sequenceIndex"],
                        [prefix stringByAppendingString:@".sequenceIndex"])
            intValue];
    input.ring = captureRing(requiredString(
        target[@"ring"], [prefix stringByAppendingString:@".target.ring"]));
    input.ringIndex =
        [requiredNumber(target[@"ringIndex"],
                        [prefix stringByAppendingString:@".target.ringIndex"])
            intValue];
    input.ringCount =
        [requiredNumber(target[@"ringCount"],
                        [prefix stringByAppendingString:@".target.ringCount"])
            intValue];
    input.yawDegrees =
        [requiredNumber(target[@"yawDegrees"],
                        [prefix stringByAppendingString:@".target.yawDegrees"])
            doubleValue];
    input.pitchDegrees = [requiredNumber(
        target[@"pitchDegrees"],
        [prefix stringByAppendingString:@".target.pitchDegrees"]) doubleValue];
    input.exifOrientation =
        [photo[@"exifOrientation"] isKindOfClass:NSNumber.class]
            ? [photo[@"exifOrientation"] intValue]
            : 1;
    input.intrinsics.fx =
        [requiredNumber(intrinsics[@"photoFx"],
                        [prefix stringByAppendingString:@".intrinsics.photoFx"])
            doubleValue];
    input.intrinsics.fy =
        [requiredNumber(intrinsics[@"photoFy"],
                        [prefix stringByAppendingString:@".intrinsics.photoFy"])
            doubleValue];
    input.intrinsics.cx =
        [requiredNumber(intrinsics[@"photoCx"],
                        [prefix stringByAppendingString:@".intrinsics.photoCx"])
            doubleValue];
    input.intrinsics.cy =
        [requiredNumber(intrinsics[@"photoCy"],
                        [prefix stringByAppendingString:@".intrinsics.photoCy"])
            doubleValue];
    input.intrinsics.referenceWidth = [requiredNumber(
        intrinsics[@"orientedPhotoWidth"],
        [prefix stringByAppendingString:@".intrinsics.orientedPhotoWidth"])
        intValue];
    input.intrinsics.referenceHeight = [requiredNumber(
        intrinsics[@"orientedPhotoHeight"],
        [prefix stringByAppendingString:@".intrinsics.orientedPhotoHeight"])
        intValue];

    for (NSUInteger valueIndex = 0; valueIndex < 9; ++valueIndex) {
      input.cameraToCaptureReferenceRotation[valueIndex] = [requiredNumber(
          rotationValues[valueIndex],
          [prefix
              stringByAppendingFormat:
                  @".pose.cameraToCaptureReferenceRotationMatrix.values[%lu]",
                  (unsigned long)valueIndex]) doubleValue];
    }
    request.frames.push_back(std::move(input));
  }

  return request;
}

NSError *nativeError(NSString *description) {
  return [NSError errorWithDomain:kErrorDomain
                             code:1
                         userInfo:@{NSLocalizedDescriptionKey : description}];
}

} // namespace

@implementation SpheraNativeStitchArtifacts

- (instancetype)initWithPanoramaURL:(NSURL *)panoramaURL
                          reportURL:(NSURL *_Nullable)reportURL
                 contributionMapURL:(NSURL *_Nullable)contributionMapURL {
  self = [super init];
  if (self) {
    _panoramaURL = panoramaURL;
    _reportURL = reportURL;
    _contributionMapURL = contributionMapURL;
  }
  return self;
}

@end

@implementation SpheraNativeEngineBridge

+ (void)stitchManifestAtURL:(NSURL *)manifestURL
        outputDirectoryURL:(NSURL *)outputDirectoryURL
         matchCacheDirectoryURL:(NSURL *)matchCacheDirectoryURL
         enableLegacyLearnedMatches:(BOOL)enableLegacyLearnedMatches
            progressHandler:(SpheraNativeStitchProgressHandler)progressHandler
                completion:(SpheraNativeStitchCompletion)completion {
  NSURL *manifestCopy = [manifestURL copy];
  NSURL *outputCopy = [outputDirectoryURL copy];
  NSURL *matchCacheCopy = [matchCacheDirectoryURL copy];
  const BOOL enableLegacy = enableLegacyLearnedMatches;
  SpheraNativeStitchProgressHandler progressCopy = [progressHandler copy];
  SpheraNativeStitchCompletion completionCopy = [completion copy];

  dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
    @autoreleasepool {
      SpheraNativeStitchArtifacts *result = nil;
      NSError *error = nil;
      try {
        sphera::StitchRequest request =
            parseRequest(manifestCopy, outputCopy);
        request.enableLegacyLearnedMatches = enableLegacy;
        if (enableLegacy && matchCacheCopy != nil) {
          request.learnedMatchCacheDirectory = fileSystemPath(matchCacheCopy);
        }
        if (progressCopy != nil) {
          request.progress = [progressCopy](double fraction,
                                            const std::string &message) {
            NSString *text =
                [NSString stringWithUTF8String:message.c_str()] ?: @"";
            progressCopy(fraction, text);
          };
        }
        sphera::StitchArtifacts artifacts =
            sphera::PanoramaEngine::stitch(request);
        NSURL *panoramaURL = [NSURL
            fileURLWithFileSystemRepresentation:artifacts.panoramaPath.c_str()
                                    isDirectory:NO
                                  relativeToURL:nil];
        NSURL *reportURL = [NSURL
            fileURLWithFileSystemRepresentation:artifacts.reportPath.c_str()
                                    isDirectory:NO
                                  relativeToURL:nil];
        NSURL *contributionURL = nil;
        if (!artifacts.contributionMapPath.empty()) {
          contributionURL = [NSURL
              fileURLWithFileSystemRepresentation:artifacts.contributionMapPath
                                                      .c_str()
                                      isDirectory:NO
                                    relativeToURL:nil];
        }
        result = [[SpheraNativeStitchArtifacts alloc]
            initWithPanoramaURL:panoramaURL
                      reportURL:reportURL
             contributionMapURL:contributionURL];
      } catch (const std::exception &exception) {
        NSString *description =
            [NSString stringWithUTF8String:exception.what()];
        if (description.length == 0) {
          description = @"The native panorama engine failed.";
        }
        @try {
          NSDictionary *failureReport = @{
            @"engine" : @"sphera-ios-native",
            @"pipeline_version" : @"sensor_first_s1_adaptive_ring_seam_v1",
            @"recipe" : @"sensor_first_s1_adaptive_ring_seam",
            @"ml_model_usage" : @"none",
            @"status" : @"error",
            @"error" : description,
          };
          NSData *json = [NSJSONSerialization dataWithJSONObject:failureReport
                                                         options:NSJSONWritingPrettyPrinted
                                                           error:nil];
          if (json != nil) {
            NSURL *reportURL =
                [outputCopy URLByAppendingPathComponent:@"report.json"];
            [json writeToURL:reportURL atomically:YES];
          }
        } @catch (__unused NSException *reportException) {
        }
        error = nativeError(description);
      } catch (...) {
        error = nativeError(
            @"The native panorama engine failed with an unknown error.");
      }

      dispatch_async(dispatch_get_main_queue(), ^{
        completionCopy(result, error);
      });
    }
  });
}

@end

#endif // !TARGET_OS_SIMULATOR
