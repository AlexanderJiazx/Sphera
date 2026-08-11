#if TARGET_OS_SIMULATOR

#import "SpheraNativeEngine.h"

@implementation SpheraNativeStitchArtifacts
@end

@implementation SpheraNativeEngineBridge

+ (void)stitchWithManifestURL:(NSURL *)manifestURL
           outputDirectoryURL:(NSURL *)outputDirectoryURL
      matchCacheDirectoryURL:(NSURL *)matchCacheDirectoryURL
    enableLegacyLearnedMatches:(BOOL)enableLegacyLearnedMatches
                   completion:(SpheraNativeStitchCompletion)completion {
  (void)manifestURL;
  (void)outputDirectoryURL;
  (void)matchCacheDirectoryURL;
  (void)enableLegacyLearnedMatches;
  NSError *error = [NSError
      errorWithDomain:@"com.sphera.capture.native-engine"
                 code:100
             userInfo:@{
               NSLocalizedDescriptionKey :
                   @"Native OpenCV stitch runs on device only; the simulator "
                   @"build excludes the iPhone OpenCV framework."
             }];
  dispatch_async(dispatch_get_main_queue(), ^{
    if (completion) {
      completion(nil, error);
    }
  });
}

@end

#endif // TARGET_OS_SIMULATOR
