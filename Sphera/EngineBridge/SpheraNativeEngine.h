#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface SpheraNativeStitchArtifacts : NSObject

@property(nonatomic, readonly) NSURL *panoramaURL;
@property(nonatomic, readonly, nullable) NSURL *reportURL;

- (instancetype)initWithPanoramaURL:(NSURL *)panoramaURL
                          reportURL:(nullable NSURL *)reportURL
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;

@end

typedef void (^SpheraNativeStitchCompletion)(
    SpheraNativeStitchArtifacts *_Nullable artifacts, NSError *_Nullable error);

/// Objective-C-only surface for the Swift/C++ boundary. No OpenCV or C++ type
/// crosses this header.
@interface SpheraNativeEngineBridge : NSObject

+ (void)stitchManifestAtURL:(NSURL *)manifestURL
              outputDirectoryURL:(NSURL *)outputDirectoryURL
    maximumPoseRefinementDegrees:(double)maximumPoseRefinementDegrees
                      completion:(SpheraNativeStitchCompletion)completion
    NS_SWIFT_NAME(stitch(manifestURL:outputDirectoryURL:maximumPoseRefinementDegrees:completion:));

@end

NS_ASSUME_NONNULL_END
