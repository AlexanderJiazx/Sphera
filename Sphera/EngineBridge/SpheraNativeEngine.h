#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface SpheraNativeStitchArtifacts : NSObject

@property(nonatomic, readonly) NSURL *panoramaURL;
@property(nonatomic, readonly, nullable) NSURL *reportURL;
@property(nonatomic, readonly, nullable) NSURL *contributionMapURL;

- (instancetype)initWithPanoramaURL:(NSURL *)panoramaURL
                          reportURL:(nullable NSURL *)reportURL
                 contributionMapURL:(nullable NSURL *)contributionMapURL
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;

@end

typedef void (^SpheraNativeStitchCompletion)(
    SpheraNativeStitchArtifacts *_Nullable artifacts, NSError *_Nullable error);

/// Called from the stitch worker thread with the current pipeline stage.
typedef void (^SpheraNativeStitchProgressHandler)(double fraction,
                                                  NSString *message);

/// Objective-C-only surface for the Swift/C++ boundary. No OpenCV or C++ type
/// crosses this header.
@interface SpheraNativeEngineBridge : NSObject

+ (void)stitchManifestAtURL:(NSURL *)manifestURL
        outputDirectoryURL:(NSURL *)outputDirectoryURL
         matchCacheDirectoryURL:(NSURL *_Nullable)matchCacheDirectoryURL
         enableLegacyLearnedMatches:(BOOL)enableLegacyLearnedMatches
            progressHandler:(SpheraNativeStitchProgressHandler _Nullable)progressHandler
                completion:(SpheraNativeStitchCompletion)completion
    NS_SWIFT_NAME(stitch(manifestURL:outputDirectoryURL:matchCacheDirectoryURL:enableLegacyLearnedMatches:progressHandler:completion:));

@end

NS_ASSUME_NONNULL_END
