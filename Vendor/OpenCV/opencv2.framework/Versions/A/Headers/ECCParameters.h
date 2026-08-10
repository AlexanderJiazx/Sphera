//
// This file is auto-generated. Please don't modify it!
//
#pragma once

#ifdef __cplusplus
//#import "opencv.hpp"
#import "opencv2/video.hpp"
#import "opencv2/video/tracking.hpp"
#else
#define CV_EXPORTS
#endif

#import <Foundation/Foundation.h>


@class IntVector;
@class TermCriteria;



NS_ASSUME_NONNULL_BEGIN

// C++: class ECCParameters
/**
 * struct ECCParameters is used by findTransformECCMultiScale
 *
 * motionType parameter, specifying the type of motion:
 *  -   **MOTION_TRANSLATION** sets a translational motion model; warpMatrix is `$$2\times 3$$` with
 *      the first `$$2\times 2$$` part being the unity matrix and the rest two parameters being
 *      estimated.
 *  -   **MOTION_EUCLIDEAN** sets a Euclidean (rigid) transformation as motion model; three
 *      parameters are estimated; warpMatrix is `$$2\times 3$$`.
 *  -   **MOTION_AFFINE** sets an affine motion model (DEFAULT); six parameters are estimated;
 *      warpMatrix is `$$2\times 3$$`.
 *  -   **MOTION_HOMOGRAPHY** sets a homography as a motion model; eight parameters are
 *      estimated;\`warpMatrix\` is `$$3\times 3$$`.
 * criteria parameter, specifying the termination criteria of the ECC algorithm;
 * criteria.epsilon defines the threshold of the increment in the correlation coefficient between two
 * iterations (a negative criteria.epsilon makes criteria.maxcount the only termination criterion).
 * Default values are shown in the declaration above.
 * itersPerLevel Criterion extension: distribution of iterations limit over pyramid levels.
 * Can be empty, in this case, this algorithm will use criteria.maxCount on each level.
 * gaussFiltSize An optional value indicating size of gaussian blur filter; (DEFAULT: 5)
 * nlevels An optional value indicating amount of levels in the pyramid; (DEFAULT: 4)
 * interpolation Type of warp interpolation. Possible values are INTER_NEAREST and INTER_LINEAR.
 * Affects accuracy, especially when motionType == MOTION_TRANSLATION. (DEFAULT: INTER_LINEAR)
 *
 * Member of `Video`
 */
CV_EXPORTS @interface ECCParameters : NSObject


#ifdef __cplusplus
@property(readonly)cv::Ptr<cv::ECCParameters> nativePtr;
#endif

#ifdef __cplusplus
- (instancetype)initWithNativePtr:(cv::Ptr<cv::ECCParameters>)nativePtr;
+ (instancetype)fromNative:(cv::Ptr<cv::ECCParameters>)nativePtr;
#endif


#pragma mark - Methods


//
//   cv::ECCParameters::ECCParameters()
//
- (instancetype)init;


    //
    // C++: int cv::ECCParameters::motionType
    //

@property int motionType;

    //
    // C++: TermCriteria cv::ECCParameters::criteria
    //

@property TermCriteria* criteria;

    //
    // C++: vector_int cv::ECCParameters::itersPerLevel
    //

@property IntVector* itersPerLevel;

    //
    // C++: int cv::ECCParameters::gaussFiltSize
    //

@property int gaussFiltSize;

    //
    // C++: int cv::ECCParameters::nlevels
    //

@property int nlevels;

    //
    // C++: int cv::ECCParameters::interpolation
    //

@property int interpolation;


@end

NS_ASSUME_NONNULL_END


