/**
 * @file frame_utils_opencv.h
 *
 * Functions for frame enhancement and demosaicing, using
 * the popular OpenCV image processing library.
 */

#ifndef LIBCAER_FRAME_UTILS_OPENCV_H_
#define LIBCAER_FRAME_UTILS_OPENCV_H_

#include "events/frame.h"

#ifdef __cplusplus
extern "C" {
#endif

// DEMOSAIC_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
enum caer_frame_utils_opencv_demosaic {
	DEMOSAIC_NORMAL = 0,
	DEMOSAIC_EDGE_AWARE = 1,
};

enum caer_frame_utils_opencv_contrast {
	CONTRAST_NORMALIZATION = 0,
	CONTRAST_HISTOGRAM_EQUALIZATION = 1,
	CONTRAST_CLAHE = 2,
};

caerFrameEventPacket caerFrameUtilsOpenCVDemosaic(caerFrameEventPacketConst framePacket,
	enum caer_frame_utils_opencv_demosaic demosaicType);
void caerFrameUtilsOpenCVContrast(caerFrameEventPacket framePacket, enum caer_frame_utils_opencv_contrast contrastType);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FRAME_UTILS_OPENCV_H_ */
