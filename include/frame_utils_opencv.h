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

enum caer_frame_utils_opencv_demosaic {
	DEMOSAIC_NORMAL, DEMOSAIC_EDGE_AWARE, // DEMOSAIC_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
};

enum caer_frame_utils_opencv_contrast {
	CONTRAST_NORMALIZATION, CONTRAST_HISTOGRAM_EQUALIZATION, CONTRAST_CLAHE,
};

caerFrameEventPacket caerFrameUtilsOpenCVDemosaic(caerFrameEventPacketConst framePacket,
	enum caer_frame_utils_opencv_demosaic demosaicType);
void caerFrameUtilsOpenCVContrast(caerFrameEventPacket framePacket, enum caer_frame_utils_opencv_contrast contrastType);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FRAME_UTILS_OPENCV_H_ */
