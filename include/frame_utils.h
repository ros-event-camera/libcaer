/**
 * @file frame_utils.h
 *
 * Basic functions for frame enhancement and demosaicing, that don't
 * require any external dependencies, such as OpenCV.
 * Use of the OpenCV variants is recommended for quality and performance.
 */

#ifndef LIBCAER_FRAME_UTILS_H_
#define LIBCAER_FRAME_UTILS_H_

#include "events/frame.h"

#ifdef __cplusplus
extern "C" {
#endif

caerFrameEventPacket caerFrameUtilsDemosaic(caerFrameEventPacketConst framePacket);
void caerFrameUtilsContrast(caerFrameEventPacket framePacket);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_FRAME_UTILS_H_ */
