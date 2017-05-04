#ifndef LIBCAER_SRC_AUTOEXPOSURE_H_
#define LIBCAER_SRC_AUTOEXPOSURE_H_

#include "libcaer.h"
#include "events/frame.h"

#define AUTOEXPOSURE_MIDDLEGRAY_MSV 5

struct auto_exposure_state {
	size_t pixelHistogram[AUTOEXPOSURE_MIDDLEGRAY_MSV];
};

typedef struct auto_exposure_state *autoExposureState;

// Returns next exposure value in µs, or -1 if currently set is optimal.
int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureLastSetValue);

#endif /* LIBCAER_SRC_AUTOEXPOSURE_H_ */
