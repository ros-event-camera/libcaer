#ifndef LIBCAER_SRC_AUTOEXPOSURE_H_
#define LIBCAER_SRC_AUTOEXPOSURE_H_

#include "libcaer.h"
#include "events/frame.h"

#define AUTOEXPOSURE_HISTOGRAM_PIXELS 256
#define AUTOEXPOSURE_HISTOGRAM_MSV 5
#define AUTOEXPOSURE_LOW_BOUNDARY 0.25f
#define AUTOEXPOSURE_HIGH_BOUNDARY 0.75f
#define AUTOEXPOSURE_UNDEROVER_FRAC 0.20f
#define AUTOEXPOSURE_CHANGE_FRAC 0.10f

struct auto_exposure_state {
	size_t pixelHistogram[AUTOEXPOSURE_HISTOGRAM_PIXELS];
	size_t msvHistogram[AUTOEXPOSURE_HISTOGRAM_MSV];
	size_t pixelsMaxNonZeroBin;
};

typedef struct auto_exposure_state *autoExposureState;

// Returns next exposure value in µs, or -1 if currently set is optimal.
int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureLastSetValue);

#endif /* LIBCAER_SRC_AUTOEXPOSURE_H_ */
