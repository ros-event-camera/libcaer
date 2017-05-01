#ifndef LIBCAER_SRC_AUTOEXPOSURE_H_
#define LIBCAER_SRC_AUTOEXPOSURE_H_

#include "libcaer.h"
#include "events/frame.h"

struct auto_exposure_state {

};

typedef struct auto_exposure_state *autoExposureState;

// Returns next exposure value, or -1 if currently set is optimal.
int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame);

#endif /* LIBCAER_SRC_AUTOEXPOSURE_H_ */
