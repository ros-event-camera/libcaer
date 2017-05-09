#include "autoexposure.h"

int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureLastSetValue) {
	// Wait for frames to actually catch up with last set value.
	uint32_t exposureFrameValue = U32T(caerFrameEventGetExposureLength(frame));
	uint32_t exposureLastSetValueEpsilonLower = (exposureLastSetValue < 10) ? (0) : (exposureLastSetValue - 10);
	uint32_t exposureLastSetValueEpsilonUpper = exposureLastSetValue + 10;

	if (exposureFrameValue < exposureLastSetValueEpsilonLower
		|| exposureFrameValue > exposureLastSetValueEpsilonUpper) {
		return (-1);
	}

	int32_t frameSizeX = caerFrameEventGetLengthX(frame);
	int32_t frameSizeY = caerFrameEventGetLengthY(frame);
	const uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafeConst(frame);

	// Reset histogram.
	for (size_t i = 0; i < AUTOEXPOSURE_MIDDLEGRAY_MSV; i++) {
		state->pixelHistogram[i] = 0;
	}

	// Fill histogram with 5 regions for MSV.
	for (int32_t y = 0; y < frameSizeY; y++) {
		for (int32_t x = 0; x < frameSizeX; x++) {
			uint16_t pixelValue = framePixels[(y * frameSizeX) + x];

			size_t index = pixelValue / ((UINT16_MAX + 1) / AUTOEXPOSURE_MIDDLEGRAY_MSV);

			state->pixelHistogram[index] += pixelValue;
		}
	}

	// Histogram mean sample value.
	float meanSampleValueNum = 0, meanSampleValueDenom = 0;
	for (size_t i = 0; i < AUTOEXPOSURE_MIDDLEGRAY_MSV; i++) {
		meanSampleValueNum += ((float) i + 1.0f) * (float) state->pixelHistogram[i];
		meanSampleValueDenom += (float) state->pixelHistogram[i];
	}

	float meanSampleValue = meanSampleValueNum / meanSampleValueDenom;
	float meanSampleValueError = (AUTOEXPOSURE_MIDDLEGRAY_MSV / 2.0f) - meanSampleValue;

	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Mean sample value error is: %f.", (double) meanSampleValueError);
	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Last set exposure value was: %d.", exposureLastSetValue);
	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Frame exposure value was: %d.", caerFrameEventGetExposureLength(frame));

	if ((meanSampleValueError > 0.1f && state->previousError > 0
		&& meanSampleValueError > (state->previousError + 0.001f))
		|| (meanSampleValueError < -0.1f && state->previousError < 0
			&& meanSampleValueError < (state->previousError - 0.001f))) {
		// Error is getting worse and worse in same direction.
		// Did we hit a local minima before? Stop now!
		state->previousError = meanSampleValueError;
		return (-1);
	}

	// Exposure okay by default.
	int32_t newExposure = -1;

	if (meanSampleValueError > 0.1f) {
		// Underexposed.
		newExposure = I32T(exposureLastSetValue) + I32T(200.0f * meanSampleValueError * meanSampleValueError);

		// Clip exposure at maximum (1s = 1000000µs).
		if (newExposure > 1000000) {
			newExposure = 1000000;
		}
	}
	else if (meanSampleValueError < -0.1f) {
		// Overexposed.
		newExposure = I32T(exposureLastSetValue) - I32T(200.0f * meanSampleValueError * meanSampleValueError);

		// Clip exposure at minimum (0µs).
		if (newExposure < 0) {
			newExposure = 0;
		}
	}

	state->previousError = meanSampleValueError;
	return (newExposure);
}
