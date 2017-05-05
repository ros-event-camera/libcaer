#include "autoexposure.h"

int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame, uint32_t exposureLastSetValue) {
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

	int32_t newExposure = -1;

	if (meanSampleValueError > 0.1f) {
		// Underexposed.
		newExposure = I32T(exposureLastSetValue) + I32T(2000.0f * meanSampleValueError * meanSampleValueError);

		// Clip exposure at maximum (1s = 1000000µs).
		if (newExposure > 1000000) {
			newExposure = 1000000;
		}
	}
	else if (meanSampleValueError < -0.1f) {
		// Overexposed.
		newExposure = I32T(exposureLastSetValue) - I32T(2000.0f * meanSampleValueError * meanSampleValueError);

		// Clip exposure at minimum (0µs).
		if (newExposure < 0) {
			newExposure = 0;
		}
	}

	// Exposure okay.
	return (newExposure);
}
