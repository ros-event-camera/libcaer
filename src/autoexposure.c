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

	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Last set exposure value was: %d.", exposureLastSetValue);
	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Frame exposure value was: %d.", exposureFrameValue);

	int32_t frameSizeX = caerFrameEventGetLengthX(frame);
	int32_t frameSizeY = caerFrameEventGetLengthY(frame);
	const uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafeConst(frame);

	// Reset histograms.
	for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_PIXELS; i++) {
		state->pixelHistogram[i] = 0;
	}
	for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_MSV; i++) {
		state->msvHistogram[i] = 0;
	}

	// Min/Max pixel values.
	uint16_t minValue = UINT16_MAX;
	uint16_t maxValue = 0;

	// Fill histograms: 256 regions for pixel values; 5 regions for MSV.
	for (int32_t y = 0; y < frameSizeY; y++) {
		for (int32_t x = 0; x < frameSizeX; x++) {
			uint16_t pixelValue = framePixels[(y * frameSizeX) + x];

			// Track min/max pixel values.
			if (pixelValue < minValue) {
				minValue = pixelValue;
			}
			if (pixelValue > maxValue) {
				maxValue = pixelValue;
			}

			// Update histograms.
			size_t pixelIndex = pixelValue / ((UINT16_MAX + 1) / AUTOEXPOSURE_HISTOGRAM_PIXELS);
			state->pixelHistogram[pixelIndex]++;

			size_t msvIndex = pixelValue / ((UINT16_MAX + 1) / AUTOEXPOSURE_HISTOGRAM_MSV);
			state->msvHistogram[msvIndex] += pixelValue;
		}
	}

	// Calculate mean sample value from histogram.
	float meanSampleValueNum = 0, meanSampleValueDenom = 0;
	for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_MSV; i++) {
		meanSampleValueNum += ((float) i + 1.0f) * (float) state->pixelHistogram[i];
		meanSampleValueDenom += (float) state->pixelHistogram[i];
	}

	float meanSampleValue = meanSampleValueNum / meanSampleValueDenom;
	float meanSampleValueError = (AUTOEXPOSURE_HISTOGRAM_MSV / 2.0f) - meanSampleValue;

	caerLog(CAER_LOG_DEBUG, "AutoExposure", "Mean sample value error is: %f.", (double) meanSampleValueError);

	// Calculate statistics on pixel histogram.
	size_t pixelsSum = 0;
	size_t pixelsWeightedSum = 0;

	for (size_t i = 0; i < AUTOEXPOSURE_HISTOGRAM_PIXELS; i++) {
		size_t v = state->pixelHistogram[i];
		pixelsSum += v;
		pixelsWeightedSum += i * v;

		if (v > 0) {
			if (i > state->pixelsMaxNonZeroBin) {
				state->pixelsMaxNonZeroBin = i;
			}
		}
	}

	size_t pixelsMeanBin = 0;

	if (pixelsSum <= 0) {
		pixelsMeanBin = AUTOEXPOSURE_HISTOGRAM_PIXELS / 2;
	}
	else {
		pixelsMeanBin = pixelsWeightedSum / pixelsSum;
	}

	size_t pixels10Bin = (size_t) (AUTOEXPOSURE_LOW_BOUNDARY * (float) state->pixelsMaxNonZeroBin);
	size_t pixels90Bin = (size_t) (AUTOEXPOSURE_HIGH_BOUNDARY * (float) state->pixelsMaxNonZeroBin);

	float pixels10Sum = 0, pixels90Sum = 0;
	float pixelsFracLow = 0, pixelsFracHigh = 0;

	if (pixelsSum > 0) {
		for (size_t i = 0; i <= pixels10Bin; i++) {
			pixels10Sum += (float) state->pixelHistogram[i];
		}

		for (size_t i = pixels90Bin; i <= state->pixelsMaxNonZeroBin; i++) {
			pixels90Sum += (float) state->pixelHistogram[i];
		}

		pixelsFracLow = pixels10Sum / (float) pixelsSum;
		pixelsFracHigh = pixels90Sum / (float) pixelsSum;
	}

	caerLog(CAER_LOG_DEBUG, "AutoExposure",
		"MaxNZBin: %zu, Bin10: %zu, Bin90: %zu, Sum: %zu, Sum10: %f, Sum90: %f, FracLow: %f, FracHigh: %f.",
		state->pixelsMaxNonZeroBin, pixels10Bin, pixels90Bin, pixelsSum, (double) pixels10Sum, (double) pixels90Sum,
		(double) pixelsFracLow, (double) pixelsFracHigh);

	float pixelsError = (float) (pixelsMeanBin - (state->pixelsMaxNonZeroBin / 2)) / (float) state->pixelsMaxNonZeroBin;

	// Exposure okay by default.
	int32_t newExposure = -1;

	if ((pixelsFracLow >= AUTOEXPOSURE_UNDEROVER_FRAC) && (pixelsFracHigh < AUTOEXPOSURE_UNDEROVER_FRAC)) {
		newExposure = I32T((float) exposureLastSetValue * (1.0f + AUTOEXPOSURE_CHANGE_FRAC));

		if (newExposure == I32T(exposureLastSetValue)) {
			newExposure++; // Ensure increase.
		}
		if (newExposure > 1000000) {
			newExposure = 1000000;
		}
	}
	else if ((pixelsFracLow < AUTOEXPOSURE_UNDEROVER_FRAC) && (pixelsFracHigh >= AUTOEXPOSURE_UNDEROVER_FRAC)) {
		newExposure = I32T((float) exposureLastSetValue * (1.0f - AUTOEXPOSURE_CHANGE_FRAC));

		if (newExposure == I32T(exposureLastSetValue)) {
			newExposure--; // Ensure decrease.
		}
		if (newExposure < 0) {
			newExposure = 0;
		}
	}

	return ((newExposure == I32T(exposureLastSetValue)) ? (-1) : (newExposure));
}
