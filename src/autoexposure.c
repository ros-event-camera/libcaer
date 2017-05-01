#include "autoexposure.h"

int32_t autoExposureCalculate(autoExposureState state, caerFrameEventConst frame) {
	int32_t currentExposureTime = caerFrameEventGetExposureLength(frame);
	int32_t frameSizeX = caerFrameEventGetLengthX(frame);
	int32_t frameSizeY = caerFrameEventGetLengthY(frame);
	const uint16_t *framePixels = caerFrameEventGetPixelArrayUnsafeConst(frame);

	// Simplest implementation imaginable: try to push average color value to mid-gray.
	// Just for initial testing purposes, needs better algorithm.
	int64_t accumulator = 0;
	uint16_t maxVal = 0;

	for (int32_t y = 0; y < frameSizeY; y++) {
		for (int32_t x = 0; x < frameSizeX; x++) {
			accumulator += framePixels[(y * frameSizeX) + x];

			if (framePixels[(y * frameSizeX) + x] > maxVal) {
				maxVal = framePixels[(y * frameSizeX) + x];
			}
		}
	}

	accumulator /= (frameSizeX * frameSizeY);

	caerLog(CAER_LOG_WARNING, "AutoExposure", "Current exposure value is %" PRIi32 " µs.", currentExposureTime);
	caerLog(CAER_LOG_WARNING, "AutoExposure", "Accumulator value is %" PRIi64 ".", accumulator);
	caerLog(CAER_LOG_WARNING, "AutoExposure", "Maximum pixel value is %" PRIu16 ".", maxVal);

	if (accumulator > ((UINT16_MAX / 2) + 2000)) {
		return (currentExposureTime - 100);
	}
	else if (accumulator < ((UINT16_MAX / 2) - 2000)) {
		return (currentExposureTime + 100);
	}

	return (-1);
}
