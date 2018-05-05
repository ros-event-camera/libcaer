#include "filters/dvs_noise.h"

struct caer_filter_dvs_noise {
	// Hot Pixel filter (learning).
	bool hotPixelLearn;
	uint32_t hotPixelTime;
	uint32_t hotPixelCount;
	bool hotPixelLearningStarted;
	int64_t hotPixelLearningStartTime;
	uint32_t *hotPixelLearningMap;
	// Hot Pixel filter (filtering).
	bool hotPixelEnabled;
	size_t hotPixelArraySize;
	struct caer_filter_dvs_pixel *hotPixelArray;
	uint64_t hotPixelStat;
	// Background Activity filter.
	bool backgroundActivityEnabled;
	uint32_t backgroundActivityTime;
	uint64_t backgroundActivityStat;
	// Refractory Period filter.
	bool refractoryPeriodEnabled;
	uint32_t refractoryPeriodTime;
	uint64_t refractoryPeriodStat;
	// Maps and their sizes.
	uint16_t sizeX;
	uint16_t sizeY;
	int64_t timestampsMap[];
};

struct dvs_pixel_with_count {
	struct caer_filter_dvs_pixel address;
	uint32_t count;
};

static int hotPixelArrayCountCompare(const void *a, const void *b);
static void hotPixelGenerateArray(caerFilterDVSNoise noiseFilter);

caerFilterDVSNoise caerFilterDVSNoiseInitialize(uint16_t sizeX, uint16_t sizeY) {
	caerFilterDVSNoise noiseFilter = calloc(1,
		sizeof(struct caer_filter_dvs_noise) + (sizeX * sizeY * sizeof(int64_t)));
	if (noiseFilter == NULL) {
		return (NULL);
	}

	noiseFilter->sizeX = sizeX;
	noiseFilter->sizeY = sizeY;

	// Default values for filters.
	noiseFilter->hotPixelTime = 1000000; // 1 second.
	noiseFilter->hotPixelCount = 10000; // 10 KEvt in 1 second => 10 KHz.
	noiseFilter->backgroundActivityTime = 20000; // 20 milliseconds within neighborhood.
	noiseFilter->refractoryPeriodTime = 100; // 100 microseconds, max. pixel firing rate 10 KHz.

	return (noiseFilter);
}

void caerFilterDVSNoiseDestroy(caerFilterDVSNoise noiseFilter) {
	// Ensure hot pixel map is also destroyed if still present,
	// for example if learning never terminated.
	if (noiseFilter->hotPixelLearningMap != NULL) {
		free(noiseFilter->hotPixelLearningMap);
	}

	// Same for hot pixel array.
	if (noiseFilter->hotPixelArray != NULL) {
		free(noiseFilter->hotPixelArray);
	}

	free(noiseFilter);
}

void caerFilterDVSNoiseApply(caerFilterDVSNoise noiseFilter, caerPolarityEventPacket polarity) {
	// Nothing to process.
	if ((polarity == NULL) || (caerEventPacketHeaderGetEventValid(&polarity->packetHeader) == 0)) {
		return;
	}

	// Hot Pixel learning: initialize and store packet-level timestamp.
	if (noiseFilter->hotPixelLearn && !noiseFilter->hotPixelLearningStarted) {
		// Initialize hot pixel learning.
		noiseFilter->hotPixelLearningMap = calloc(noiseFilter->sizeX * noiseFilter->sizeY, sizeof(uint32_t));
		if (noiseFilter->hotPixelLearningMap == NULL) {
			caerLog(CAER_LOG_ERROR, "DVS Noise Filter",
				"HotPixel Learning: failed to allocate memory for learning map.");
			noiseFilter->hotPixelLearn = false; // Disable learning on failure.
		}
		else {
			noiseFilter->hotPixelLearningStarted = true;

			// Store start timestamp.
			caerPolarityEventConst firstEvent = caerPolarityEventPacketGetEventConst(polarity, 0);
			noiseFilter->hotPixelLearningStartTime = caerPolarityEventGetTimestamp64(firstEvent, polarity);
		}
	}

	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		size_t pixelIndex = (y * noiseFilter->sizeX) + x; // Target pixel.

		// Hot Pixel learning: determine which pixels are abnormally active,
		// by counting how many times they spike in a given time period. The
		// ones above a given threshold will be considered "hot".
		// This is done first, so that other filters (including the Hot Pixel
		// filter itself) don't influence the learning operation.
		if (noiseFilter->hotPixelLearningStarted) {
			noiseFilter->hotPixelLearningMap[pixelIndex]++;

			if (ts > (noiseFilter->hotPixelLearningStartTime + noiseFilter->hotPixelTime)) {
				// Enough time has passed, we can proceed with data evaluation.
				hotPixelGenerateArray(noiseFilter);

				// Done, reset and notify end of learning.
				free(noiseFilter->hotPixelLearningMap);
				noiseFilter->hotPixelLearningMap = NULL;

				noiseFilter->hotPixelLearningStarted = false;
				noiseFilter->hotPixelLearn = false;
			}
		}

		// Hot Pixel filter: filter out abnormally active pixels by their address.
		if (noiseFilter->hotPixelEnabled) {
			bool filteredOut = false;

			for (size_t i = 0; i < noiseFilter->hotPixelArraySize; i++) {
				if ((x == noiseFilter->hotPixelArray[i].x) && (y == noiseFilter->hotPixelArray[i].y)) {
					caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
					noiseFilter->hotPixelStat++;

					filteredOut = true;
					break;
				}
			}

			if (filteredOut) {
				// Go to next event, don't execute other filters and don't
				// update timestamps map. Hot pixels don't provide any useful
				// timing information, as they are repeating noise.
				continue;
			}
		}

		if (noiseFilter->backgroundActivityEnabled) {
			// Compute map limits.
			bool borderLeft = (x == 0);
			bool borderDown = (y == (noiseFilter->sizeY - 1));
			bool borderRight = (x == (noiseFilter->sizeX - 1));
			bool borderUp = (y == 0);

			// Background Activity filter: if difference between current timestamp
			// and stored neighbor timestamp is smaller than given time limit, it
			// means the event is supported by a neighbor and thus valid. If it is
			// bigger, then the event is not supported, and we need to check the
			// next neighbor. If all are bigger, the event is invalid.
			size_t index = pixelIndex - noiseFilter->sizeX;

			if (!borderUp) {
				if (!borderLeft && (ts - noiseFilter->timestampsMap[index - 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}

				if ((ts - noiseFilter->timestampsMap[index]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}

				if (!borderRight
					&& (ts - noiseFilter->timestampsMap[index + 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
			}
			index += noiseFilter->sizeX; // Next row.

			{
				if (!borderLeft && (ts - noiseFilter->timestampsMap[index - 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}

				if (!borderRight
					&& (ts - noiseFilter->timestampsMap[index + 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
			}
			index += noiseFilter->sizeX; // Next row.

			if (!borderDown) {
				if (!borderLeft && (ts - noiseFilter->timestampsMap[index - 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}

				if ((ts - noiseFilter->timestampsMap[index]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}

				if (!borderRight
					&& (ts - noiseFilter->timestampsMap[index + 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
			}
			// No index to adjust to next row.

			// Event is not supported by any neighbor if we get here, invalidate it.
			// Then jump over the Refractory Period filter, as it's useless to
			// execute it (and would cause a double-invalidate error).
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
			noiseFilter->backgroundActivityStat++;

			goto BAInvalidatedEvent;

		}

		// If we're sent here, the event was valid due to neighbor support.
		BAValidEvent:
		// Refractory Period filter.
		if (noiseFilter->refractoryPeriodEnabled) {
			if ((ts - noiseFilter->timestampsMap[pixelIndex]) < noiseFilter->refractoryPeriodTime) {
				caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
				noiseFilter->refractoryPeriodStat++;
			}
		}

		BAInvalidatedEvent:
		// Update pixel timestamp (one write). Always update so filters are
		// ready at enable-time right away.
		noiseFilter->timestampsMap[pixelIndex] = ts;
	}
}

bool caerFilterDVSNoiseConfigSet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t param) {
	switch (paramAddr) {
		case CAER_FILTER_DVS_HOTPIXEL_LEARN:
			noiseFilter->hotPixelLearn = param;
			break;

		case CAER_FILTER_DVS_HOTPIXEL_TIME:
			noiseFilter->hotPixelTime = U32T(param);
			break;

		case CAER_FILTER_DVS_HOTPIXEL_COUNT:
			noiseFilter->hotPixelCount = U32T(param);
			break;

		case CAER_FILTER_DVS_HOTPIXEL_ENABLE:
			noiseFilter->hotPixelEnabled = param;
			break;

		case CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE:
			noiseFilter->backgroundActivityEnabled = param;
			break;

		case CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME:
			noiseFilter->backgroundActivityTime = U32T(param);
			break;

		case CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE:
			noiseFilter->refractoryPeriodEnabled = param;
			break;

		case CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME:
			noiseFilter->refractoryPeriodTime = U32T(param);
			break;

		default:
			// Unrecognized or invalid parameter address.
			return (false);
			break;
	}

	// Done!
	return (true);
}

bool caerFilterDVSNoiseConfigGet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t *param) {
	// Ensure param is zeroed out.
	*param = 0;

	switch (paramAddr) {
		case CAER_FILTER_DVS_HOTPIXEL_LEARN:
			*param = noiseFilter->hotPixelLearn;
			break;

		case CAER_FILTER_DVS_HOTPIXEL_TIME:
			*param = noiseFilter->hotPixelTime;
			break;

		case CAER_FILTER_DVS_HOTPIXEL_COUNT:
			*param = noiseFilter->hotPixelCount;
			break;

		case CAER_FILTER_DVS_HOTPIXEL_ENABLE:
			*param = noiseFilter->hotPixelEnabled;
			break;

		case CAER_FILTER_DVS_HOTPIXEL_STATISTICS:
			*param = noiseFilter->hotPixelStat;
			break;

		case CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE:
			*param = noiseFilter->backgroundActivityEnabled;
			break;

		case CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME:
			*param = noiseFilter->backgroundActivityTime;
			break;

		case CAER_FILTER_DVS_BACKGROUND_ACTIVITY_STATISTICS:
			*param = noiseFilter->backgroundActivityStat;
			break;

		case CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE:
			*param = noiseFilter->refractoryPeriodEnabled;
			break;

		case CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME:
			*param = noiseFilter->refractoryPeriodTime;
			break;

		case CAER_FILTER_DVS_REFRACTORY_PERIOD_STATISTICS:
			*param = noiseFilter->refractoryPeriodStat;
			break;

		default:
			// Unrecognized or invalid parameter address.
			return (false);
			break;
	}

	// Done!
	return (true);
}

ssize_t caerFilterDVSNoiseConfigGetHotPixels(caerFilterDVSNoise noiseFilter, caerFilterDVSPixel *hotPixels) {
	*hotPixels = NULL;

	// No hot pixels listed.
	if (noiseFilter->hotPixelArraySize == 0) {
		return (0);
	}

	// Allocate memory for array copy.
	*hotPixels = malloc(noiseFilter->hotPixelArraySize * sizeof(struct caer_filter_dvs_pixel));
	if (*hotPixels == NULL) {
		// Memory allocation failure.
		return (-1);
	}

	// Copy pixel array over.
	memcpy(*hotPixels, noiseFilter->hotPixelArray,
		noiseFilter->hotPixelArraySize * sizeof(struct caer_filter_dvs_pixel));

	return ((ssize_t) noiseFilter->hotPixelArraySize);
}

static int hotPixelArrayCountCompare(const void *a, const void *b) {
	const struct dvs_pixel_with_count *aa = a;
	const struct dvs_pixel_with_count *bb = b;

	if (aa->count < bb->count) {
		return (-1);
	}
	else if (aa->count > bb->count) {
		return (1);
	}
	else {
		return (0);
	}
}

static void hotPixelGenerateArray(caerFilterDVSNoise noiseFilter) {
	// Remove old array, if present.
	if (noiseFilter->hotPixelArray != NULL) {
		free(noiseFilter->hotPixelArray);
		noiseFilter->hotPixelArray = NULL;
		noiseFilter->hotPixelArraySize = 0;
	}

	// Count number of hot pixels.
	size_t hotPixelsNumber = 0;

	for (size_t i = 0; i < (noiseFilter->sizeX * noiseFilter->sizeY); i++) {
		if (noiseFilter->hotPixelLearningMap[i] >= noiseFilter->hotPixelCount) {
			hotPixelsNumber++;
		}
	}

	// Store hot pixels with count, so we can then sort them by activity easily.
	struct dvs_pixel_with_count hotPixels[hotPixelsNumber];

	size_t idx = 0;
	for (size_t i = 0; i < (noiseFilter->sizeX * noiseFilter->sizeY); i++) {
		if (noiseFilter->hotPixelLearningMap[i] >= noiseFilter->hotPixelCount) {
			hotPixels[idx].address.x = U16T(i % noiseFilter->sizeX);
			hotPixels[idx].address.y = U16T(i / noiseFilter->sizeX);
			hotPixels[idx].count = noiseFilter->hotPixelLearningMap[i];
			idx++;
		}
	}

	qsort(hotPixels, hotPixelsNumber, sizeof(struct dvs_pixel_with_count), &hotPixelArrayCountCompare);

	// Allocate dynamic memory for new array.
	noiseFilter->hotPixelArray = malloc(hotPixelsNumber * sizeof(struct caer_filter_dvs_pixel));
	if (noiseFilter->hotPixelArray == NULL) {
		caerLog(CAER_LOG_ERROR, "DVS Noise Filter",
			"HotPixel Learning: failed to allocate memory for hot pixels array.");
		return;
	}

	// Set size and fill array with pre-sorted data.
	noiseFilter->hotPixelArraySize = hotPixelsNumber;

	for (size_t i = 0; i < hotPixelsNumber; i++) {
		noiseFilter->hotPixelArray[i].x = hotPixels[i].address.x;
		noiseFilter->hotPixelArray[i].y = hotPixels[i].address.y;
	}
}
