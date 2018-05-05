#include "filters/dvs_noise.h"

struct caer_filter_dvs_noise {
	// Hot Pixel filter (learning).
	bool hotPixelLearn;
	uint32_t hotPixelTime;
	uint32_t hotPixelCount;
	uint32_t *hotPixelMap;
	// Hot Pixel filter (filtering).
	bool hotPixelEnabled;
	size_t hotPixelListSize;
	caerFilterDVSPixel hotPixelList;
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
	int64_t *timestampsMap[];
};

caerFilterDVSNoise caerFilterDVSNoiseInitialize(uint16_t sizeX, uint16_t sizeY) {
	caerFilterDVSNoise noiseFilter = calloc(1, sizeof(struct caer_filter_dvs_noise) + (sizeX * sizeof(int64_t *)));
	if (noiseFilter == NULL) {
		return (NULL);
	}

	noiseFilter->timestampsMap[0] = calloc(sizeX * sizeY, sizeof(int64_t));
	if (noiseFilter->timestampsMap[0] == NULL) {
		free(noiseFilter);
		return (NULL);
	}

	for (size_t i = 1; i < sizeX; i++) {
		noiseFilter->timestampsMap[i] = noiseFilter->timestampsMap[0] + (i * sizeY);
	}

	noiseFilter->sizeX = sizeX;
	noiseFilter->sizeY = sizeY;

	return (noiseFilter);
}

void caerFilterDVSNoiseDestroy(caerFilterDVSNoise noiseFilter) {
	// Ensure hot pixel map is also destroyed if still present,
	// for example if learning never terminated.
	if (noiseFilter->hotPixelMap != NULL) {
		free(noiseFilter->hotPixelMap);
	}

	free(noiseFilter->timestampsMap[0]);
	free(noiseFilter);
}

void caerFilterDVSNoiseApply(caerFilterDVSNoise noiseFilter, caerPolarityEventPacket polarity) {
	// Nothing to process.
	if (polarity == NULL) {
		return;
	}

	CAER_POLARITY_ITERATOR_ALL_START(polarity)
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);

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
			if (!borderLeft) {
				if (!borderUp
					&& (ts - noiseFilter->timestampsMap[x - 1][y - 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
				if ((ts - noiseFilter->timestampsMap[x - 1][y]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
				if (!borderDown
					&& (ts - noiseFilter->timestampsMap[x - 1][y + 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
			}

			if (!borderUp && (ts - noiseFilter->timestampsMap[x][y - 1]) < noiseFilter->backgroundActivityTime) {
				goto BAValidEvent;
			}
			if (!borderDown && (ts - noiseFilter->timestampsMap[x][y + 1]) < noiseFilter->backgroundActivityTime) {
				goto BAValidEvent;
			}

			if (!borderRight) {
				if (!borderUp
					&& (ts - noiseFilter->timestampsMap[x + 1][y - 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
				if ((ts - noiseFilter->timestampsMap[x + 1][y]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
				if (!borderDown
					&& (ts - noiseFilter->timestampsMap[x + 1][y + 1]) < noiseFilter->backgroundActivityTime) {
					goto BAValidEvent;
				}
			}

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
			if ((ts - noiseFilter->timestampsMap[x][y]) < noiseFilter->refractoryPeriodTime) {
				caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
				noiseFilter->refractoryPeriodStat++;
			}
		}

		BAInvalidatedEvent:
		// Update pixel timestamp (one write). Always update so filters are
		// ready at enable-time right away.
		noiseFilter->timestampsMap[x][y] = ts;
	}
}

bool caerFilterDVSNoiseConfigSet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t param) {
	switch (paramAddr) {
		case CAER_FILTER_DVS_HOTPIXEL_LEARN:

			break;

		case CAER_FILTER_DVS_HOTPIXEL_TIME:

			break;

		case CAER_FILTER_DVS_HOTPIXEL_COUNT:

			break;

		case CAER_FILTER_DVS_HOTPIXEL_ENABLE:

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
			break;

		case CAER_FILTER_DVS_HOTPIXEL_TIME:

			break;

		case CAER_FILTER_DVS_HOTPIXEL_COUNT:

			break;

		case CAER_FILTER_DVS_HOTPIXEL_ENABLE:

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
	return (0);
}
