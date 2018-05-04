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
	uint8_t subSampleFactor;
	uint16_t sizeXSubSampled;
	uint16_t sizeYSubSampled;
	int64_t *timestampsMap[];
};

caerFilterDVSNoise caerFilterDVSNoiseInitialize(uint16_t sizeX, uint16_t sizeY, uint8_t subSampleFactor) {
	uint16_t sizeXSubSampled = (sizeX >> subSampleFactor);
	uint16_t sizeYSubSampled = (sizeY >> subSampleFactor);

	caerFilterDVSNoise noiseFilter = malloc(
		sizeof(struct caer_filter_dvs_noise) + (sizeXSubSampled * sizeof(int64_t *)));
	if (noiseFilter == NULL) {
		return (NULL);
	}

	noiseFilter->timestampsMap[0] = calloc(sizeXSubSampled * sizeYSubSampled, sizeof(int64_t));
	if (noiseFilter->timestampsMap[0] == NULL) {
		free(noiseFilter);
		return (NULL);
	}

	for (size_t i = 1; i < sizeXSubSampled; i++) {
		noiseFilter->timestampsMap[i] = noiseFilter->timestampsMap[0] + (i * sizeYSubSampled);
	}

	noiseFilter->sizeX = sizeX;
	noiseFilter->sizeY = sizeY;
	noiseFilter->subSampleFactor = subSampleFactor;
	noiseFilter->sizeXSubSampled = sizeXSubSampled;
	noiseFilter->sizeYSubSampled = sizeYSubSampled;

	return (noiseFilter);
}

void caerFilterDVSNoiseDestroy(caerFilterDVSNoise noiseFilter) {
	free(noiseFilter->timestampsMap[0]);
	free(noiseFilter);
}

void caerFilterDVSNoiseApply(caerFilterDVSNoise noiseFilter, caerPolarityEventPacket polarity) {
	// Nothing to process.
	if (polarity == NULL) {
		return;
	}

	CAER_POLARITY_ITERATOR_ALL_START(polarity)
		uint16_t x = (caerPolarityEventGetX(caerPolarityIteratorElement) >> noiseFilter->subSampleFactor);
		uint16_t y = (caerPolarityEventGetY(caerPolarityIteratorElement) >> noiseFilter->subSampleFactor);
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);

		if (noiseFilter->backgroundActivityEnabled) {
			// Compute map limits.
			bool borderLeft = (x == 0);
			bool borderDown = (y == (noiseFilter->sizeYSubSampled - 1));
			bool borderRight = (x == (noiseFilter->sizeXSubSampled - 1));
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
		// Update pixel timestamp (one write).
		if (noiseFilter->backgroundActivityEnabled || noiseFilter->refractoryPeriodEnabled) {
			noiseFilter->timestampsMap[x][y] = ts;
		}
	}
}

bool caerFilterDVSNoiseConfigSet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t param) {

}

bool caerFilterDVSNoiseConfigGet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t *param) {

}

ssize_t caerFilterDVSNoiseConfigGetHotPixels(caerFilterDVSNoise noiseFilter, caerFilterDVSPixel *hotPixels) {

}
