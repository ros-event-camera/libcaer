#include "filters/dvs_noise.h"

struct caer_filter_dvs_noise {

};

caerFilterDVSNoise caerFilterDVSNoiseInitialize(void) {

}

void caerFilterDVSNoiseDestroy(caerFilterDVSNoise noiseFilter) {

}

void caerFilterDVSNoiseApply(caerFilterDVSNoise noiseFilter, caerPolarityEventPacket polarity) {

}

bool caerFilterDVSNoiseConfigSet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t param) {

}

bool caerFilterDVSNoiseConfigGet(caerFilterDVSNoise noiseFilter, uint8_t paramAddr, uint64_t *param) {

}

ssize_t caerFilterDVSNoiseConfigGetHotPixels(caerFilterDVSNoise noiseFilter, caerFilterDVSPixel *hotPixels) {

}
