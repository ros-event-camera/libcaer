#ifndef LIBCAER_DEVICES_DYNAPSE_HPP_
#define LIBCAER_DEVICES_DYNAPSE_HPP_

#include <libcaer/devices/dynapse.h>
#include "usb.hpp"
#include "../events/spike.hpp"
#include "../events/special.hpp"

namespace libcaer {
namespace devices {

class dynapse: public usb {
public:
	struct caer_dynapse_info infoGet() {
		return (caerDynapseInfoGet(handle));
	}

	void sendDataToUSB(int *data, int numConfig) {
		bool success = caerDynapseSendDataToUSB(handle, data, numConfig);
		if (!success) {
			throw std::runtime_error("Failed to send config data to device.");
		}
	}

	void writeSRAM(uint16_t *data, uint32_t baseAddr, uint32_t numWords) {
		bool success = caerDynapseWriteSRAM(handle, data, baseAddr, numWords);
		if (!success) {
			throw std::runtime_error("Failed to write SRAM.");
		}
	}

	void writeCam(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType) {
		bool success = caerDynapseWriteCam(handle, preNeuronAddr, postNeuronAddr, camId, synapseType);
		if (!success) {
			throw std::runtime_error("Failed to write CAM.");
		}
	}

	uint32_t generateCamBits(uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType) {
		return (caerDynapseGenerateCamBits(preNeuronAddr, postNeuronAddr, camId, synapseType));
	}
};

}
}

#endif /* LIBCAER_DEVICES_DYNAPSE_HPP_ */
