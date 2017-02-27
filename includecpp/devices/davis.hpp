#ifndef LIBCAER_DEVICES_DAVIS_HPP_
#define LIBCAER_DEVICES_DAVIS_HPP_

#include <libcaer/devices/davis.h>
#include "usb.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"
#include "../events/frame.hpp"
#include "../events/imu6.hpp"
#include "../events/sample.hpp"

namespace libcaer {
namespace devices {

class davis: public usb {
protected:
	// Forward construction to base class.
	davis(uint16_t deviceID, uint16_t deviceType) :
		usb(deviceID, deviceType, 0, 0, std::string()) {
	}

	davis(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, deviceType, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

public:
	struct caer_davis_info infoGet() {
		return (caerDavisInfoGet(handle));
	}

	// STATIC.
	static uint16_t biasVDACGenerate(struct caer_bias_vdac vdacBias) {
		return (caerBiasVDACGenerate(vdacBias));
	}

	static struct caer_bias_vdac biasVDACParse(uint16_t vdacBias) {
		return (caerBiasVDACParse(vdacBias));
	}

	static uint16_t biasCoarseFineGenerate(struct caer_bias_coarsefine coarseFineBias) {
		return (caerBiasCoarseFineGenerate(coarseFineBias));
	}

	static struct caer_bias_coarsefine biasCoarseFineParse(uint16_t coarseFineBias) {
		return (caerBiasCoarseFineParse(coarseFineBias));
	}

	static uint16_t biasShiftedSourceGenerate(struct caer_bias_shiftedsource shiftedSourceBias) {
		return (caerBiasShiftedSourceGenerate(shiftedSourceBias));
	}

	static struct caer_bias_shiftedsource biasShiftedSourceParse(uint16_t shiftedSourceBias) {
		return (caerBiasShiftedSourceParse(shiftedSourceBias));
	}
};

class davisfx2: public davis {
public:
	davisfx2(uint16_t deviceID) :
		davis(deviceID, CAER_DEVICE_DAVIS_FX2) {
	}

	davisfx2(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		davis(deviceID, CAER_DEVICE_DAVIS_FX2, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}
};

class davisfx3: public davis {
public:
	davisfx3(uint16_t deviceID) :
		davis(deviceID, CAER_DEVICE_DAVIS_FX3) {
	}

	davisfx3(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		davis(deviceID, CAER_DEVICE_DAVIS_FX3, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}
};

}
}

#endif /* LIBCAER_DEVICES_DAVIS_HPP_ */
