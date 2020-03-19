#ifndef LIBCAER_DEVICES_MIPI_CX3_HPP_
#define LIBCAER_DEVICES_MIPI_CX3_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"

#include <libcaer/devices/mipi_cx3.h>

#include "usb.hpp"

namespace libcaer {
namespace devices {

class mipiCx3 : public usb {
public:
	mipiCx3(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_MIPI_CX3) {
	}

	mipiCx3(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, CAER_DEVICE_MIPI_CX3, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_mipi_cx3_info infoGet() const noexcept {
		return (caerMipiCx3InfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_MIPI_CX3_HPP_ */
