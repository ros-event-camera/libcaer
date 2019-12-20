#ifndef LIBCAER_DEVICES_DV_EXPLORER_S_HPP_
#define LIBCAER_DEVICES_DV_EXPLORER_S_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"

#include <libcaer/devices/dv_explorer_s.h>

#include "usb.hpp"

namespace libcaer {
namespace devices {

class dvExplorerS : public usb {
public:
	dvExplorerS(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_DV_EXPLORER_S) {
	}

	dvExplorerS(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, CAER_DEVICE_DV_EXPLORER_S, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dvx_s_info infoGet() const noexcept {
		return (caerDVExplorerSInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_DV_EXPLORER_S_HPP_ */
