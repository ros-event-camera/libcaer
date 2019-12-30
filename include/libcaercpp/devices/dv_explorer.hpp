#ifndef LIBCAER_DEVICES_DV_EXPLORER_HPP_
#define LIBCAER_DEVICES_DV_EXPLORER_HPP_

#include "../events/polarity.hpp"
#include "../events/special.hpp"
#include "../events/imu6.hpp"

#include <libcaer/devices/dv_explorer.h>

#include "usb.hpp"

namespace libcaer {
namespace devices {

class dvExplorer : public usb {
public:
	dvExplorer(uint16_t deviceID) : usb(deviceID, CAER_DEVICE_DV_EXPLORER) {
	}

	dvExplorer(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) :
		usb(deviceID, CAER_DEVICE_DV_EXPLORER, busNumberRestrict, devAddressRestrict, serialNumberRestrict) {
	}

	struct caer_dvx_info infoGet() const noexcept {
		return (caerDVExplorerInfoGet(handle.get()));
	}

	std::string toString() const noexcept override {
		return (infoGet().deviceString);
	}
};
} // namespace devices
} // namespace libcaer

#endif /* LIBCAER_DEVICES_DV_EXPLORER_HPP_ */
