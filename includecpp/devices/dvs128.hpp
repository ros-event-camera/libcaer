#ifndef LIBCAER_DEVICES_DVS128_HPP_
#define LIBCAER_DEVICES_DVS128_HPP_

#include <libcaer/devices/dvs128.h>
#include "usb.hpp"
#include "../events/polarity.hpp"
#include "../events/special.hpp"

namespace libcaer {
namespace devices {

class dvs128: public usb {
public:
	struct caer_dvs128_info infoGet() {
		return (caerDVS128InfoGet(handle));
	}
};

}
}

#endif /* LIBCAER_DEVICES_DVS128_HPP_ */
