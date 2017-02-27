#ifndef LIBCAER_DEVICES_USB_HPP_
#define LIBCAER_DEVICES_USB_HPP_

#include <libcaer/devices/usb.h>
#include <stdexcept>

namespace libcaer {
namespace devices {

class usb {
protected:
	usb(uint16_t deviceID, uint16_t deviceType) :
		usb(deviceID, deviceType, 0, 0, std::string()) {
	}

	usb(uint16_t deviceID, uint16_t deviceType, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
		const std::string &serialNumberRestrict) {
		handle = caerDeviceOpen(deviceID, deviceType, busNumberRestrict, devAddressRestrict,
			(serialNumberRestrict.empty()) ? (nullptr) : (serialNumberRestrict.c_str()));

		// Handle constructor failure.
		if (handle == nullptr) {
			throw std::runtime_error("Failed to open device.");
		}
	}

public:
	// This can be called from base class pointers!
	~usb() {
		// Run destructors, free all memory.
		// Never fails in current implementation.
		caerDeviceClose(&handle);
	}

	void sendDefaultConfig() {
		bool success = caerDeviceSendDefaultConfig(handle);
		if (!success) {
			throw std::runtime_error("Failed to send default configuration.");
		}
	}

	void configSet(int8_t modAddr, uint8_t paramAddr, uint32_t param) {
		bool success = caerDeviceConfigSet(handle, modAddr, paramAddr, param);
		if (!success) {
			throw std::runtime_error("Failed to set configuration parameter.");
		}
	}

	void configGet(int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
		bool success = caerDeviceConfigGet(handle, modAddr, paramAddr, param);
		if (!success) {
			throw std::runtime_error("Failed to get configuration parameter.");
		}
	}

	uint32_t configGet(int8_t modAddr, uint8_t paramAddr) {
		uint32_t param = 0;
		configGet(modAddr, paramAddr, &param);
		return (param);
	}

	void dataStart() {
		dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);
	}

	void dataStart(void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
		void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) {
		bool success = caerDeviceDataStart(handle, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr,
			dataShutdownNotify, dataShutdownUserPtr);
		if (!success) {
			throw std::runtime_error("Failed to start getting data.");
		}
	}

	void dataStop() {
		bool success = caerDeviceDataStop(handle);
		if (!success) {
			throw std::runtime_error("Failed to stop getting data.");
		}
	}

	caerEventPacketContainer dataGet() {
		return (caerDeviceDataGet(handle));
	}

protected:
	caerDeviceHandle handle;
};

}
}

#endif /* LIBCAER_DEVICES_USB_HPP_ */
