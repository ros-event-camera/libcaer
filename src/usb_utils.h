#ifndef LIBCAER_SRC_USB_UTILS_H_
#define LIBCAER_SRC_USB_UTILS_H_

#include "libcaer.h"
#include <libusb.h>

#define USB_DEFAULT_DEVICE_VID 0x152A

#define USB_DEFAULT_DATA_ENDPOINT 0x82

#define VENDOR_REQUEST_FPGA_CONFIG          0xBF
#define VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE 0xC2

struct usb_state {
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// USB Data Transfers
	struct libusb_transfer **dataTransfers;
	size_t dataTransfersLength;
	size_t activeDataTransfers;
	// User data pointer/callback
	void *userData;
	void (*userCallback)(void *handle, uint8_t *buffer, size_t bytesSent);
};

typedef struct usb_state *usbState;

struct usb_info {
	uint8_t busNumber;
	uint8_t devAddress;
	char serialNumber[8 + 1];
	char *deviceString;
};

struct usb_info usbGenerateInfo(libusb_device_handle *devHandle, const char *deviceName, uint16_t deviceID);
bool spiConfigSend(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
bool spiConfigReceive(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param);
libusb_device_handle *usbDeviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID,
	uint8_t busNumber, uint8_t devAddress, const char *serialNumber, int32_t requiredLogicRevision,
	int32_t requiredFirmwareVersion);
void usbDeviceClose(libusb_device_handle *devHandle);
void usbAllocateTransfers(usbState state, uint32_t bufferNum, uint32_t bufferSize, uint8_t dataEndPoint);
void usbDeallocateTransfers(usbState state);

#endif /* LIBCAER_SRC_USB_UTILS_H_ */
