#ifndef LIBCAER_SRC_USB_UTILS_H_
#define LIBCAER_SRC_USB_UTILS_H_

#include "libcaer.h"
#include <libusb.h>
#include <stdatomic.h>

#if defined(HAVE_PTHREADS)
#include "c11threads_posix.h"
#endif

#define MAX_THREAD_NAME_LENGTH 15

#define USB_DEFAULT_DEVICE_VID 0x152A

#define USB_DEFAULT_DATA_ENDPOINT 0x82

#define VENDOR_REQUEST_FPGA_CONFIG          0xBF
#define VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE 0xC2

struct usb_state {
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// USB thread state
	char usbThreadName[15 + 1]; // +1 for terminating NUL character.
	thrd_t usbThread;
	atomic_bool usbThreadRun;
	// USB Transfer Settings
	atomic_uint_fast32_t usbBufferNumber;
	atomic_uint_fast32_t usbBufferSize;
	// USB Data Transfers
	uint8_t dataEndPoint;
	mtx_t dataTransfersLock;
	struct libusb_transfer **dataTransfers; // LOCK PROTECTED.
	uint32_t dataTransfersLength; // LOCK PROTECTED.
	atomic_uint_fast32_t activeDataTransfers;
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


bool usbDeviceOpen(usbState state, uint16_t devVID, uint16_t devPID, uint8_t busNumber, uint8_t devAddress,
	const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion);
void usbDeviceClose(usbState state);

void usbSetThreadName(usbState state, const char *threadName);
void usbSetUserCallback(usbState state, void (*userCallback)(void *userData, uint8_t *buffer, size_t bytesSent),
	void *userData);
void usbSetDataEndpoint(usbState state, uint8_t dataEndPoint);
void usbSetTransfersNumber(usbState state, uint32_t transfersNumber);
void usbSetTransfersSize(usbState state, uint32_t transfersSize);
uint32_t usbGetTransfersNumber(usbState state);
uint32_t usbGetTransfersSize(usbState state);
libusb_device_handle *usbGetDeviceHandle(usbState state);

struct usb_info usbGenerateInfo(usbState state, const char *deviceName, uint16_t deviceID);

void usbAllocateTransfers(usbState state);
void usbCancelTransfersAsync(usbState state);
void usbDeallocateTransfers(usbState state);

static inline bool usbThreadIsRunning(usbState state) {
	return (atomic_load(&state->usbThreadRun));
}
bool usbThreadStart(usbState state, int (*usbThread)(void *inPtr), void *inPtr);
bool usbThreadStop(usbState state);
void usbThreadRun(usbState state);

bool spiConfigSend(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
bool spiConfigSendAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
bool spiConfigReceive(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param);
bool spiConfigReceiveAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr,
	void (*userCallback)(void *userData, uint32_t param), void *userData);

#endif /* LIBCAER_SRC_USB_UTILS_H_ */
