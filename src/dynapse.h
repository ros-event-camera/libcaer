#ifndef LIBCAER_SRC_DYNAPSE_H_
#define LIBCAER_SRC_DYNAPSE_H_

#include "devices/dynapse.h"
#include "ringbuffer/ringbuffer.h"
#include <stdatomic.h>
#include <libusb.h>

#if defined(HAVE_PTHREADS)
#include "c11threads_posix.h"
#endif

#define DYNAPSE_DEVICE_NAME "Dynap-se"

#define DYNAPSE_DEVICE_VID 0x152A
#define DYNAPSE_DEVICE_PID 0x841D
#define DYNAPSE_DEVICE_DID_TYPE 0x00

#define DYNAPSE_REQUIRED_LOGIC_REVISION 1
#define DYNAPSE_REQUIRED_FIRMWARE_VERSION 1

#define DYNAPSE_EVENT_TYPES 2
#define DYNAPSE_SPIKE_EVENT_POS 1

#define DYNAPSE_SPIKE_DEFAULT_SIZE 4096
#define DYNAPSE_SPECIAL_DEFAULT_SIZE 128

#define DYNAPSE_DATA_ENDPOINT 0x82

#define VENDOR_REQUEST_FPGA_CONFIG          0xBF
#define VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE 0xC2

struct dynapse_state {
	// Data Acquisition Thread -> Mainloop Exchange
	RingBuffer dataExchangeBuffer;
	atomic_uint_fast32_t dataExchangeBufferSize; // Only takes effect on DataStart() calls!
	atomic_bool dataExchangeBlocking;
	atomic_bool dataExchangeStartProducers;
	atomic_bool dataExchangeStopProducers;
	void (*dataNotifyIncrease)(void *ptr);
	void (*dataNotifyDecrease)(void *ptr);
	void *dataNotifyUserPtr;
	void (*dataShutdownNotify)(void *ptr);
	void *dataShutdownUserPtr;
	// USB Device State
	char deviceThreadName[15 + 1]; // +1 for terminating NUL character.
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// USB Transfer Settings
	atomic_uint_fast32_t usbBufferNumber;
	atomic_uint_fast32_t usbBufferSize;
	// Data Acquisition Thread
	thrd_t dataAcquisitionThread;
	atomic_bool dataAcquisitionThreadRun;
	atomic_uint_fast32_t dataAcquisitionThreadConfigUpdate;
	struct libusb_transfer **dataTransfers;
	size_t dataTransfersLength;
	size_t activeDataTransfers;
	// Timestamp fields
	int32_t wrapOverflow;
	int32_t wrapAdd;
	int32_t lastTimestamp;
	int32_t currentTimestamp;
	// Packet Container state
	caerEventPacketContainer currentPacketContainer;
	atomic_uint_fast32_t maxPacketContainerPacketSize;
	atomic_uint_fast32_t maxPacketContainerInterval;
	int64_t currentPacketContainerCommitTimestamp;
	// Spike Packet state
	caerSpikeEventPacket currentSpikePacket;
	int32_t currentSpikePacketPosition;
	// Special Packet state
	caerSpecialEventPacket currentSpecialPacket;
	int32_t currentSpecialPacketPosition;
};

typedef struct dynapse_state *dynapseState;

struct dynapse_handle {
	uint16_t deviceType;
	// Information fields.
	struct caer_dynapse_info info;
	// State for data management.
	struct dynapse_state state;
};

typedef struct dynapse_handle *dynapseHandle;

caerDeviceHandle dynapseOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
bool dynapseClose(caerDeviceHandle handle);

bool dynapseSendDefaultConfig(caerDeviceHandle handle);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool dynapseConfigSet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool dynapseConfigGet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool dynapseDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool dynapseDataStop(caerDeviceHandle handle);
caerEventPacketContainer dynapseDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DYNAPSE_H_ */
