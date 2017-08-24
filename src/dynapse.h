#ifndef LIBCAER_SRC_DYNAPSE_H_
#define LIBCAER_SRC_DYNAPSE_H_

#include "devices/dynapse.h"
#include "ringbuffer.h"
#include "usb_utils.h"

#define DYNAPSE_DEVICE_NAME "Dynap-se"

#define DYNAPSE_DEVICE_PID 0x841D

#define DYNAPSE_REQUIRED_LOGIC_REVISION 4 
#define DYNAPSE_REQUIRED_FIRMWARE_VERSION 3

#define VENDOR_REQUEST_FPGA_CONFIG_AER          0xC5
#define VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE 0xC6

#define DYNAPSE_EVENT_TYPES 2
#define DYNAPSE_SPIKE_EVENT_POS 1

#define DYNAPSE_SPIKE_DEFAULT_SIZE 4096
#define DYNAPSE_SPECIAL_DEFAULT_SIZE 128

// Chip 0 ID as output via USB.
#define DYNAPSE_CONFIG_DYNAPSE_U0_OUT	1 // DYNAPSE_U0+1 -> 0+1 = 1.
// Chip 1 ID as output via USB.
#define DYNAPSE_CONFIG_DYNAPSE_U1_OUT	8
// Chip 2 ID as output via USB.
#define DYNAPSE_CONFIG_DYNAPSE_U2_OUT	4
// Chip 3 ID as output via USB.
#define DYNAPSE_CONFIG_DYNAPSE_U3_OUT	12

struct dynapse_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	caerRingBuffer dataExchangeBuffer;
	atomic_uint_fast32_t dataExchangeBufferSize; // Only takes effect on DataStart() calls!
	atomic_bool dataExchangeBlocking;
	atomic_bool dataExchangeStartProducers;
	atomic_bool dataExchangeStopProducers;
	void (*dataNotifyIncrease)(void *ptr);
	void (*dataNotifyDecrease)(void *ptr);
	void *dataNotifyUserPtr;
	// USB Device State
	struct usb_state usbState;
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
