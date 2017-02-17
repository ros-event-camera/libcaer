#ifndef LIBCAER_SRC_DAS1V4_H_
#define LIBCAER_SRC_DAS1V4_H_

#include "devices/das1v4.h"
#include "ringbuffer/ringbuffer.h"
#include "usb_utils.h"
#include <stdatomic.h>

#if defined(HAVE_PTHREADS)
	#include "c11threads_posix.h"
#endif

#define DAS1V4_DEVICE_NAME "CochleaAMS1cV4"

#define DAS1V4_DEVICE_PID 0x8406

#define VENDOR_REQUEST_FPGA_CONFIG_AER          0xC5
#define VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE 0xC6

#define DAS1V4_EVENT_TYPES 2
#define DAS1V4_SPIKE_EVENT_POS 1

#define DAS1V4_SPIKE_DEFAULT_SIZE 4096
#define DAS1V4_SPECIAL_DEFAULT_SIZE 128

#define DAS1V4_REQUIRED_LOGIC_REVISION 0
#define DAS1V4_REQUIRED_FIRMWARE_VERSION 0

struct das1v4_state {
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
	struct usb_state usbState;
	// USB Transfer Settings
	atomic_uint_fast32_t usbBufferNumber;
	atomic_uint_fast32_t usbBufferSize;
	// Data Acquisition Thread
	thrd_t dataAcquisitionThread;
	atomic_bool dataAcquisitionThreadRun;
	atomic_uint_fast32_t dataAcquisitionThreadConfigUpdate;
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

typedef struct das1v4_state *das1v4State;

struct das1v4_handle {
	uint16_t deviceType;
	// Information fields.
	struct caer_das1v4_info info;
	// State for data management.
	struct das1v4_state state;
};

typedef struct das1v4_handle *das1v4Handle;

caerDeviceHandle das1v4Open(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
bool das1v4Close(caerDeviceHandle handle);

bool das1v4SendDefaultConfig(caerDeviceHandle handle);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool das1v4ConfigSet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool das1v4ConfigGet(caerDeviceHandle handle, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool das1v4DataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool das1v4DataStop(caerDeviceHandle handle);
caerEventPacketContainer das1v4DataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DAS1V4_H_ */
