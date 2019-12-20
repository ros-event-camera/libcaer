#ifndef LIBCAER_SRC_DV_EXPLORER_H_
#define LIBCAER_SRC_DV_EXPLORER_H_

#include "libcaer/devices/device_discover.h"
#include "libcaer/devices/dv_explorer.h"

#include "container_generation.h"
#include "data_exchange.h"
#include "usb_utils.h"

#define IMU_TYPE_TEMP 0x01
#define IMU_TYPE_GYRO 0x02
#define IMU_TYPE_ACCEL 0x04
#define IMU_TOTAL_COUNT 14

#define IMU6_EVENT_PKT_POS 2
#define DV_EXPLORER_EVENT_TYPES 3

#define DV_EXPLORER_POLARITY_DEFAULT_SIZE 4096
#define DV_EXPLORER_SPECIAL_DEFAULT_SIZE 128
#define DV_EXPLORER_IMU_DEFAULT_SIZE 64

#define DV_EXPLORER_DEVICE_NAME "DV Explorer"

#define DV_EXPLORER_DEVICE_PID 0x8419
#define DV_EXPLORER_REQUIRED_LOGIC_VERSION 18
#define DV_EXPLORER_REQUIRED_LOGIC_PATCH_LEVEL 1
#define DV_EXPLORER_REQUIRED_FIRMWARE_VERSION 6

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

struct dv_explorer_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastY;
		uint16_t lastX;
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
	} dvs;
	struct {
		// IMU specific fields
		bool ignoreEvents;
		bool flipX;
		bool flipY;
		bool flipZ;
		uint8_t type;
		uint8_t count;
		uint8_t tmpData;
		float accelScale;
		float gyroScale;
		// Current composite events, for later copy, to not loose them on commits.
		struct caer_imu6_event currentEvent;
	} imu;
	// Packet Container state
	struct container_generation container;
	struct {
		// Polarity Packet state
		caerPolarityEventPacket polarity;
		int32_t polarityPosition;
		// IMU6 Packet state
		caerIMU6EventPacket imu6;
		int32_t imu6Position;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
	// Device timing data.
	struct {
		uint16_t logicClock;
		uint16_t usbClock;
		uint16_t clockDeviationFactor;
		float logicClockActual;
		float usbClockActual;
	} deviceClocks;
	struct {
		// Debug transfer support (FX3 only).
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct dv_explorer_state *dvExplorerState;

struct dv_explorer_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_dvx_info info;
	// State for data management, common to all DVEXPLORER.
	struct dv_explorer_state state;
};

typedef struct dv_explorer_handle *dvExplorerHandle;

ssize_t dvExplorerFind(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle dvExplorerOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
bool dvExplorerClose(caerDeviceHandle cdh);

bool dvExplorerSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool dvExplorerConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool dvExplorerConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool dvExplorerDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool dvExplorerDataStop(caerDeviceHandle handle);
caerEventPacketContainer dvExplorerDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DV_EXPLORER_H_ */
