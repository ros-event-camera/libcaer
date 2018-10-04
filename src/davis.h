#ifndef LIBCAER_SRC_DAVIS_H_
#define LIBCAER_SRC_DAVIS_H_

#include "devices/davis.h"
#include "devices/device_discover.h"

#include "filters/dvs_noise.h"

#include "autoexposure.h"
#include "container_generation.h"
#include "data_exchange.h"
#include "usb_utils.h"

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET 0
#define APS_READOUT_SIGNAL 1

/**
 * Enable APS frame debugging by only looking at the reset or signal
 * frames, and not at the resulting correlated frame.
 * Supported values:
 * 0 - normal output (ROI region 0), no debug (default)
 * 1 - normal output (ROI region 0), and in addition both reset and
 *     signal separately (marked as ROI regions 1 for reset and 2
 *     for signal respectively)
 */
#define APS_DEBUG_FRAME 1

#define APS_ADC_DEPTH 10

#define APS_ADC_CHANNELS 1

#define APS_ROI_REGIONS 1

#define IMU6_COUNT 15

#define DVS_HOTPIXEL_HW_MAX 8

#define SPI_CONFIG_MSG_SIZE 6

#define DAVIS_EVENT_TYPES 4

#define DAVIS_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_FRAME_DEFAULT_SIZE 8
#define DAVIS_IMU_DEFAULT_SIZE 64

#define DAVIS_DEVICE_NAME "DAVIS"

#define DAVIS_FX2_DEVICE_PID 0x841B
#define DAVIS_FX2_REQUIRED_LOGIC_REVISION 16
#define DAVIS_FX2_REQUIRED_FIRMWARE_VERSION 4

#define DAVIS_FX3_DEVICE_PID 0x841A
#define DAVIS_FX3_REQUIRED_LOGIC_REVISION 16
#define DAVIS_FX3_REQUIRED_FIRMWARE_VERSION 4

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

struct davis_state {
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
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
		struct {
			atomic_bool autoTrainRunning;
			caerFilterDVSNoise noiseFilter;
		} pixelFilterAutoTrain;
	} dvs;
	struct {
		// APS specific fields
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
		bool flipX;
		bool flipY;
		bool ignoreEvents;
		bool globalShutter;
		uint16_t currentReadoutType;
		uint16_t countX[APS_READOUT_TYPES_NUM];
		uint16_t countY[APS_READOUT_TYPES_NUM];
		struct {
			int32_t tsStartFrame;
			int32_t tsStartExposure;
			int32_t tsEndExposure;
			uint16_t *pixels;
#if APS_DEBUG_FRAME == 1
			uint16_t *resetPixels;
			uint16_t *signalPixels;
#endif
		} frame;
		struct {
			// Temporary values from device.
			uint16_t update;
			uint16_t tmpData;
			// Parameters for frame parsing.
			uint16_t positionX;
			uint16_t positionY;
			uint16_t sizeX;
			uint16_t sizeY;
		} roi;
		struct {
			uint8_t tmpData;
			uint32_t currentFrameExposure;
			uint32_t lastSetExposure;
			atomic_bool enabled;
			struct auto_exposure_state state;
		} autoExposure;
	} aps;
	struct {
		// IMU specific fields
		bool ignoreEvents;
		bool flipX;
		bool flipY;
		bool flipZ;
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
		// Frame Packet state
		caerFrameEventPacket frame;
		int32_t framePosition;
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
		uint16_t adcClock;
		uint16_t usbClock;
		uint16_t clockDeviationFactor;
		float logicClockActual;
		float adcClockActual;
		float usbClockActual;
	} deviceClocks;
	struct {
		// Debug transfer support (FX3 only).
		bool enabled;
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct davis_state *davisState;

struct davis_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_davis_info info;
	// State for data management, common to all DAVIS.
	struct davis_state state;
};

typedef struct davis_handle *davisHandle;

ssize_t davisFindAll(caerDeviceDiscoveryResult *discoveredDevices);
ssize_t davisFindFX2(caerDeviceDiscoveryResult *discoveredDevices);
ssize_t davisFindFX3(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle davisOpenAll(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
caerDeviceHandle davisOpenFX2(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
caerDeviceHandle davisOpenFX3(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);

bool davisClose(caerDeviceHandle cdh);

bool davisSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool davisConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool davisConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool davisDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool davisDataStop(caerDeviceHandle handle);
caerEventPacketContainer davisDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DAVIS_H_ */
