#ifndef LIBCAER_SRC_DAVIS_COMMON_H_
#define LIBCAER_SRC_DAVIS_COMMON_H_

#include "devices/davis.h"
#include "devices/device_discover.h"

#include "filters/dvs_noise.h"

#include "autoexposure.h"
#include "container_generation.h"
#include "data_exchange.h"

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

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET 0
#define APS_READOUT_SIGNAL 1

#define APS_ADC_DEPTH 10
#define APS_ADC_CHANNELS 1

#define DVS_HOTPIXEL_HW_MAX 8

#define IMU_TYPE_TEMP 0x01
#define IMU_TYPE_GYRO 0x02
#define IMU_TYPE_ACCEL 0x04
#define IMU_TOTAL_COUNT 14

#define DAVIS_EVENT_TYPES 4

#define DAVIS_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_FRAME_DEFAULT_SIZE 8
#define DAVIS_IMU_DEFAULT_SIZE 64

struct davis_common_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
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
		uint16_t expectedCountX;
		uint16_t expectedCountY;
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
			uint16_t tmpData;
			uint16_t update;
			// Parameters for frame parsing.
			uint16_t positionX;
			uint16_t positionY;
			uint16_t sizeX;
			uint16_t sizeY;
		} roi;
		struct {
			bool offsetDirection; // 0 is increasing, 1 is decreasing.
			int16_t offset;
		} cDavisSupport;
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
};

typedef struct davis_common_state *davisCommonState;

#endif /* LIBCAER_SRC_DAVIS_COMMON_H_ */
