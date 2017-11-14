#ifndef LIBCAER_SRC_DAVIS_RPI_H_
#define LIBCAER_SRC_DAVIS_RPI_H_

#include "devices/davis.h"
#include "data_exchange.h"
#include "container_generation.h"
#include "autoexposure.h"

#define APS_READOUT_TYPES_NUM 2
#define APS_READOUT_RESET  0
#define APS_READOUT_SIGNAL 1

/**
 * Enable APS frame debugging by only looking at the reset or signal
 * frames, and not at the resulting correlated frame.
 * Supported values:
 * 0 - normal output, no debug (default)
 * 1 - both reset and signal separately (marked as ROI regions
 *     [0,1,2,3] for signal and [4,5,6,7] for reset respectively)
 */
#define APS_DEBUG_FRAME 0

#define APS_ADC_DEPTH 10

#define APS_ADC_CHANNELS 1

#define APS_ROI_REGIONS DAVIS_APS_ROI_REGIONS_MAX

#define IMU6_COUNT 15

#define SPI_CONFIG_MSG_SIZE 6

#define DAVIS_RPI_EVENT_TYPES 4

#define DAVIS_RPI_POLARITY_DEFAULT_SIZE 4096
#define DAVIS_RPI_SPECIAL_DEFAULT_SIZE 128
#define DAVIS_RPI_FRAME_DEFAULT_SIZE 8
#define DAVIS_RPI_IMU_DEFAULT_SIZE 64

#define DAVIS_RPI_DEVICE_NAME "DAVISRPi"

#define DAVIS_RPI_REQUIRED_LOGIC_REVISION 9912

#define DAVIS_RPI_MAX_TRANSACTION_NUM 4096
#define DAVIS_RPI_MAX_WAIT_REQ_COUNT   100

/**
 * Support benchmarking the GPIO data exchange performance on RPi,
 * using the appropriate StreamTester logic (MachXO3_IoT).
 */
#define DAVIS_RPI_BENCHMARK 0

#define DAVIS_RPI_BENCHMARK_LIMIT_EVENTS (4 * 1000 * 1000)

enum benchmarkMode { ZEROS = 0, ONES = 1, COUNTER = 2, SWITCHING = 3, ALTERNATING = 4 };

// Alternative, simplified biasing support.
#define DAVIS_BIAS_ADDRESS_MAX 36
#define DAVIS_CHIP_REG_LENGTH 7

struct davis_rpi_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// Data transfer via GPIO.
	struct {
		volatile uint32_t *gpioReg;
		int spiFd;
		mtx_t spiLock;
		atomic_uint_fast32_t threadState;
		thrd_t thread;
		void (*shutdownCallback)(void *shutdownCallbackPtr);
		void *shutdownCallbackPtr;
	} gpio;
#if DAVIS_RPI_BENCHMARK == 1
	struct {
		enum benchmarkMode testMode;
		uint16_t expectedValue;
		size_t dataCount;
		size_t errorCount;
		struct timespec startTime;
	} benchmark;
#endif
	struct {
		uint8_t currentBiasArray[DAVIS_BIAS_ADDRESS_MAX + 1][2];
		uint8_t currentChipRegister[DAVIS_CHIP_REG_LENGTH];
	} biasing;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastY;
		bool gotY;
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
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
		bool resetRead;
		uint16_t currentReadoutType;
		uint16_t countX[APS_READOUT_TYPES_NUM];
		uint16_t countY[APS_READOUT_TYPES_NUM];
		uint16_t expectedCountX;
		uint16_t *expectedCountY;
		struct {
			int32_t tsStartFrame;
			int32_t tsStartExposure;
			int32_t tsEndExposure;
			size_t *pixelIndexes;
			size_t pixelIndexesPosition[APS_READOUT_TYPES_NUM];
			uint16_t *resetPixels;
			uint16_t *pixels;
		} frame;
		struct {
			// Temporary values from device.
			uint16_t update;
			uint16_t tmpData;
			bool deviceEnabled[APS_ROI_REGIONS];
			uint16_t startColumn[APS_ROI_REGIONS];
			uint16_t startRow[APS_ROI_REGIONS];
			uint16_t endColumn[APS_ROI_REGIONS];
			uint16_t endRow[APS_ROI_REGIONS];
			// Parameters for frame parsing.
			bool enabled[APS_ROI_REGIONS];
			uint16_t positionX[APS_ROI_REGIONS];
			uint16_t positionY[APS_ROI_REGIONS];
			uint16_t sizeX[APS_ROI_REGIONS];
			uint16_t sizeY[APS_ROI_REGIONS];
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
};

typedef struct davis_rpi_state *davisRPiState;

struct davis_rpi_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_davis_info info;
	// State for data management, for DAVIS IOT version.
	struct davis_rpi_state state;
};

typedef struct davis_rpi_handle *davisRPiHandle;

// busNumberRestrict, devAddressRestrict and serialNumberRestrict are ignored, only one device connected.
caerDeviceHandle davisRPiOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict);
bool davisRPiClose(caerDeviceHandle cdh);

bool davisRPiSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool davisRPiConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool davisRPiConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool davisRPiDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool davisRPiDataStop(caerDeviceHandle handle);
caerEventPacketContainer davisRPiDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DAVIS_RPI_H_ */
