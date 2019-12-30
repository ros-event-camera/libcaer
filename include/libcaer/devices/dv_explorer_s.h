/**
 * @file dv_explorer_s.h
 *
 * DV_EXPLORER_S specific configuration defines and information structures.
 */

#ifndef LIBCAER_DEVICES_DV_EXPLORER_S_H_
#define LIBCAER_DEVICES_DV_EXPLORER_S_H_

#include "../events/polarity.h"
#include "../events/special.h"

#include "usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device type definition for iniVation DV_EXPLORER_S.
 */
#define CAER_DEVICE_DV_EXPLORER_S 9

/**
 * Samsung chip identifier.
 * 640x480, semi-synchronous readout.
 */
#define DV_EXPLORER_S_CHIP_ID 20

#define DVX_S_DVS 20
#define DVX_S_DVS_MODE 0
#define DVX_S_DVS_EVENT_FLATTEN 1
#define DVX_S_DVS_EVENT_ON_ONLY 2
#define DVX_S_DVS_EVENT_OFF_ONLY 3
#define DVX_S_DVS_SUBSAMPLE_ENABLE 4
#define DVX_S_DVS_AREA_BLOCKING_ENABLE 5
#define DVX_S_DVS_DUAL_BINNING_ENABLE 6
#define DVX_S_DVS_SUBSAMPLE_VERTICAL 7
#define DVX_S_DVS_SUBSAMPLE_HORIZONTAL 8
#define DVX_S_DVS_AREA_BLOCKING_0 9
#define DVX_S_DVS_AREA_BLOCKING_1 10
#define DVX_S_DVS_AREA_BLOCKING_2 11
#define DVX_S_DVS_AREA_BLOCKING_3 12
#define DVX_S_DVS_AREA_BLOCKING_4 13
#define DVX_S_DVS_AREA_BLOCKING_5 14
#define DVX_S_DVS_AREA_BLOCKING_6 15
#define DVX_S_DVS_AREA_BLOCKING_7 16
#define DVX_S_DVS_AREA_BLOCKING_8 17
#define DVX_S_DVS_AREA_BLOCKING_9 18
#define DVX_S_DVS_AREA_BLOCKING_10 19
#define DVX_S_DVS_AREA_BLOCKING_11 20
#define DVX_S_DVS_AREA_BLOCKING_12 21
#define DVX_S_DVS_AREA_BLOCKING_13 22
#define DVX_S_DVS_AREA_BLOCKING_14 23
#define DVX_S_DVS_AREA_BLOCKING_15 24
#define DVX_S_DVS_AREA_BLOCKING_16 25
#define DVX_S_DVS_AREA_BLOCKING_17 26
#define DVX_S_DVS_AREA_BLOCKING_18 27
#define DVX_S_DVS_AREA_BLOCKING_19 28
#define DVX_S_DVS_TIMESTAMP_RESET 29
#define DVX_S_DVS_GLOBAL_RESET_ENABLE 30
#define DVX_S_DVS_GLOBAL_RESET_DURING_READOUT 31
#define DVX_S_DVS_GLOBAL_HOLD_ENABLE 32
#define DVX_S_DVS_FIXED_READ_TIME_ENABLE 33
#define DVX_S_DVS_EXTERNAL_TRIGGER_MODE 34
#define DVX_S_DVS_TIMING_ED 35
#define DVX_S_DVS_TIMING_GH2GRS 36
#define DVX_S_DVS_TIMING_GRS 37
#define DVX_S_DVS_TIMING_GH2SEL 38
#define DVX_S_DVS_TIMING_SELW 39
#define DVX_S_DVS_TIMING_SEL2AY_R 40
#define DVX_S_DVS_TIMING_SEL2AY_F 41
#define DVX_S_DVS_TIMING_SEL2R_R 42
#define DVX_S_DVS_TIMING_SEL2R_F 43
#define DVX_S_DVS_TIMING_NEXT_SEL 44
#define DVX_S_DVS_TIMING_NEXT_GH 45
#define DVX_S_DVS_TIMING_READ_FIXED 46

#define DVX_S_DVS_MODE_OFF 0
#define DVX_S_DVS_MODE_MONITOR 1
#define DVX_S_DVS_MODE_STREAM 2

#define DVX_S_DVS_EXTERNAL_TRIGGER_MODE_TIMESTAMP_RESET 0
#define DVX_S_DVS_EXTERNAL_TRIGGER_MODE_SINGLE_FRAME 1

#define DVX_S_DVS_CROPPER 21
#define DVX_S_DVS_CROPPER_ENABLE 0
#define DVX_S_DVS_CROPPER_Y_START_ADDRESS 1
#define DVX_S_DVS_CROPPER_Y_END_ADDRESS 2
#define DVX_S_DVS_CROPPER_X_START_ADDRESS 3
#define DVX_S_DVS_CROPPER_X_END_ADDRESS 4

#define DVX_S_DVS_ACTIVITY_DECISION 22
#define DVX_S_DVS_ACTIVITY_DECISION_ENABLE 0
#define DVX_S_DVS_ACTIVITY_DECISION_POS_THRESHOLD 1
#define DVX_S_DVS_ACTIVITY_DECISION_NEG_THRESHOLD 2
#define DVX_S_DVS_ACTIVITY_DECISION_DEC_RATE 3
#define DVX_S_DVS_ACTIVITY_DECISION_DEC_TIME 4
#define DVX_S_DVS_ACTIVITY_DECISION_POS_MAX_COUNT 5

#define DVX_S_DVS_BIAS 23
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOG 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_SF 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_ON 2
#define DVX_S_DVS_BIAS_CURRENT_RANGE_nRST 3
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGA 4
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGD 5
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_SF 6
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_nOFF 7
#define DVX_S_DVS_BIAS_CURRENT_AMP 8
#define DVX_S_DVS_BIAS_CURRENT_ON 9
#define DVX_S_DVS_BIAS_CURRENT_OFF 10
#define DVX_S_DVS_BIAS_SIMPLE 20

#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOG_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOG_50uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_SF_0_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_SF_5uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_ON_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_ON_50uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_nRST_0_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_nRST_5uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGA_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGA_50uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGD_5uA 0
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGD_50uA 1
#define DVX_S_DVS_BIAS_CURRENT_RANGE_LOGD_500uA 2
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_SF_x0_1 0
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_SF_x1 1
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_nOFF_x0_1 0
#define DVX_S_DVS_BIAS_CURRENT_LEVEL_nOFF_x1 1

#define DVX_S_DVS_BIAS_SIMPLE_VERY_LOW 0
#define DVX_S_DVS_BIAS_SIMPLE_LOW 1
#define DVX_S_DVS_BIAS_SIMPLE_DEFAULT 2
#define DVX_S_DVS_BIAS_SIMPLE_HIGH 3
#define DVX_S_DVS_BIAS_SIMPLE_VERY_HIGH 4

/**
 * DV_EXPLORER_S device-related information.
 */
struct caer_dvx_s_info {
	/// Unique device identifier. Also 'source' for events.
	int16_t deviceID;
	/// Device serial number.
	char deviceSerialNumber[8 + 1];
	/// Device USB bus number.
	uint8_t deviceUSBBusNumber;
	/// Device USB device address.
	uint8_t deviceUSBDeviceAddress;
	/// Device information string, for logging purposes.
	/// If not NULL, pointed-to memory is *only* valid while the corresponding
	/// device is open! After calling deviceClose() this is invalid memory!
	char *deviceString;
	/// USB firmware version.
	int16_t firmwareVersion;
	/// Chip identifier/type.
	int16_t chipID;
	/// DVS X axis resolution.
	int16_t dvsSizeX;
	/// DVS Y axis resolution.
	int16_t dvsSizeY;
};

/**
 * Return basic information on the device, such as its ID, its
 * resolution, the logic version, and so on. See the 'struct
 * caer_dvx_s_info' documentation for more details.
 *
 * @param handle a valid device handle.
 *
 * @return a copy of the device information structure if successful,
 *         an empty structure (all zeros) on failure.
 */
struct caer_dvx_s_info caerDVExplorerSInfoGet(caerDeviceHandle handle);

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_DV_EXPLORER_S_H_ */
