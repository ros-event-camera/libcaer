#ifndef LIBCAER_DEVICES_FILTER_DVS_H_
#define LIBCAER_DEVICES_FILTER_DVS_H_

/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * enable DVS Region of Interest (ROI) filtering.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_ENABLE       0
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * start position on the X axis for Region of Interest.
 * Must be between 0 and DVS_SIZE_X-1, and be smaller
 * or equal to CAER_HOST_CONFIG_FILTER_DVS_ROI_END_COLUMN.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_START_COLUMN 1
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * start position on the Y axis for Region of Interest.
 * Must be between 0 and DVS_SIZE_Y-1, and be smaller
 * or equal to CAER_HOST_CONFIG_FILTER_DVS_ROI_END_ROW.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_START_ROW    2
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * end position on the X axis for Region of Interest.
 * Must be between 0 and DVS_SIZE_X-1, and be greater
 * or equal to CAER_HOST_CONFIG_FILTER_DVS_ROI_START_COLUMN.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_END_COLUMN   3
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * end position on the Y axis for Region of Interest.
 * Must be between 0 and DVS_SIZE_Y-1, and be greater
 * or equal to CAER_HOST_CONFIG_FILTER_DVS_ROI_START_ROW.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_END_ROW      4
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * number of events filtered out by the Region of Interest
 * (ROI) filter.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_ROI_STATISTICS   5

/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * enable full filtering for up to eight pixels.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_ENABLE     6
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 0, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_0_COLUMN   7
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 0, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_0_ROW      8
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 1, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_1_COLUMN   9
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 1, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_1_ROW      10
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 2, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_2_COLUMN   11
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 2, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_2_ROW      12
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 3, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_3_COLUMN   13
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 3, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_3_ROW      14
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 4, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_4_COLUMN   15
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 4, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_4_ROW      16
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 5, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_5_COLUMN   17
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 5, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_5_ROW      18
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 6, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_6_COLUMN   19
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 6, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_6_ROW      20
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 7, X axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_7_COLUMN   21
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * the pixel filter completely suppresses up to eight pixels in the
 * DVS array, filtering out all events produced by them.
 * This is the pixel 7, Y axis setting.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_7_ROW      22
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * number of events filtered out by the full pixel filter.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_PIXEL_STATISTICS 23

/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * enable the background-activity filter, which tries to remove events
 * caused by transistor leakage, by rejecting uncorrelated events.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE     24
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * specify the time difference constant for the background-activity
 * filter in microseconds. Events that do correlated within this
 * time-frame are let through, while others are filtered out.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_BACKGROUND_ACTIVITY_TIME       25
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * number of events filtered out by the background-activity filter.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_BACKGROUND_ACTIVITY_STATISTICS 26

/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * enable the refractory period filter, which limits the firing rate
 * of pixels.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_REFRACTORY_PERIOD_ENABLE     27
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * specify the time constant for the refractory period filter.
 * Pixels will be inhibited from generating new events during this
 * time after the last even has fired.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_REFRACTORY_PERIOD_TIME       28
/**
 * Parameter address for module CAER_HOST_CONFIG_FILTER:
 * number of events filtered out by the refractory period filter.
 */
#define CAER_HOST_CONFIG_FILTER_DVS_REFRACTORY_PERIOD_STATISTICS 29

#endif /* LIBCAER_DEVICES_FILTER_DVS_H_ */
