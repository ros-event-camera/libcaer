#ifndef LIBCAER_SRC_TIMESTAMPS_H_
#define LIBCAER_SRC_TIMESTAMPS_H_

#include "libcaer.h"
#include "events/common.h"
#include <stdatomic.h>

static inline void commonLog(enum caer_log_level logLevel, const char *deviceString, uint8_t deviceLogLevel,
	const char *format, ...) ATTRIBUTE_FORMAT(4);

static inline void commonLog(enum caer_log_level logLevel, const char *deviceString, uint8_t deviceLogLevel,
	const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(), deviceLogLevel, logLevel,
		deviceString, format, argumentList);
	va_end(argumentList);
}

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T(U64T(U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void checkStrictMonotonicTimestamp(int32_t tsCurrent, int32_t tsLast, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	uint8_t deviceLogLevel = atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed);

	if (tsCurrent <= tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, deviceLogLevel,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

static inline void checkMonotonicTimestamp(int32_t tsCurrent, int32_t tsLast, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	uint8_t deviceLogLevel = atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed);

	if (tsCurrent < tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, deviceLogLevel,
			"Timestamps: non monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

#endif /* LIBCAER_SRC_TIMESTAMPS_H_ */
