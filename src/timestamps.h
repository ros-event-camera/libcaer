#ifndef LIBCAER_SRC_TIMESTAMPS_H_
#define LIBCAER_SRC_TIMESTAMPS_H_

#include "libcaer.h"
#include "events/common.h"
#include <stdatomic.h>

struct timestamps_state_new_logic {
	int32_t wrapOverflow;
	int32_t wrapAdd;
	int32_t last;
	int32_t current;
};

typedef struct timestamps_state_new_logic *timestampsStateNewLogic;

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
	if (tsCurrent <= tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

static inline void checkMonotonicTimestamp(int32_t tsCurrent, int32_t tsLast, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	if (tsCurrent < tsLast) {
		commonLog(CAER_LOG_ALERT, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamps: non monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			tsLast, tsCurrent, (tsLast - tsCurrent));
	}
}

static inline bool handleTimestampWrapNewLogic(timestampsStateNewLogic timestamps, uint16_t wrapData, uint32_t wrapAdd,
	const char *deviceString, atomic_uint_fast8_t *deviceLogLevelAtomic) {
	// Detect big timestamp wrap-around.
	int64_t wrapJump = (wrapAdd * wrapData);
	int64_t wrapSum = I64T(timestamps->wrapAdd) + wrapJump;

	if (wrapSum > I64T(INT32_MAX)) {
		// Reset wrapAdd at this point, so we can again
		// start detecting overruns of the 32bit value.
		// We reset not to zero, but to the remaining value after
		// multiple wrap-jumps are taken into account.
		int64_t wrapRemainder = wrapSum - I64T(INT32_MAX) - 1LL;
		timestamps->wrapAdd = I32T(wrapRemainder);

		timestamps->last = 0;
		timestamps->current = timestamps->wrapAdd;

		// Increment TSOverflow counter.
		timestamps->wrapOverflow++;

		// Commit packets to separate before wrap from after cleanly.
		return (true);
	}
	else {
		// Each wrap is 2^15 µs (~32ms), and we have
		// to multiply it with the wrap counter,
		// which is located in the data part of this
		// event.
		timestamps->wrapAdd = I32T(wrapSum);

		timestamps->last = timestamps->current;
		timestamps->current = timestamps->wrapAdd;

		// Check monotonicity of timestamps.
		checkStrictMonotonicTimestamp(timestamps->current, timestamps->last, deviceString, deviceLogLevelAtomic);

		commonLog(CAER_LOG_DEBUG, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
			"Timestamp wrap event received with multiplier of %" PRIu16 ".", wrapData);

		return (false);
	}
}

static inline void handleTimestampUpdateNewLogic(timestampsStateNewLogic timestamps, uint16_t tsData,
	const char *deviceString, atomic_uint_fast8_t *deviceLogLevelAtomic) {
	// Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
	timestamps->last = timestamps->current;
	timestamps->current = timestamps->wrapAdd + (tsData & 0x7FFF);

	// Check monotonicity of timestamps.
	checkStrictMonotonicTimestamp(timestamps->current, timestamps->last, deviceString, deviceLogLevelAtomic);
}

static inline void handleTimestampResetNewLogic(timestampsStateNewLogic timestamps, const char *deviceString,
	atomic_uint_fast8_t *deviceLogLevelAtomic) {
	timestamps->wrapOverflow = 0;
	timestamps->wrapAdd = 0;
	timestamps->last = 0;
	timestamps->current = 0;

	commonLog(CAER_LOG_INFO, deviceString, atomic_load_explicit(deviceLogLevelAtomic, memory_order_relaxed),
		"Timestamp reset event received.");
}

#endif /* LIBCAER_SRC_TIMESTAMPS_H_ */
