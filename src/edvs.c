#include "edvs.h"

static void edvsLog(enum caer_log_level logLevel, edvsHandle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static bool serialThreadStart(edvsHandle handle);
static void serialThreadStop(edvsHandle handle);
static int serialThreadRun(void *handlePtr);
static void edvsEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent);
static bool edvsSendBiases(edvsState state, int biasID);

static void edvsLog(enum caer_log_level logLevel, edvsHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static inline void checkMonotonicTimestamp(edvsHandle handle) {
	if (handle->state.currentTimestamp < handle->state.lastTimestamp) {
		edvsLog(CAER_LOG_ALERT, handle,
			"Timestamps: non monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void freeAllDataMemory(edvsState state) {
	if (state->dataExchangeBuffer != NULL) {
		ringBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPolarityPacket != NULL) {
		free(&state->currentPolarityPacket->packetHeader);
		state->currentPolarityPacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT, NULL);
		}
	}

	if (state->currentSpecialPacket != NULL) {
		free(&state->currentSpecialPacket->packetHeader);
		state->currentSpecialPacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT, NULL);
		}
	}

	if (state->currentPacketContainer != NULL) {
		caerEventPacketContainerFree(state->currentPacketContainer);
		state->currentPacketContainer = NULL;
	}
}

caerDeviceHandle edvsOpen(uint16_t deviceID, const char *serialPortName, uint32_t serialBaudRate) {
	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", EDVS_DEVICE_NAME);

	edvsHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_EDVS;

	edvsState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	atomic_store(&state->dataExchangeBufferSize, 64);
	atomic_store(&state->dataExchangeBlocking, false);
	atomic_store(&state->dataExchangeStartProducers, true);
	atomic_store(&state->dataExchangeStopProducers, true);

	// Packet settings (size (in events) and time interval (in µs)).
	atomic_store(&state->maxPacketContainerPacketSize, 4096);
	atomic_store(&state->maxPacketContainerInterval, 10000);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);

	// Set device string.
	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s ID-%" PRIu16, EDVS_DEVICE_NAME, deviceID);

	char *fullLogString = malloc(fullLogStringLength + 1);
	if (fullLogString == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device string.");
		free(handle);

		return (NULL);
	}

	snprintf(fullLogString, fullLogStringLength + 1, "%s ID-%" PRIu16, EDVS_DEVICE_NAME, deviceID);

	handle->info.deviceString = fullLogString;

	// Try to open an eDVS device on a specific serial port.
	enum sp_return retVal = sp_get_port_by_name(serialPortName, &state->serialState.serialPort);
	if (retVal != SP_OK) {
		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to open device.");
		free(handle->info.deviceString);
		free(handle);

		return (NULL);
	}

	// Open the serial port.
	retVal = sp_open(state->serialState.serialPort, SP_MODE_READ_WRITE);
	if (retVal != SP_OK) {
		edvsLog(CAER_LOG_ERROR, handle, "Failed to open serial port, error: %d.", retVal);
		sp_free_port(state->serialState.serialPort);
		free(handle->info.deviceString);
		free(handle);

		return (NULL);
	}

	// Setup serial port communication.
	atomic_store(&state->serialState.serialReadSize, 1024);

	sp_set_baudrate(state->serialState.serialPort, (int) serialBaudRate);
	sp_set_bits(state->serialState.serialPort, 8);
	sp_set_parity(state->serialState.serialPort, SP_PARITY_NONE);
	sp_set_stopbits(state->serialState.serialPort, 1);
	sp_set_rts(state->serialState.serialPort, SP_RTS_FLOW_CONTROL);
	sp_set_cts(state->serialState.serialPort, SP_CTS_FLOW_CONTROL);
	sp_set_dtr(state->serialState.serialPort, SP_DTR_OFF);
	sp_set_dsr(state->serialState.serialPort, SP_DSR_IGNORE);
	sp_set_xon_xoff(state->serialState.serialPort, SP_XONXOFF_DISABLED);
	sp_set_flowcontrol(state->serialState.serialPort, SP_FLOWCONTROL_RTSCTS);

	const char *cmdNoEcho = "!U0\n";
	if (sp_blocking_write(state->serialState.serialPort, cmdNoEcho, 4, 0) != 4) {
		edvsLog(CAER_LOG_ERROR, handle, "Failed to send echo disable command.");
		sp_close(state->serialState.serialPort);
		sp_free_port(state->serialState.serialPort);
		free(handle->info.deviceString);
		free(handle);

		return (NULL);
	}

	const char *cmdEventFormat = "!E2\n";
	if (sp_blocking_write(state->serialState.serialPort, cmdEventFormat, 4, 0) != 4) {
		edvsLog(CAER_LOG_ERROR, handle, "Failed to send event format command.");
		sp_close(state->serialState.serialPort);
		sp_free_port(state->serialState.serialPort);
		free(handle->info.deviceString);
		free(handle);

		return (NULL);
	}

	// TODO: do we need to read startup data, to flush the pipe?

	// Populate info variables based on data from device.
	handle->info.deviceID = I16T(deviceID);
	handle->info.deviceIsMaster = true;
	handle->info.dvsSizeX = EDVS_ARRAY_SIZE_X;
	handle->info.dvsSizeY = EDVS_ARRAY_SIZE_Y;

	edvsLog(CAER_LOG_DEBUG, handle, "Initialized device successfully on port '%s'.",
		sp_get_port_name(state->serialState.serialPort));

	return ((caerDeviceHandle) handle);
}

bool edvsClose(caerDeviceHandle cdh) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	edvsLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Close and free serial port.
	sp_close(state->serialState.serialPort);
	sp_free_port(state->serialState.serialPort);

	edvsLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	return (true);
}

struct caer_edvs_info caerEDVSInfoGet(caerDeviceHandle cdh) {
	edvsHandle handle = (edvsHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_edvs_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_EDVS) {
		struct caer_edvs_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool edvsSendDefaultConfig(caerDeviceHandle cdh) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	// Set all biases to default value. Based on DSV128 Fast biases.
	caerIntegerToByteArray(1992, state->biases[EDVS_CONFIG_BIAS_CAS], BIAS_LENGTH);
	caerIntegerToByteArray(1108364, state->biases[EDVS_CONFIG_BIAS_INJGND], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->biases[EDVS_CONFIG_BIAS_REQPD], BIAS_LENGTH);
	caerIntegerToByteArray(8159221, state->biases[EDVS_CONFIG_BIAS_PUX], BIAS_LENGTH);
	caerIntegerToByteArray(132, state->biases[EDVS_CONFIG_BIAS_DIFFOFF], BIAS_LENGTH);
	caerIntegerToByteArray(309590, state->biases[EDVS_CONFIG_BIAS_REQ], BIAS_LENGTH);
	caerIntegerToByteArray(969, state->biases[EDVS_CONFIG_BIAS_REFR], BIAS_LENGTH);
	caerIntegerToByteArray(16777215, state->biases[EDVS_CONFIG_BIAS_PUY], BIAS_LENGTH);
	caerIntegerToByteArray(209996, state->biases[EDVS_CONFIG_BIAS_DIFFON], BIAS_LENGTH);
	caerIntegerToByteArray(13125, state->biases[EDVS_CONFIG_BIAS_DIFF], BIAS_LENGTH);
	caerIntegerToByteArray(271, state->biases[EDVS_CONFIG_BIAS_FOLL], BIAS_LENGTH);
	caerIntegerToByteArray(217, state->biases[EDVS_CONFIG_BIAS_PR], BIAS_LENGTH);

	// Send ALL biases to device.
	return (edvsSendBiases(state, -1));
}

bool edvsConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_SERIAL:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_SERIAL_READ_SIZE:
					atomic_store(&state->serialState.serialReadSize, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
					atomic_store(&state->dataExchangeBufferSize, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
					atomic_store(&state->dataExchangeBlocking, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
					atomic_store(&state->dataExchangeStartProducers, param);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
					atomic_store(&state->dataExchangeStopProducers, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_PACKETS:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE:
					atomic_store(&state->maxPacketContainerPacketSize, param);
					break;

				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL:
					atomic_store(&state->maxPacketContainerInterval, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					atomic_store(&state->deviceLogLevel, U8T(param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case EDVS_CONFIG_DVS:
			switch (paramAddr) {
				case EDVS_CONFIG_DVS_RUN:
					if (param && !atomic_load(&state->dvsRunning)) {
						const char *cmdStartDVS = "E+\n";
						if (sp_blocking_write(state->serialState.serialPort, cmdStartDVS, 3, 0) != 3) {
							return (false);
						}

						atomic_store(&state->dvsRunning, true);
					}
					else if (!param && atomic_load(&state->dvsRunning)) {
						const char *cmdStopDVS = "E-\n";
						if (sp_blocking_write(state->serialState.serialPort, cmdStopDVS, 3, 0) != 3) {
							return (false);
						}

						atomic_store(&state->dvsRunning, false);
					}
					break;

				case EDVS_CONFIG_DVS_TIMESTAMP_RESET:
					if (param) {
						atomic_store(&state->dvsTSReset, true);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case EDVS_CONFIG_BIAS:
			switch (paramAddr) {
				case EDVS_CONFIG_BIAS_CAS:
				case EDVS_CONFIG_BIAS_INJGND:
				case EDVS_CONFIG_BIAS_PUX:
				case EDVS_CONFIG_BIAS_PUY:
				case EDVS_CONFIG_BIAS_REQPD:
				case EDVS_CONFIG_BIAS_REQ:
				case EDVS_CONFIG_BIAS_FOLL:
				case EDVS_CONFIG_BIAS_PR:
				case EDVS_CONFIG_BIAS_REFR:
				case EDVS_CONFIG_BIAS_DIFF:
				case EDVS_CONFIG_BIAS_DIFFON:
				case EDVS_CONFIG_BIAS_DIFFOFF:
					caerIntegerToByteArray(param, state->biases[paramAddr], BIAS_LENGTH);
					return (edvsSendBiases(state, paramAddr));
					break;

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool edvsConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_SERIAL:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_SERIAL_READ_SIZE:
					*param = U32T(atomic_load(&state->serialState.serialReadSize));
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE:
					*param = U32T(atomic_load(&state->dataExchangeBufferSize));
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING:
					*param = atomic_load(&state->dataExchangeBlocking);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS:
					*param = atomic_load(&state->dataExchangeStartProducers);
					break;

				case CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS:
					*param = atomic_load(&state->dataExchangeStopProducers);
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_PACKETS:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE:
					*param = U32T(atomic_load(&state->maxPacketContainerPacketSize));
					break;

				case CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL:
					*param = U32T(atomic_load(&state->maxPacketContainerInterval));
					break;

				default:
					return (false);
					break;
			}
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					*param = atomic_load(&state->deviceLogLevel);
					break;

				default:
					return (false);
					break;
			}
			break;

		case EDVS_CONFIG_DVS:
			switch (paramAddr) {
				case EDVS_CONFIG_DVS_RUN:
					*param = atomic_load(&state->dvsRunning);
					break;

				case EDVS_CONFIG_DVS_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				default:
					return (false);
					break;
			}
			break;

		case EDVS_CONFIG_BIAS:
			switch (paramAddr) {
				case EDVS_CONFIG_BIAS_CAS:
				case EDVS_CONFIG_BIAS_INJGND:
				case EDVS_CONFIG_BIAS_PUX:
				case EDVS_CONFIG_BIAS_PUY:
				case EDVS_CONFIG_BIAS_REQPD:
				case EDVS_CONFIG_BIAS_REQ:
				case EDVS_CONFIG_BIAS_FOLL:
				case EDVS_CONFIG_BIAS_PR:
				case EDVS_CONFIG_BIAS_REFR:
				case EDVS_CONFIG_BIAS_DIFF:
				case EDVS_CONFIG_BIAS_DIFFON:
				case EDVS_CONFIG_BIAS_DIFFOFF:
					*param = caerByteArrayToInteger(state->biases[paramAddr], BIAS_LENGTH);
					break;

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

static bool serialThreadStart(edvsHandle handle) {
	// Start serial communication thread.
	if ((errno = thrd_create(&handle->state.serialState.serialThread, &serialThreadRun, handle)) != thrd_success) {
		return (false);
	}

	// Wait for serial communication thread to be ready.
	while (!atomic_load_explicit(&handle->state.serialState.serialThreadRun, memory_order_relaxed)) {
		;
	}

	return (true);
}

static void serialThreadStop(edvsHandle handle) {
	// Shut down serial communication thread.
	atomic_store(&handle->state.serialState.serialThreadRun, false);

	// Wait for serial communication thread to terminate.
	if ((errno = thrd_join(handle->state.serialState.serialThread, NULL)) != thrd_success) {
		// This should never happen!
		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to join serial thread. Error: %d.", errno);
	}
}

static int serialThreadRun(void *handlePtr) {
	edvsHandle handle = handlePtr;
	edvsState state = &handle->state;

	edvsLog(CAER_LOG_DEBUG, handle, "Starting serial communication thread ...");

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char threadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	strncpy(threadName, handle->info.deviceString, MAX_THREAD_NAME_LENGTH);
	threadName[MAX_THREAD_NAME_LENGTH] = '\0';

	thrd_set_name(threadName);

	// Signal data thread ready back to start function.
	atomic_store(&state->serialState.serialThreadRun, true);

	edvsLog(CAER_LOG_DEBUG, handle, "Serial communication thread running.");

	// Handle serial port reading (10 ms timeout).
	while (atomic_load_explicit(&state->serialState.serialThreadRun, memory_order_relaxed)) {
		size_t readSize = atomic_load_explicit(&state->serialState.serialReadSize, memory_order_relaxed);

		uint8_t dataBuffer[readSize];
		int bytesRead = sp_blocking_read(state->serialState.serialPort, dataBuffer, readSize, 10);
		if (bytesRead < 0) {
			// ERROR: call exceptional shut-down callback and exit.
			if (state->serialState.serialShutdownCallback != NULL) {
				state->serialState.serialShutdownCallback(state->serialState.serialShutdownCallbackPtr);
			}
			break;
		}

		if (bytesRead > 0) {
			// Read something, process it and try again.
			edvsEventTranslator(handle, dataBuffer, (size_t) bytesRead);
		}
	}

	// Ensure threadRun is false on termination.
	atomic_store(&state->serialState.serialThreadRun, false);

	edvsLog(CAER_LOG_DEBUG, handle, "Serial communication thread shut down.");

	return (EXIT_SUCCESS);
}

bool edvsDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr), void (*dataNotifyDecrease)(void *ptr),
	void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr), void *dataShutdownUserPtr) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	// Store new data available/not available anymore call-backs.
	state->dataNotifyIncrease = dataNotifyIncrease;
	state->dataNotifyDecrease = dataNotifyDecrease;
	state->dataNotifyUserPtr = dataNotifyUserPtr;

	state->serialState.serialShutdownCallback = dataShutdownNotify;
	state->serialState.serialShutdownCallbackPtr = dataShutdownUserPtr;

	// Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
	// will then set this correctly.
	state->currentPacketContainerCommitTimestamp = -1;

	// Initialize RingBuffer.
	state->dataExchangeBuffer = ringBufferInit(atomic_load(&state->dataExchangeBufferSize));
	if (state->dataExchangeBuffer == NULL) {
		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(EDVS_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPolarityPacket = caerPolarityEventPacketAllocate(EDVS_POLARITY_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPolarityPacket == NULL) {
		freeAllDataMemory(state);

		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(EDVS_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	if (!serialThreadStart(handle)) {
		freeAllDataMemory(state);

		edvsLog(CAER_LOG_CRITICAL, handle, "Failed to start serial data transfers.");
		return (false);
	}

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 6.
		edvsConfigSet((caerDeviceHandle) handle, EDVS_CONFIG_DVS, EDVS_CONFIG_DVS_RUN, true);
	}

	return (true);
}

bool edvsDataStop(caerDeviceHandle cdh) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;

	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 6.
		edvsConfigSet((caerDeviceHandle) handle, EDVS_CONFIG_DVS, EDVS_CONFIG_DVS_RUN, false);
	}

	serialThreadStop(handle);

	// Empty ringbuffer.
	caerEventPacketContainer container;
	while ((container = ringBufferGet(state->dataExchangeBuffer)) != NULL) {
		// Notify data-not-available call-back.
		if (state->dataNotifyDecrease != NULL) {
			state->dataNotifyDecrease(state->dataNotifyUserPtr);
		}

		// Free container, which will free its subordinate packets too.
		caerEventPacketContainerFree(container);
	}

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPolarityPacketPosition = 0;
	state->currentSpecialPacketPosition = 0;

	return (true);
}

// Remember to properly free the returned memory after usage!
caerEventPacketContainer edvsDataGet(caerDeviceHandle cdh) {
	edvsHandle handle = (edvsHandle) cdh;
	edvsState state = &handle->state;
	caerEventPacketContainer container = NULL;

	retry: container = ringBufferGet(state->dataExchangeBuffer);

	if (container != NULL) {
		// Found an event container, return it and signal this piece of data
		// is no longer available for later acquisition.
		if (state->dataNotifyDecrease != NULL) {
			state->dataNotifyDecrease(state->dataNotifyUserPtr);
		}

		return (container);
	}

	// Didn't find any event container, either report this or retry, depending
	// on blocking setting.
	if (atomic_load_explicit(&state->dataExchangeBlocking, memory_order_relaxed)) {
		// Don't retry right away in a tight loop, back off and wait a little.
		// If no data is available, sleep for a millisecond to avoid wasting resources.
		struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 1000000 };
		if (thrd_sleep(&noDataSleep, NULL) == 0) {
			goto retry;
		}
	}

	// Nothing.
	return (NULL);
}

#define TS_WRAP_ADD 0x10000
#define HIGH_BIT_MASK 0x80
#define LOW_BITS_MASK 0x7F

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T((U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void initContainerCommitTimestamp(edvsState state) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = state->currentTimestamp
			+ I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
	}
}

static void edvsEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
	edvsHandle handle = vhd;
	edvsState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!atomic_load(&state->serialState.serialThreadRun)) {
		return;
	}

	size_t i = 0;
	while (i < bytesSent) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(EDVS_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentPolarityPacket == NULL) {
			state->currentPolarityPacket = caerPolarityEventPacketAllocate(EDVS_POLARITY_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentPolarityPacket == NULL) {
				edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}
		else if (state->currentPolarityPacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPolarityPacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerPolarityEventPacket grownPacket = (caerPolarityEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPolarityPacket, state->currentPolarityPacketPosition * 2);
			if (grownPacket == NULL) {
				edvsLog(CAER_LOG_CRITICAL, handle, "Failed to grow polarity event packet.");
				return;
			}

			state->currentPolarityPacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(EDVS_SPECIAL_DEFAULT_SIZE,
				I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}
		else if (state->currentSpecialPacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpecialPacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentSpecialPacket, state->currentSpecialPacketPosition * 2);
			if (grownPacket == NULL) {
				edvsLog(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentSpecialPacket = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		uint8_t yByte = buffer[i];

		if (yByte & HIGH_BIT_MASK) {
			edvsLog(CAER_LOG_WARNING, handle, "Data not aligned - flushing rest of buffer.");
		}
		else if ((i + 3) < bytesSent) {
			uint8_t xByte = buffer[i + 1];
			uint8_t ts1Byte = buffer[i + 2];
			uint8_t ts2Byte = buffer[i + 3];

			uint16_t shortTS = U16T((ts1Byte << 8) | ts2Byte);

			// Timestamp reset.
			if (atomic_load(&state->dvsTSReset)) {
				atomic_store(&state->dvsTSReset, false);

				state->wrapOverflow = 0;
				state->wrapAdd = 0;
				state->lastShortTimestamp = 0;
				state->lastTimestamp = 0;
				state->currentTimestamp = 0;
				state->currentPacketContainerCommitTimestamp = -1;
				initContainerCommitTimestamp(state);

				// Defer timestamp reset event to later, so we commit it
				// alone, in its own packet.
				// Commit packets when doing a reset to clearly separate them.
				tsReset = true;
			}
			// Timestamp wrapped.
			else if (shortTS < state->lastShortTimestamp) {
				// Detect big timestamp wrap-around.
				if (state->wrapAdd == (INT32_MAX - (TS_WRAP_ADD - 1))) {
					// Reset wrapAdd to zero at this point, so we can again
					// start detecting overruns of the 32bit value.
					state->wrapAdd = 0;

					state->lastTimestamp = 0;
					state->currentTimestamp = 0;

					// Increment TSOverflow counter.
					state->wrapOverflow++;

					caerSpecialEvent currentEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
						state->currentSpecialPacketPosition++);
					caerSpecialEventSetTimestamp(currentEvent, INT32_MAX);
					caerSpecialEventSetType(currentEvent, TIMESTAMP_WRAP);
					caerSpecialEventValidate(currentEvent, state->currentSpecialPacket);

					// Commit packets to separate before wrap from after cleanly.
					tsBigWrap = true; // TODO: is this possible here?
				}
				else {
					// timestamp bit 15 is one -> wrap: now we need to increment
					// the wrapAdd, uses only 14 bit timestamps. Each wrap is 2^14 µs (~16ms).
					state->wrapAdd += TS_WRAP_ADD;

					state->lastTimestamp = state->currentTimestamp;
					state->currentTimestamp = state->wrapAdd;
					initContainerCommitTimestamp(state);

					// Check monotonicity of timestamps.
					checkMonotonicTimestamp(handle);
				}
			}

			// Expand to 32 bits. (Tick is 1µs already.)
			state->lastShortTimestamp = shortTS;
			state->lastTimestamp = state->currentTimestamp;
			state->currentTimestamp = state->wrapAdd + shortTS;
			initContainerCommitTimestamp(state);

			// Check monotonicity of timestamps.
			checkMonotonicTimestamp(handle);

			uint8_t x = (xByte & LOW_BITS_MASK);
			uint8_t y = (yByte & LOW_BITS_MASK);
			bool polarity = (xByte & HIGH_BIT_MASK);

			// Check range conformity.
			if (x < EDVS_ARRAY_SIZE_X && y < EDVS_ARRAY_SIZE_Y) {
				caerPolarityEvent currentEvent = caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
					state->currentPolarityPacketPosition++);
				caerPolarityEventSetTimestamp(currentEvent, state->currentTimestamp);
				caerPolarityEventSetPolarity(currentEvent, polarity);
				caerPolarityEventSetY(currentEvent, y);
				caerPolarityEventSetX(currentEvent, x);
				caerPolarityEventValidate(currentEvent, state->currentPolarityPacket);
			}
			else {
				if (x >= EDVS_ARRAY_SIZE_X) {
					edvsLog(CAER_LOG_ALERT, handle, "X address out of range (0-%d): %" PRIu16 ".",
					EDVS_ARRAY_SIZE_X - 1, x);
				}
				if (y >= EDVS_ARRAY_SIZE_Y) {
					edvsLog(CAER_LOG_ALERT, handle, "Y address out of range (0-%d): %" PRIu16 ".",
					EDVS_ARRAY_SIZE_Y - 1, y);
				}
			}

			// Thresholds on which to trigger packet container commit.
			// forceCommit is already defined above.
			// Trigger if any of the global container-wide thresholds are met.
			int32_t currentPacketContainerCommitSize = I32T(
				atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
			bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
				&& ((state->currentPolarityPacketPosition >= currentPacketContainerCommitSize)
					|| (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize));

			bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
				> state->currentPacketContainerCommitTimestamp;

			// FIXME: with the current EDVS architecture, currentTimestamp always comes together
			// with an event, so the very first event that matches this threshold will be
			// also part of the committed packet container. This doesn't break any of the invariants.

			// Commit packet containers to the ring-buffer, so they can be processed by the
			// main-loop, when any of the required conditions are met.
			if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
				// One or more of the commit triggers are hit. Set the packet container up to contain
				// any non-empty packets. Empty packets are not forwarded to save memory.
				bool emptyContainerCommit = true;

				if (state->currentPolarityPacketPosition > 0) {
					caerEventPacketContainerSetEventPacket(state->currentPacketContainer, POLARITY_EVENT,
						(caerEventPacketHeader) state->currentPolarityPacket);

					state->currentPolarityPacket = NULL;
					state->currentPolarityPacketPosition = 0;
					emptyContainerCommit = false;
				}

				if (state->currentSpecialPacketPosition > 0) {
					caerEventPacketContainerSetEventPacket(state->currentPacketContainer, SPECIAL_EVENT,
						(caerEventPacketHeader) state->currentSpecialPacket);

					state->currentSpecialPacket = NULL;
					state->currentSpecialPacketPosition = 0;
					emptyContainerCommit = false;
				}

				// If the commit was triggered by a packet container limit being reached, we always
				// update the time related limit. The size related one is updated implicitly by size
				// being reset to zero after commit (new packets are empty).
				if (containerTimeCommit) {
					while (generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
						> state->currentPacketContainerCommitTimestamp) {
						state->currentPacketContainerCommitTimestamp += I32T(
							atomic_load_explicit( &state->maxPacketContainerInterval, memory_order_relaxed));
					}
				}

				// Filter out completely empty commits. This can happen when data is turned off,
				// but the timestamps are still going forward.
				if (emptyContainerCommit) {
					caerEventPacketContainerFree(state->currentPacketContainer);
					state->currentPacketContainer = NULL;
				}
				else {
					if (!ringBufferPut(state->dataExchangeBuffer, state->currentPacketContainer)) {
						// Failed to forward packet container, just drop it, it doesn't contain
						// any critical information anyway.
						edvsLog(CAER_LOG_NOTICE, handle, "Dropped EventPacket Container because ring-buffer full!");

						caerEventPacketContainerFree(state->currentPacketContainer);
						state->currentPacketContainer = NULL;
					}
					else {
						if (state->dataNotifyIncrease != NULL) {
							state->dataNotifyIncrease(state->dataNotifyUserPtr);
						}

						state->currentPacketContainer = NULL;
					}
				}

				// The only critical timestamp information to forward is the timestamp reset event.
				// The timestamp big-wrap can also (and should!) be detected by observing a packet's
				// tsOverflow value, not the special packet TIMESTAMP_WRAP event, which is only informative.
				// For the timestamp reset event (TIMESTAMP_RESET), we thus ensure that it is always
				// committed, and we send it alone, in its own packet container, to ensure it will always
				// be ordered after any other event packets in any processing or output stream.
				if (tsReset) {
					// Allocate packet container just for this event.
					caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(EDVS_EVENT_TYPES);
					if (tsResetContainer == NULL) {
						edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset event packet container.");
						return;
					}

					// Allocate special packet just for this event.
					caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1,
						I16T(handle->info.deviceID), state->wrapOverflow);
					if (tsResetPacket == NULL) {
						edvsLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset special event packet.");
						return;
					}

					// Create timestamp reset event.
					caerSpecialEvent tsResetEvent = caerSpecialEventPacketGetEvent(tsResetPacket, 0);
					caerSpecialEventSetTimestamp(tsResetEvent, INT32_MAX);
					caerSpecialEventSetType(tsResetEvent, TIMESTAMP_RESET);
					caerSpecialEventValidate(tsResetEvent, tsResetPacket);

					// Assign special packet to packet container.
					caerEventPacketContainerSetEventPacket(tsResetContainer, SPECIAL_EVENT,
						(caerEventPacketHeader) tsResetPacket);

					// Reset MUST be committed, always, else downstream data processing and
					// outputs get confused if they have no notification of timestamps
					// jumping back go zero.
					while (!ringBufferPut(state->dataExchangeBuffer, tsResetContainer)) {
						// Prevent dead-lock if shutdown is requested and nothing is consuming
						// data anymore, but the ring-buffer is full (and would thus never empty),
						// thus blocking the USB handling thread in this loop.
						if (!atomic_load(&state->serialState.serialThreadRun)) {
							return;
						}
					}

					// Signal new container as usual.
					if (state->dataNotifyIncrease != NULL) {
						state->dataNotifyIncrease(state->dataNotifyUserPtr);
					}
				}
			}

			i += 3;
		}

		i++;
	}
}

static bool edvsSendBiases(edvsState state, int biasID) {
	// Biases are already stored in an array with the same format as expected by
	// the device, we can thus send them directly.
	char cmdSetBias[128];
	size_t startBias = (size_t) biasID;
	size_t stopBias = startBias + 1;

	// With -1 as ID, we program all biases.
	if (biasID == -1) {
		startBias = 0;
		stopBias = BIAS_NUMBER;
	}

	for (size_t i = startBias; i < stopBias; i++) {
		snprintf(cmdSetBias, 128, "!B%zu=%" PRIu32 "\n", i, caerByteArrayToInteger(state->biases[i], BIAS_LENGTH));
		size_t cmdSetBiasLength = strlen(cmdSetBias);

		if (sp_blocking_write(state->serialState.serialPort, cmdSetBias, cmdSetBiasLength, 0)
			!= (int) cmdSetBiasLength) {
			return (false);
		}
	}

	// Flush biases to chip.
	const char *cmdFlushBiases = "!BF\n";
	if (sp_blocking_write(state->serialState.serialPort, cmdFlushBiases, 4, 0) != 4) {
		return (false);
	}

	return (true);
}
