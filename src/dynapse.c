#include "dynapse.h"

static void dynapseEventTranslator(void *vdh, uint8_t *buffer, size_t bytesSent);
static int dynapseDataAcquisitionThread(void *inPtr);
static void dynapseDataAcquisitionThreadConfig(dynapseHandle handle);

static inline void checkStrictMonotonicTimestamp(dynapseHandle handle) {
	if (handle->state.currentTimestamp <= handle->state.lastTimestamp) {
		caerLog(CAER_LOG_ALERT, handle->info.deviceString,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void freeAllDataMemory(dynapseState state) {
	if (state->dataExchangeBuffer != NULL) {
		ringBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentSpikePacket != NULL) {
		free(&state->currentSpikePacket->packetHeader);
		state->currentSpikePacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DYNAPSE_SPIKE_EVENT_POS, NULL);
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

caerDeviceHandle dynapseOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict) {
	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DYNAPSE_DEVICE_NAME);

	dynapseHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DYNAPSE;

	dynapseState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	atomic_store_explicit(&state->dataExchangeBufferSize, 64, memory_order_relaxed);
	atomic_store_explicit(&state->dataExchangeBlocking, false, memory_order_relaxed);
	atomic_store_explicit(&state->dataExchangeStartProducers, true, memory_order_relaxed);
	atomic_store_explicit(&state->dataExchangeStopProducers, true, memory_order_relaxed);
	atomic_store_explicit(&state->usbBufferNumber, 8, memory_order_relaxed);
	atomic_store_explicit(&state->usbBufferSize, 8192, memory_order_relaxed);

	// Packet settings (size (in events) and time interval (in µs)).
	atomic_store_explicit(&state->maxPacketContainerPacketSize, 8192, memory_order_relaxed);
	atomic_store_explicit(&state->maxPacketContainerInterval, 10000, memory_order_relaxed);

	atomic_thread_fence(memory_order_release);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	snprintf(state->deviceThreadName, 15 + 1, "%s ID-%" PRIu16, DYNAPSE_DEVICE_NAME, deviceID);
	state->deviceThreadName[15] = '\0';

	// Search for device and open it.
	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	// libusb may create its own threads at this stage, so we temporarly set
	// a different thread name.
	char originalThreadName[15 + 1]; // +1 for terminating NUL character.
	thrd_get_name(originalThreadName, 15);
	originalThreadName[15] = '\0';

	thrd_set_name(state->deviceThreadName);
	int res = libusb_init(&state->usbState.deviceContext);

	thrd_set_name(originalThreadName);

	if (res != LIBUSB_SUCCESS) {
		free(handle);
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to initialize libusb context. Error: %d.", res);
		return (NULL);
	}

	// Try to open a Dynap-se device on a specific USB port.
	state->usbState.deviceHandle = usbDeviceOpen(state->usbState.deviceContext, USB_DEFAULT_DEVICE_VID,
		DYNAPSE_DEVICE_PID,
		DYNAPSE_DEVICE_DID_TYPE, busNumberRestrict, devAddressRestrict, serialNumberRestrict,
		DYNAPSE_REQUIRED_LOGIC_REVISION, DYNAPSE_REQUIRED_FIRMWARE_VERSION);
	if (state->usbState.deviceHandle == NULL) {
		libusb_exit(state->usbState.deviceContext);
		free(handle);

		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to open %s device.", DYNAPSE_DEVICE_NAME);
		return (NULL);
	}

	state->usbState.userData = handle;
	state->usbState.userCallback = &dynapseEventTranslator;

	struct usb_info usbInfo = usbGenerateInfo(state->usbState.deviceHandle, DYNAPSE_DEVICE_NAME, deviceID);
	if (usbInfo.deviceString == NULL) {
		usbDeviceClose(state->usbState.deviceHandle);
		libusb_exit(state->usbState.deviceContext);
		free(handle);

		return (NULL);
	}

	// Populate info variables based on data from device.
	uint32_t param32 = 0;

	handle->info.deviceID = I16T(deviceID);
	strncpy(handle->info.deviceSerialNumber, usbInfo.serialNumber, 8 + 1);
	handle->info.deviceUSBBusNumber = usbInfo.busNumber;
	handle->info.deviceUSBDeviceAddress = usbInfo.devAddress;
	handle->info.deviceString = usbInfo.deviceString;
	spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION,
		&param32);
	handle->info.logicVersion = I16T(param32);
	spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER,
		&param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK,
		&param32);
	handle->info.logicClock = I16T(param32);
	spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER,
		&param32);
	handle->info.chipID = I16T(param32);

	caerLog(CAER_LOG_DEBUG, usbInfo.deviceString,
		"Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".", usbInfo.busNumber,
		usbInfo.devAddress);

	return ((caerDeviceHandle) handle);
}

bool dynapseClose(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "Shutting down ...");

	// Finally, close the device fully.
	usbDeviceClose(state->usbState.deviceHandle);

	// Destroy libusb context.
	libusb_exit(state->usbState.deviceContext);

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	return (true);
}

struct caer_dynapse_info caerDynapseInfoGet(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_dynapse_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		struct caer_dynapse_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool dynapseSendDefaultConfig(caerDeviceHandle cdh) {
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL, false);

	dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs time-slices (defaults to 1ms)

	return (true);
}

bool dynapseConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
					atomic_store(&state->usbBufferNumber, param);

					// Notify data acquisition thread to change buffers.
					atomic_fetch_or(&state->dataAcquisitionThreadConfigUpdate, 1 << 0);
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					atomic_store(&state->usbBufferSize, param);

					// Notify data acquisition thread to change buffers.
					atomic_fetch_or(&state->dataAcquisitionThreadConfigUpdate, 1 << 0);
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

		case DYNAPSE_CONFIG_MUX:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_MUX_RUN:
				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN:
				case DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL:
					return (spiConfigSend(state->usbState.deviceHandle, DYNAPSE_CONFIG_MUX, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						uint8_t spiMultiConfig[6 + 6] = { 0 };

						spiMultiConfig[0] = DYNAPSE_CONFIG_MUX;
						spiMultiConfig[1] = DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[2] = 0x00;
						spiMultiConfig[3] = 0x00;
						spiMultiConfig[4] = 0x00;
						spiMultiConfig[5] = 0x01;

						spiMultiConfig[6] = DYNAPSE_CONFIG_MUX;
						spiMultiConfig[7] = DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[8] = 0x00;
						spiMultiConfig[9] = 0x00;
						spiMultiConfig[10] = 0x00;
						spiMultiConfig[11] = 0x00;

						return (libusb_control_transfer(state->usbState.deviceHandle,
							LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
							VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, 2, 0, spiMultiConfig, sizeof(spiMultiConfig), 0)
							== sizeof(spiMultiConfig));
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_AER:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_AER_RUN:
				case DYNAPSE_CONFIG_AER_ACK_DELAY:
				case DYNAPSE_CONFIG_AER_ACK_EXTENSION:
				case DYNAPSE_CONFIG_AER_WAIT_ON_TRANSFER_STALL:
				case DYNAPSE_CONFIG_AER_EXTERNAL_AER_CONTROL:
					return (spiConfigSend(state->usbState.deviceHandle, DYNAPSE_CONFIG_AER, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CHIP:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_CHIP_RUN:
				case DYNAPSE_CONFIG_CHIP_ID:
				case DYNAPSE_CONFIG_CHIP_CONTENT:
				case DYNAPSE_CONFIG_CHIP_REQ_DELAY:
				case DYNAPSE_CONFIG_CHIP_REQ_EXTENSION:
					return (spiConfigSend(state->usbState.deviceHandle, DYNAPSE_CONFIG_CHIP, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DYNAPSE_CONFIG_USB:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_USB_RUN:
				case DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigSend(state->usbState.deviceHandle, DYNAPSE_CONFIG_USB, paramAddr, param));
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

bool dynapseConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
					*param = U32T(atomic_load(&state->usbBufferNumber));
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					*param = U32T(atomic_load(&state->usbBufferSize));
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

		case DYNAPSE_CONFIG_MUX:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_MUX_RUN:
				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN:
				case DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL:
					return (spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_MUX, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_AER:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_AER_RUN:
				case DYNAPSE_CONFIG_AER_ACK_DELAY:
				case DYNAPSE_CONFIG_AER_ACK_EXTENSION:
				case DYNAPSE_CONFIG_AER_WAIT_ON_TRANSFER_STALL:
				case DYNAPSE_CONFIG_AER_EXTERNAL_AER_CONTROL:
					return (spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_AER, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CHIP:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_CHIP_RUN:
				case DYNAPSE_CONFIG_CHIP_ID:
				case DYNAPSE_CONFIG_CHIP_CONTENT:
				case DYNAPSE_CONFIG_CHIP_REQ_DELAY:
				case DYNAPSE_CONFIG_CHIP_REQ_EXTENSION:
					return (spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_CHIP, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SYSINFO:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION:
				case DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER:
				case DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER:
				case DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK:
					return (spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_USB:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_USB_RUN:
				case DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY:
					return (spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_USB, paramAddr, param));
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

bool dynapseDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	// Store new data available/not available anymore call-backs.
	state->dataNotifyIncrease = dataNotifyIncrease;
	state->dataNotifyDecrease = dataNotifyDecrease;
	state->dataNotifyUserPtr = dataNotifyUserPtr;
	state->dataShutdownNotify = dataShutdownNotify;
	state->dataShutdownUserPtr = dataShutdownUserPtr;

	// Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
	// will then set this correctly.
	state->currentPacketContainerCommitTimestamp = -1;

	// Initialize RingBuffer.
	state->dataExchangeBuffer = ringBufferInit(atomic_load(&state->dataExchangeBufferSize));
	if (state->dataExchangeBuffer == NULL) {
		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentSpikePacket = caerSpikeEventPacketAllocate(DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSpikePacket == NULL) {
		freeAllDataMemory(state);

		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate spike event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(DYNAPSE_SPECIAL_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate special event packet.");
		return (false);
	}

	if ((errno = thrd_create(&state->dataAcquisitionThread, &dynapseDataAcquisitionThread, handle)) != thrd_success) {
		freeAllDataMemory(state);

		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to start data acquisition thread. Error: %d.",
		errno);
		return (false);
	}

	// Wait for the data acquisition thread to be ready.
	while (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
		;
	}

	return (true);
}

bool dynapseDataStop(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	// Stop data acquisition thread.
	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false); // Ensure chip turns off.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, false); // Turn off timestamping too.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, false);
	}

	atomic_store(&state->dataAcquisitionThreadRun, false);

	// Wait for data acquisition thread to terminate...
	if ((errno = thrd_join(state->dataAcquisitionThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to join data acquisition thread. Error: %d.",
		errno);
		return (false);
	}

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
	state->currentSpikePacketPosition = 0;
	state->currentSpecialPacketPosition = 0;

	return (true);
}

caerEventPacketContainer dynapseDataGet(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;
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

#define TS_WRAP_ADD 0x8000

static inline int64_t generateFullTimestamp(int32_t tsOverflow, int32_t timestamp) {
	return (I64T((U64T(tsOverflow) << TS_OVERFLOW_SHIFT) | U64T(timestamp)));
}

static inline void initContainerCommitTimestamp(dynapseState state) {
	if (state->currentPacketContainerCommitTimestamp == -1) {
		state->currentPacketContainerCommitTimestamp = state->currentTimestamp
			+ I32T(atomic_load_explicit(&state->maxPacketContainerInterval, memory_order_relaxed)) - 1;
	}
}

static void dynapseEventTranslator(void *vhd, uint8_t *buffer, size_t bytesSent) {
	dynapseHandle handle = vhd;
	dynapseState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		caerLog(CAER_LOG_ALERT, handle->info.deviceString,
			"%zu bytes received via USB, which is not a multiple of two.", bytesSent);
		bytesSent &= (size_t) ~0x01;
	}

	for (size_t i = 0; i < bytesSent; i += 2) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentSpikePacket == NULL) {
			state->currentSpikePacket = caerSpikeEventPacketAllocate(
			DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpikePacket == NULL) {
				caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate spike event packet.");
				return;
			}
		}
		else if (state->currentSpikePacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpikePacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpikeEventPacket grownPacket = (caerSpikeEventPacket) caerGenericEventPacketGrow(
				(caerEventPacketHeader) state->currentSpikePacket, state->currentSpikePacketPosition * 2);
			if (grownPacket == NULL) {
				caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow spike event packet.");
				return;
			}

			state->currentSpikePacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(
			DYNAPSE_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to allocate special event packet.");
				return;
			}
		}
		else if (state->currentSpecialPacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpecialPacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerGenericEventPacketGrow(
				(caerEventPacketHeader) state->currentSpecialPacket, state->currentSpecialPacketPosition * 2);
			if (grownPacket == NULL) {
				caerLog(CAER_LOG_CRITICAL, handle->info.deviceString, "Failed to grow special event packet.");
				return;
			}

			state->currentSpecialPacket = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(*((uint16_t * ) (&buffer[i])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			// Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
			state->lastTimestamp = state->currentTimestamp;
			state->currentTimestamp = state->wrapAdd + (event & 0x7FFF);
			initContainerCommitTimestamp(state);

			// Check monotonicity of timestamps.
			checkStrictMonotonicTimestamp(handle);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							caerLog(CAER_LOG_ERROR, handle->info.deviceString, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							state->wrapOverflow = 0;
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;
							state->currentPacketContainerCommitTimestamp = -1;
							initContainerCommitTimestamp(state);

							caerLog(CAER_LOG_INFO, handle->info.deviceString, "Timestamp reset event received.");

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets. Done in main thread
							// to avoid deadlock inside callback.
							atomic_fetch_or(&state->dataAcquisitionThreadConfigUpdate, 1 << 1);

							break;
						}

						default:
							caerLog(CAER_LOG_ERROR, handle->info.deviceString,
								"Caught special event that can't be handled: %d.", data);
							break;
					}
					break;

				case 1: // AER addresses of Spikes.
				case 2: // Special encoding over 4 cases.
				case 5:
				case 6: {
					uint8_t sourceCoreID = 0; // code == 1

					if (code == 2) {
						sourceCoreID = 1;
					}
					else if (code == 5) {
						sourceCoreID = 2;
					}
					else if (code == 6) {
						sourceCoreID = 3;
					}

					uint8_t chipID = data & 0x0F;
					uint32_t neuronID = (data >> 4) & 0x00FF;

					caerSpikeEvent currentSpikeEvent = caerSpikeEventPacketGetEvent(state->currentSpikePacket,
						state->currentSpikePacketPosition);

					// Timestamp at event-stream insertion point.
					caerSpikeEventSetTimestamp(currentSpikeEvent, state->currentTimestamp);
					caerSpikeEventSetSourceCoreID(currentSpikeEvent, sourceCoreID);
					caerSpikeEventSetChipID(currentSpikeEvent, chipID);
					caerSpikeEventSetNeuronID(currentSpikeEvent, neuronID);
					caerSpikeEventValidate(currentSpikeEvent, state->currentSpikePacket);
					state->currentSpikePacketPosition++;

					break;
				}

				case 7: { // Timestamp wrap
					// Detect big timestamp wrap-around.
					int64_t wrapJump = (TS_WRAP_ADD * data);
					int64_t wrapSum = I64T(state->wrapAdd) + wrapJump;

					if (wrapSum > I64T(INT32_MAX)) {
						// Reset wrapAdd at this point, so we can again
						// start detecting overruns of the 32bit value.
						// We reset not to zero, but to the remaining value after
						// multiple wrap-jumps are taken into account.
						int64_t wrapRemainder = wrapSum - I64T(INT32_MAX) - 1LL;
						state->wrapAdd = I32T(wrapRemainder);

						state->lastTimestamp = 0;
						state->currentTimestamp = state->wrapAdd;

						// Increment TSOverflow counter.
						state->wrapOverflow++;

						caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
							state->currentSpecialPacket, state->currentSpecialPacketPosition);
						caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
						caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
						caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
						state->currentSpecialPacketPosition++;

						// Commit packets to separate before wrap from after cleanly.
						tsBigWrap = true;
					}
					else {
						// Each wrap is 2^15 µs (~32ms), and we have
						// to multiply it with the wrap counter,
						// which is located in the data part of this
						// event.
						state->wrapAdd = I32T(wrapSum);

						state->lastTimestamp = state->currentTimestamp;
						state->currentTimestamp = state->wrapAdd;
						initContainerCommitTimestamp(state);

						// Check monotonicity of timestamps.
						checkStrictMonotonicTimestamp(handle);

						caerLog(CAER_LOG_DEBUG, handle->info.deviceString,
							"Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					}

					break;
				}

				default:
					caerLog(CAER_LOG_ERROR, handle->info.deviceString, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// forceCommit is already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = I32T(
			atomic_load_explicit(&state->maxPacketContainerPacketSize, memory_order_relaxed));
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentSpikePacketPosition >= currentPacketContainerCommitSize)
				|| (state->currentSpecialPacketPosition >= currentPacketContainerCommitSize));

		bool containerTimeCommit = generateFullTimestamp(state->wrapOverflow, state->currentTimestamp)
			> state->currentPacketContainerCommitTimestamp;

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentSpikePacketPosition > 0) {
				caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DYNAPSE_SPIKE_EVENT_POS,
					(caerEventPacketHeader) state->currentSpikePacket);

				state->currentSpikePacket = NULL;
				state->currentSpikePacketPosition = 0;
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
					caerLog(CAER_LOG_INFO, handle->info.deviceString,
						"Dropped EventPacket Container because ring-buffer full!");

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
				caerEventPacketContainer tsResetContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
				if (tsResetContainer == NULL) {
					caerLog(CAER_LOG_CRITICAL, handle->info.deviceString,
						"Failed to allocate tsReset event packet container.");
					return;
				}

				// Allocate special packet just for this event.
				caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
					state->wrapOverflow);
				if (tsResetPacket == NULL) {
					caerLog(CAER_LOG_CRITICAL, handle->info.deviceString,
						"Failed to allocate tsReset special event packet.");
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
					if (!atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)) {
						return;
					}
				}

				// Signal new container as usual.
				if (state->dataNotifyIncrease != NULL) {
					state->dataNotifyIncrease(state->dataNotifyUserPtr);
				}
			}
		}
	}
}

static int dynapseDataAcquisitionThread(void *inPtr) {
	// inPtr is a pointer to device handle.
	dynapseHandle handle = inPtr;
	dynapseState state = &handle->state;

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "Initializing data acquisition thread ...");

	// Set thread name.
	thrd_set_name(state->deviceThreadName);

	// Reset configuration update, so as to not re-do work afterwards.
	atomic_store(&state->dataAcquisitionThreadConfigUpdate, 0);

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 2.
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, true);
		dynapseConfigSet((caerDeviceHandle) handle, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, true);
	}

	// Create buffers as specified in config file.
	usbAllocateTransfers(&state->usbState, U32T(atomic_load(&state->usbBufferNumber)),
		U32T(atomic_load(&state->usbBufferSize)), USB_DEFAULT_DATA_ENDPOINT);

	// Signal data thread ready back to start function.
	atomic_store(&state->dataAcquisitionThreadRun, true);

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "data acquisition thread ready to process events.");

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 1, .tv_usec = 0 };

	while (atomic_load_explicit(&state->dataAcquisitionThreadRun, memory_order_relaxed)
		&& state->usbState.activeDataTransfers > 0) {
		// Check config refresh, in this case to adjust buffer sizes.
		if (atomic_load_explicit(&state->dataAcquisitionThreadConfigUpdate, memory_order_relaxed) != 0) {
			dynapseDataAcquisitionThreadConfig(handle);
		}

		libusb_handle_events_timeout(state->usbState.deviceContext, &te);
	}

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "shutting down data acquisition thread ...");

	// Cancel all transfers and handle them.
	usbDeallocateTransfers(&state->usbState);

	// Ensure shutdown is stored and notified, could be because of all data transfers going away!
	atomic_store(&state->dataAcquisitionThreadRun, false);

	if (state->dataShutdownNotify != NULL) {
		state->dataShutdownNotify(state->dataShutdownUserPtr);
	}

	caerLog(CAER_LOG_DEBUG, handle->info.deviceString, "data acquisition thread shut down.");

	return (EXIT_SUCCESS);
}

static void dynapseDataAcquisitionThreadConfig(dynapseHandle handle) {
	dynapseState state = &handle->state;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uint32_t configUpdate = U32T(atomic_exchange(&state->dataAcquisitionThreadConfigUpdate, 0));

	if ((configUpdate >> 0) & 0x01) {
		// Do buffer size change: cancel all and recreate them.
		usbDeallocateTransfers(&state->usbState);
		usbAllocateTransfers(&state->usbState, U32T(atomic_load(&state->usbBufferNumber)),
			U32T(atomic_load(&state->usbBufferSize)), USB_DEFAULT_DATA_ENDPOINT);
	}

	if ((configUpdate >> 1) & 0x01) {
		// Get new Master/Slave information from device. Done here to prevent deadlock
		// inside asynchronous callback.
		uint32_t param32 = 0;

		spiConfigReceive(state->usbState.deviceHandle, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER,
			&param32);

		atomic_thread_fence(memory_order_seq_cst);
		handle->info.deviceIsMaster = param32;
		atomic_thread_fence(memory_order_seq_cst);
	}
}
