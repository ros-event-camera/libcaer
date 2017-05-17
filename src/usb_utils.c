#include "usb_utils.h"

struct usb_control_struct {
	union {
		void (*controlOutCallback)(void *controlOutCallbackPtr, int status);
		void (*controlInCallback)(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize);
	};
	void *controlCallbackPtr;
};

typedef struct usb_control_struct *usbControl;

struct usb_data_completion_struct {
	atomic_uint_fast32_t completed;
	uint8_t *data;
	size_t dataSize;
};

typedef struct usb_data_completion_struct *usbDataCompletion;

struct usb_config_receive_struct {
	void (*configReceiveCallback)(void *configReceiveCallbackPtr, int status, uint32_t param);
	void *configReceiveCallbackPtr;
};

typedef struct usb_config_receive_struct *usbConfigReceive;

static void usbAllocateTransfers(usbState state);
static void usbDeallocateTransfers(usbState state);
static int usbThreadRun(void *usbStatePtr);

static void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer);
static bool usbControlTransferAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status),
	void (*controlInCallback)(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize),
	void *controlCallbackPtr);
static void LIBUSB_CALL usbControlOutCallback(struct libusb_transfer *transfer);
static void LIBUSB_CALL usbControlInCallback(struct libusb_transfer *transfer);
static void syncControlOutCallback(void *controlOutCallbackPtr, int status);
static void syncControlInCallback(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize);
static void spiConfigReceiveCallback(void *configReceiveCallbackPtr, int status, uint8_t *buffer, size_t bufferSize);

bool usbDeviceOpen(usbState state, uint16_t devVID, uint16_t devPID, uint8_t busNumber, uint8_t devAddress,
	const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion) {
	// Search for device and open it.
	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	// libusb may create its own threads at this stage, so we temporarily set
	// a different thread name.
	char originalThreadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	thrd_get_name(originalThreadName, MAX_THREAD_NAME_LENGTH);
	originalThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	thrd_set_name(state->usbThreadName);

	int res = libusb_init(&state->deviceContext);

	thrd_set_name(originalThreadName);

	if (res != LIBUSB_SUCCESS) {
		caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Failed to initialize libusb context. Error: %d.", res);
		return (false);
	}

	libusb_device_handle *devHandle = NULL;
	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(state->deviceContext, &devicesList);

	if (result >= 0) {
		// Cycle thorough all discovered devices and find a match.
		for (size_t i = 0; i < (size_t) result; i++) {
			struct libusb_device_descriptor devDesc;

			if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
				continue;
			}

			// Check if this is the device we want (VID/PID).
			if (devDesc.idVendor == devVID && devDesc.idProduct == devPID) {
				// If a USB port restriction is given, honor it first.
				if (busNumber > 0 && libusb_get_bus_number(devicesList[i]) != busNumber) {
					caerLog(CAER_LOG_ERROR, state->usbThreadName,
						"USB bus number restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8 ").",
						busNumber, libusb_get_bus_number(devicesList[i]));

					continue;
				}

				if (devAddress > 0 && libusb_get_device_address(devicesList[i]) != devAddress) {
					caerLog(CAER_LOG_ERROR, state->usbThreadName,
						"USB device address restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8 ").",
						devAddress, libusb_get_device_address(devicesList[i]));

					continue;
				}

				// Verify device firmware version (after port restriction).
				if (requiredFirmwareVersion >= 0 && U8T(devDesc.bcdDevice & 0x00FF) < U16T(requiredFirmwareVersion)) {
					caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
						"Device firmware version too old. You have version %" PRIu8 "; but at least version %" PRIu16 " is required. Please updated by following the Flashy upgrade documentation at 'http://inilabs.com/support/reflashing/'.",
						U8T(devDesc.bcdDevice & 0x00FF), U16T(requiredFirmwareVersion));

					continue;
				}

				if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
					devHandle = NULL;

					continue;
				}

				// Check the serial number restriction, if any is present.
				if (serialNumber != NULL && !caerStrEquals(serialNumber, "")) {
					char deviceSerialNumber[MAX_SERIAL_NUMBER_LENGTH + 1] = { 0 };
					int getStringDescResult = libusb_get_string_descriptor_ascii(devHandle, devDesc.iSerialNumber,
						(unsigned char *) deviceSerialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);

					// Check serial number success and length.
					if (getStringDescResult < 0 || getStringDescResult > MAX_SERIAL_NUMBER_LENGTH) {
						libusb_close(devHandle);
						devHandle = NULL;

						continue;
					}

					// Now check if the Serial Number matches.
					if (!caerStrEquals(serialNumber, deviceSerialNumber)) {
						libusb_close(devHandle);
						devHandle = NULL;

						caerLog(CAER_LOG_ERROR, state->usbThreadName,
							"USB serial number restriction is present (%s), this device didn't match it (%s).",
							serialNumber, deviceSerialNumber);

						continue;
					}
				}

				// Check that the active configuration is set to number 1. If not, do so.
				int activeConfiguration;
				if (libusb_get_configuration(devHandle, &activeConfiguration) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				if (activeConfiguration != 1) {
					if (libusb_set_configuration(devHandle, 1) != LIBUSB_SUCCESS) {
						libusb_close(devHandle);
						devHandle = NULL;

						continue;
					}
				}

				// Claim interface 0 (default).
				if (libusb_claim_interface(devHandle, 0) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				if (requiredLogicRevision >= 0) {
					// Communication with device open, get logic version information.
					uint32_t param32 = 0;

					// Get logic version from generic SYSINFO module.
					spiConfigReceive(state, 6, 0, &param32);

					// Verify device logic version.
					if (param32 < U32T(requiredLogicRevision)) {
						libusb_release_interface(devHandle, 0);
						libusb_close(devHandle);
						devHandle = NULL;

						caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
							"Device logic revision too old. You have revision %" PRIu32 "; but at least revision %" PRIu32 " is required. Please updated by following the Flashy upgrade documentation at 'http://inilabs.com/support/reflashing/'.",
							param32, U32T(requiredLogicRevision));

						continue;
					}
				}

				// Initialize transfers mutex.
				if (mtx_init(&state->dataTransfersLock, mtx_plain) != thrd_success) {
					libusb_release_interface(devHandle, 0);
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				break;
			}
		}

		libusb_free_device_list(devicesList, true);
	}

	// Found and configured it!
	if (devHandle != NULL) {
		state->deviceHandle = devHandle;
		return (true);
	}

	// Didn't find anything.
	libusb_exit(state->deviceContext);
	caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Failed to open device.");
	return (false);
}

void usbDeviceClose(usbState state) {
	mtx_destroy(&state->dataTransfersLock);

	// Release interface 0 (default).
	libusb_release_interface(state->deviceHandle, 0);

	libusb_close(state->deviceHandle);

	libusb_exit(state->deviceContext);
}

void usbSetThreadName(usbState state, const char *threadName) {
	strncpy(state->usbThreadName, threadName, MAX_THREAD_NAME_LENGTH);
	state->usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';
}

void usbSetDataCallback(usbState state,
	void (*usbDataCallback)(void *usbDataCallbackPtr, uint8_t *buffer, size_t bytesSent), void *usbDataCallbackPtr) {
	state->usbDataCallback = usbDataCallback;
	state->usbDataCallbackPtr = usbDataCallbackPtr;
}

void usbSetShutdownCallback(usbState state, void (*usbShutdownCallback)(void *usbShutdownCallbackPtr),
	void *usbShutdownCallbackPtr) {
	state->usbShutdownCallback = usbShutdownCallback;
	state->usbShutdownCallbackPtr = usbShutdownCallbackPtr;
}

void usbSetDataEndpoint(usbState state, uint8_t dataEndPoint) {
	state->dataEndPoint = dataEndPoint;
}

void usbSetTransfersNumber(usbState state, uint32_t transfersNumber) {
	atomic_store(&state->usbBufferNumber, transfersNumber);

	usbCancelTransfersAsync(state);
}

void usbSetTransfersSize(usbState state, uint32_t transfersSize) {
	atomic_store(&state->usbBufferSize, transfersSize);

	usbCancelTransfersAsync(state);
}

uint32_t usbGetTransfersNumber(usbState state) {
	return (U32T(atomic_load(&state->usbBufferNumber)));
}

uint32_t usbGetTransfersSize(usbState state) {
	return (U32T(atomic_load(&state->usbBufferSize)));
}

struct usb_info usbGenerateInfo(usbState state, const char *deviceName, uint16_t deviceID) {
	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	uint8_t busNumber = libusb_get_bus_number(libusb_get_device(state->deviceHandle));
	uint8_t devAddress = libusb_get_device_address(libusb_get_device(state->deviceHandle));

	char serialNumber[MAX_SERIAL_NUMBER_LENGTH + 1] = { 0 };
	int getStringDescResult = libusb_get_string_descriptor_ascii(state->deviceHandle, 3, (unsigned char *) serialNumber,
	MAX_SERIAL_NUMBER_LENGTH + 1);

	// Check serial number success and length.
	if (getStringDescResult < 0 || getStringDescResult > MAX_SERIAL_NUMBER_LENGTH) {
		caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Unable to get serial number for %s device.", deviceName);

		struct usb_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]",
		deviceName, deviceID, serialNumber, busNumber, devAddress);

	char *fullLogString = malloc(fullLogStringLength + 1);
	if (fullLogString == NULL) {
		caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Unable to allocate memory for %s device info string.",
			deviceName);

		struct usb_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	snprintf(fullLogString, fullLogStringLength + 1, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]", deviceName,
		deviceID, serialNumber, busNumber, devAddress);

	struct usb_info usbInfo;

	usbInfo.busNumber = busNumber;
	usbInfo.devAddress = devAddress;
	strncpy(usbInfo.serialNumber, serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);
	usbInfo.deviceString = fullLogString;

	return (usbInfo);
}

static void usbAllocateTransfers(usbState state) {
	// Lock mutex.
	mtx_lock(&state->dataTransfersLock);

	uint32_t bufferNum = usbGetTransfersNumber(state);
	uint32_t bufferSize = usbGetTransfersSize(state);

	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
			"Failed to allocate memory for %" PRIu32 " libusb transfers. Error: %d.", bufferNum, errno);
		mtx_unlock(&state->dataTransfersLock);
		return;
	}
	state->dataTransfersLength = bufferNum;

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
				"Unable to allocate further libusb transfers (%zu of %" PRIu32 ").", i, bufferNum);
			continue;
		}

		// Create data buffer.
		state->dataTransfers[i]->length = (int) bufferSize;
		state->dataTransfers[i]->buffer = malloc(bufferSize);
		if (state->dataTransfers[i]->buffer == NULL) {
			caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
				"Unable to allocate buffer for libusb transfer %zu. Error: %d.", i,
				errno);

			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		state->dataTransfers[i]->dev_handle = state->deviceHandle;
		state->dataTransfers[i]->endpoint = state->dataEndPoint;
		state->dataTransfers[i]->type = LIBUSB_TRANSFER_TYPE_BULK;
		state->dataTransfers[i]->callback = &usbDataTransferCallback;
		state->dataTransfers[i]->user_data = state;
		state->dataTransfers[i]->timeout = 0;
		state->dataTransfers[i]->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->dataTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_fetch_add(&state->activeDataTransfers, 1);
		}
		else {
			caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Unable to submit libusb transfer %zu. Error: %s (%d).", i,
				libusb_strerror(errno), errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}
	}

	if (atomic_load(&state->activeDataTransfers) == 0) {
		// Didn't manage to allocate any USB transfers, free array memory and log failure.
		free(state->dataTransfers);
		state->dataTransfers = NULL;
		state->dataTransfersLength = 0;

		caerLog(CAER_LOG_CRITICAL, state->usbThreadName, "Unable to allocate any libusb transfers.");
	}

	mtx_unlock(&state->dataTransfersLock);
}

void usbCancelTransfersAsync(usbState state) {
	mtx_lock(&state->dataTransfersLock);

	// Cancel all current transfers.
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		if (state->dataTransfers[i] != NULL) {
			errno = libusb_cancel_transfer(state->dataTransfers[i]);
			if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
				caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
					"Unable to cancel libusb transfer %zu. Error: %s (%d).", i, libusb_strerror(errno), errno);
				// Proceed with trying to cancel all transfers regardless of errors.
			}
		}
	}

	mtx_unlock(&state->dataTransfersLock);
}

// Handle events until no more transfers exist, then deallocate them all.
// Use this in conjunction with usbCancelTransfers() above.
static void usbDeallocateTransfers(usbState state) {
	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (atomic_load_explicit(&state->activeDataTransfers, memory_order_relaxed) > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// No more transfers in flight, deallocate them all here.
	mtx_lock(&state->dataTransfersLock);

	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		if (state->dataTransfers[i] != NULL) {
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;
		}
	}

	// And lastly free the transfers array.
	free(state->dataTransfers);
	state->dataTransfers = NULL;
	state->dataTransfersLength = 0;

	mtx_unlock(&state->dataTransfersLock);
}

static void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer) {
	usbState state = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Handle data.
		(*state->usbDataCallback)(state->usbDataCallbackPtr, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter and exiting.
	// Freeing the transfers is taken care of by usbDeallocateTransfers().
	atomic_fetch_sub(&state->activeDataTransfers, 1);

	// If we got here but were not cancelled, it means the device went away
	// or some other, unrecoverable error happened. So we make sure the
	// USB thread can be terminated correctly.
	if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
		atomic_store(&state->usbThreadRun, false);
	}
}

bool usbThreadStart(usbState state) {
	if ((errno = thrd_create(&state->usbThread, &usbThreadRun, state)) != thrd_success) {
		return (false);
	}

	// Wait for the data acquisition thread to be ready.
	while (!atomic_load_explicit(&state->usbThreadRun, memory_order_relaxed)) {
		;
	}

	return (true);
}

bool usbThreadStop(usbState state) {
	// Shut down USB thread.
	atomic_store(&state->usbThreadRun, false);
	usbCancelTransfersAsync(state);

	// Wait for data acquisition thread to terminate...
	if ((errno = thrd_join(state->usbThread, NULL)) != thrd_success) {
		// This should never happen!
		return (false);
	}

	return (true);
}

static int usbThreadRun(void *usbStatePtr) {
	usbState state = usbStatePtr;

	caerLog(CAER_LOG_DEBUG, state->usbThreadName, "Initializing USB thread ...");

	// Set thread name.
	thrd_set_name(state->usbThreadName);

	// Create buffers as specified by settings.
	usbAllocateTransfers(state);

	// Signal data thread ready back to start function.
	atomic_store(&state->usbThreadRun, true);

	caerLog(CAER_LOG_DEBUG, state->usbThreadName, "USB thread ready.");

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 1, .tv_usec = 0 };

	handleUSBEvents: while (atomic_load_explicit(&state->activeDataTransfers, memory_order_relaxed) > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// activeDataTransfers drops to zero in three cases:
	// - the device went away
	// - the thread was shutdown, which cancels all transfers
	// - the USB buffer number/size changed, which cancels all transfers
	// In the first two cases we want to exit, in the third we want to go back
	// to the loop and continue handling USB events. Both the device dying and
	// the thread being shutdown set usbThreadRun to false, so we can use it
	// to discriminate between the cases.
	if (usbThreadIsRunning(state)) {
		usbDeallocateTransfers(state);
		usbAllocateTransfers(state);

		goto handleUSBEvents;
	}

	caerLog(CAER_LOG_DEBUG, state->usbThreadName, "Shutting down USB thread ...");

	// Cleanup transfers.
	usbDeallocateTransfers(state);

	if (state->usbShutdownCallback != NULL) {
		state->usbShutdownCallback(state->usbShutdownCallbackPtr);
	}

	caerLog(CAER_LOG_DEBUG, state->usbThreadName, "USB thread shut down.");

	return (EXIT_SUCCESS);
}

static bool usbControlTransferAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status),
	void (*controlInCallback)(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize),
	void *controlCallbackPtr) {
	// Check inputs. If both or neither callbacks are present,
	// that's an error, since only IN or OUT is possible.
	if (controlOutCallback == NULL && controlInCallback == NULL) {
		return (false);
	}
	if (controlOutCallback != NULL && controlInCallback != NULL) {
		return (false);
	}

	// If doing IN, data must always be NULL, the callback will handle it.
	if (controlInCallback != NULL && data != NULL) {
		return (false);
	}

	// If doing OUT and data is NULL (no data), dataSize must be zero!
	if (controlOutCallback != NULL && data == NULL && dataSize != 0) {
		return (false);
	}

	struct libusb_transfer *controlTransfer = libusb_alloc_transfer(0);
	if (controlTransfer == NULL) {
		return (false);
	}

	// Create data buffer.
	uint8_t *controlTransferBuffer = calloc(1,
		(LIBUSB_CONTROL_SETUP_SIZE + dataSize + sizeof(struct usb_control_struct)));
	if (controlTransferBuffer == NULL) {
		caerLog(CAER_LOG_CRITICAL, state->usbThreadName,
			"Unable to allocate buffer for libusb control transfer. Error: %d.",
			errno);

		libusb_free_transfer(controlTransfer);

		return (false);
	}

	// Put additional data in the unused part of the transfer buffer, this way
	// all memory is in one block and freed when the transfer is freed.
	usbControl extraControlData = (usbControl) &controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + dataSize];

	if (controlOutCallback != NULL) {
		extraControlData->controlOutCallback = controlOutCallback;
	}
	if (controlInCallback != NULL) {
		extraControlData->controlInCallback = controlInCallback;
	}
	extraControlData->controlCallbackPtr = controlCallbackPtr;

	// Initialize Transfer.
	uint8_t direction = (controlOutCallback != NULL) ? (LIBUSB_ENDPOINT_OUT) : (LIBUSB_ENDPOINT_IN);
	libusb_fill_control_setup(controlTransferBuffer, direction | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		bRequest, wValue, wIndex, U16T(dataSize));

	libusb_transfer_cb_fn controlCallback =
		(controlOutCallback != NULL) ? (&usbControlOutCallback) : (&usbControlInCallback);
	libusb_fill_control_transfer(controlTransfer, state->deviceHandle, controlTransferBuffer, controlCallback,
		extraControlData, 0);

	controlTransfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	// Put data in buffer. Only for OUT transfers, 'data' is always NULL for IN transfers.
	if (data != NULL) {
		memcpy(controlTransferBuffer + LIBUSB_CONTROL_SETUP_SIZE, data, dataSize);
	}

	if ((errno = libusb_submit_transfer(controlTransfer)) != LIBUSB_SUCCESS) {
		// The transfer buffer is freed automatically here thanks to
		// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
		libusb_free_transfer(controlTransfer);

		return (false);
	}

	return (true);
}

bool usbControlTransferOutAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize, void (*controlOutCallback)(void *controlOutCallbackPtr, int status), void *controlOutCallbackPtr) {
	return (usbControlTransferAsync(state, bRequest, wValue, wIndex, data, dataSize, controlOutCallback, NULL,
		controlOutCallbackPtr));
}

bool usbControlTransferInAsync(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, size_t dataSize,
	void (*controlInCallback)(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize),
	void *controlInCallbackPtr) {
	return (usbControlTransferAsync(state, bRequest, wValue, wIndex, NULL, dataSize, NULL, controlInCallback,
		controlInCallbackPtr));
}

// Async USB control exists to avoid the problem described in
// https://sourceforge.net/p/libusb/mailman/message/34129129/
// where the config function after the loop is never entered.
static void LIBUSB_CALL usbControlOutCallback(struct libusb_transfer *transfer) {
	usbControl extraControlData = transfer->user_data;

	(*extraControlData->controlOutCallback)(extraControlData->controlCallbackPtr, transfer->status);

	libusb_free_transfer(transfer);
}

static void LIBUSB_CALL usbControlInCallback(struct libusb_transfer *transfer) {
	usbControl extraControlData = transfer->user_data;

	(*extraControlData->controlInCallback)(extraControlData->controlCallbackPtr, transfer->status,
		libusb_control_transfer_get_data(transfer), (size_t) transfer->actual_length);

	libusb_free_transfer(transfer);
}

// Implement synchronous API over asynchronous one: wait till callbacks are done.
// This is done again here because the libusb API is unsuitable as it does its
// own event handling, which we want to fully delegate to the USB thread.
bool usbControlTransferOut(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize) {
	atomic_uint_fast32_t completed = ATOMIC_VAR_INIT(0);

	bool retVal = usbControlTransferOutAsync(state, bRequest, wValue, wIndex, data, dataSize, &syncControlOutCallback,
		&completed);
	if (!retVal) {
		// Failed to send out async request.
		return (false);
	}

	// Request is out and will be handled at some point by USB thread, wait on that.
	struct timespec waitForCompletionSleep = { .tv_sec = 0, .tv_nsec = 100000 };

	while (!atomic_load(&completed)) {
		// Sleep for 100µs to avoid busy loop.
		thrd_sleep(&waitForCompletionSleep, NULL);
	}

	if (atomic_load(&completed) == 1) {
		// Success.
		return (true);
	}
	else {
		// Failure.
		return (false);
	}
}

bool usbControlTransferIn(usbState state, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data,
	size_t dataSize) {
	struct usb_data_completion_struct dataCompletion = { ATOMIC_VAR_INIT(0), data, dataSize };

	bool retVal = usbControlTransferInAsync(state, bRequest, wValue, wIndex, dataSize, &syncControlInCallback,
		&dataCompletion);
	if (!retVal) {
		// Failed to send out async request.
		return (false);
	}

	// Request is out and will be handled at some point by USB thread, wait on that.
	struct timespec waitForCompletionSleep = { .tv_sec = 0, .tv_nsec = 100000 };

	while (!atomic_load(&dataCompletion.completed)) {
		// Sleep for 100µs to avoid busy loop.
		thrd_sleep(&waitForCompletionSleep, NULL);
	}

	if (atomic_load(&dataCompletion.completed) == 1) {
		// Success.
		return (true);
	}
	else {
		// Failure.
		return (false);
	}
}

static void syncControlOutCallback(void *controlOutCallbackPtr, int status) {
	atomic_uint_fast32_t *completed = controlOutCallbackPtr;

	if (status == LIBUSB_TRANSFER_COMPLETED) {
		atomic_store(completed, 1);
	}
	else {
		atomic_store(completed, 2);
	}
}

static void syncControlInCallback(void *controlInCallbackPtr, int status, uint8_t *buffer, size_t bufferSize) {
	usbDataCompletion dataCompletion = controlInCallbackPtr;

	if (status == LIBUSB_TRANSFER_COMPLETED && bufferSize == dataCompletion->dataSize) {
		// Copy data to location given by user.
		memcpy(dataCompletion->data, buffer, dataCompletion->dataSize);

		atomic_store(&dataCompletion->completed, 1);
	}
	else {
		atomic_store(&dataCompletion->completed, 2);
	}
}

bool spiConfigSend(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	uint8_t spiConfig[4] = { 0 };

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	return (usbControlTransferOut(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig,
		sizeof(spiConfig)));
}

bool spiConfigSendAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param,
	void (*configSendCallback)(void *configSendCallbackPtr, int status), void *configSendCallbackPtr) {
	uint8_t spiConfig[4] = { 0 };

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	return (usbControlTransferOutAsync(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig,
		sizeof(spiConfig), configSendCallback, configSendCallbackPtr));
}

bool spiConfigReceive(usbState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param) {
	uint8_t spiConfig[4] = { 0 };

	if (!usbControlTransferIn(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig))) {
		return (false);
	}

	*param = 0;
	*param |= U32T(spiConfig[0] << 24);
	*param |= U32T(spiConfig[1] << 16);
	*param |= U32T(spiConfig[2] << 8);
	*param |= U32T(spiConfig[3] << 0);

	return (true);
}

bool spiConfigReceiveAsync(usbState state, uint8_t moduleAddr, uint8_t paramAddr,
	void (*configReceiveCallback)(void *configReceiveCallbackPtr, int status, uint32_t param),
	void *configReceiveCallbackPtr) {
	usbConfigReceive config = calloc(1, sizeof(*config));
	if (config == NULL) {
		return (false);
	}

	config->configReceiveCallback = configReceiveCallback;
	config->configReceiveCallbackPtr = configReceiveCallbackPtr;

	bool retVal = usbControlTransferInAsync(state, VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, sizeof(uint32_t),
		&spiConfigReceiveCallback, config);
	if (!retVal) {
		free(config);

		return (false);
	}

	return (true);
}

static void spiConfigReceiveCallback(void *configReceiveCallbackPtr, int status, uint8_t *buffer, size_t bufferSize) {
	usbConfigReceive config = configReceiveCallbackPtr;

	uint32_t param = 0;

	if (status == LIBUSB_TRANSFER_COMPLETED && bufferSize == sizeof(uint32_t)) {
		param |= U32T(buffer[0] << 24);
		param |= U32T(buffer[1] << 16);
		param |= U32T(buffer[2] << 8);
		param |= U32T(buffer[3] << 0);
	}

	(*config->configReceiveCallback)(config->configReceiveCallbackPtr, status, param);

	free(config);
}
