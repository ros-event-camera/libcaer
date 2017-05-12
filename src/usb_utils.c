#include "usb_utils.h"

struct usb_config_receive_struct {
	void (*userCallback)(void *userData, uint32_t param);
	void *userData;
};

typedef struct usb_config_receive_struct *usbConfigReceiveStruct;

void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer);
void LIBUSB_CALL usbConfigSendCallback(struct libusb_transfer *transfer);
void LIBUSB_CALL usbConfigReceiveCallback(struct libusb_transfer *transfer);

struct usb_info usbGenerateInfo(libusb_device_handle *devHandle, const char *deviceName, uint16_t deviceID) {
	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	uint8_t busNumber = libusb_get_bus_number(libusb_get_device(devHandle));
	uint8_t devAddress = libusb_get_device_address(libusb_get_device(devHandle));

	char serialNumber[8 + 1] = { 0 };
	int getStringDescResult = libusb_get_string_descriptor_ascii(devHandle, 3, (unsigned char *) serialNumber, 8 + 1);

	// Check serial number success and length.
	if (getStringDescResult < 0 || getStringDescResult > 8) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Unable to get serial number for %s device.", deviceName);

		struct usb_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]",
		deviceName, deviceID, serialNumber, busNumber, devAddress);

	char *fullLogString = malloc(fullLogStringLength + 1);
	if (fullLogString == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate memory for %s device info string.", deviceName);

		struct usb_info emptyInfo = { 0, .deviceString = NULL };
		return (emptyInfo);
	}

	snprintf(fullLogString, fullLogStringLength + 1, "%s ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]", deviceName,
		deviceID, serialNumber, busNumber, devAddress);

	struct usb_info usbInfo;
	usbInfo.busNumber = busNumber;
	usbInfo.devAddress = devAddress;
	strncpy(usbInfo.serialNumber, serialNumber, 8 + 1);
	usbInfo.deviceString = fullLogString;

	return (usbInfo);
}

bool spiConfigSend(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	uint8_t spiConfig[4] = { 0 };

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	return (libusb_control_transfer(devHandle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig), 0) == sizeof(spiConfig));
}

bool spiConfigSendAsync(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	struct libusb_transfer *controlTransfer = libusb_alloc_transfer(0);
	if (controlTransfer == NULL) {
		return (false);
	}

	// Create data buffer.
	uint8_t *controlTransferBuffer = calloc(1, (LIBUSB_CONTROL_SETUP_SIZE + sizeof(uint32_t)));
	if (controlTransferBuffer == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate buffer for libusb control transfer. Error: %d.",
		errno);

		libusb_free_transfer(controlTransfer);

		return (false);
	}

	// Initialize Transfer.
	libusb_fill_control_setup(controlTransferBuffer,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, sizeof(uint32_t));

	libusb_fill_control_transfer(controlTransfer, devHandle, controlTransferBuffer, &usbConfigSendCallback, NULL, 0);

	controlTransfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	// Put data in buffer.
	controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + 0] = U8T(param >> 24);
	controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + 1] = U8T(param >> 16);
	controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + 2] = U8T(param >> 8);
	controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE + 3] = U8T(param >> 0);

	if ((errno = libusb_submit_transfer(controlTransfer)) != LIBUSB_SUCCESS) {
		// The transfer buffer is freed automatically here thanks to
		// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
		libusb_free_transfer(controlTransfer);

		return (false);
	}

	return (true);
}

bool spiConfigReceive(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param) {
	uint8_t spiConfig[4] = { 0 };

	if (libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig), 0) != sizeof(spiConfig)) {
		return (false);
	}

	*param = 0;
	*param |= U32T(spiConfig[0] << 24);
	*param |= U32T(spiConfig[1] << 16);
	*param |= U32T(spiConfig[2] << 8);
	*param |= U32T(spiConfig[3] << 0);

	return (true);
}

bool spiConfigReceiveAsync(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr,
	void (*userCallback)(void *userData, uint32_t param), void *userData) {
	struct libusb_transfer *controlTransfer = libusb_alloc_transfer(0);
	if (controlTransfer == NULL) {
		return (false);
	}

	// Create data buffer.
	uint8_t *controlTransferBuffer = calloc(1,
		(LIBUSB_CONTROL_SETUP_SIZE + sizeof(uint32_t) + sizeof(struct usb_config_receive_struct)));
	if (controlTransferBuffer == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate buffer for libusb control transfer. Error: %d.",
		errno);

		libusb_free_transfer(controlTransfer);

		return (false);
	}

	// Put additional data in the unused part of the transfer buffer, this way
	// all memory is in one block and freed when the transfer is freed.
	usbConfigReceiveStruct configReceive = (usbConfigReceiveStruct) &controlTransferBuffer[LIBUSB_CONTROL_SETUP_SIZE
		+ sizeof(uint32_t)];

	configReceive->userCallback = userCallback;
	configReceive->userData = userData;

	// Initialize Transfer.
	libusb_fill_control_setup(controlTransferBuffer,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_REQUEST_FPGA_CONFIG, moduleAddr, paramAddr, sizeof(uint32_t));

	libusb_fill_control_transfer(controlTransfer, devHandle, controlTransferBuffer, &usbConfigReceiveCallback,
		configReceive, 0);

	controlTransfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	if ((errno = libusb_submit_transfer(controlTransfer)) != LIBUSB_SUCCESS) {
		// The transfer buffer is freed automatically here thanks to
		// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
		libusb_free_transfer(controlTransfer);

		return (false);
	}

	return (true);
}

libusb_device_handle *usbDeviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID, uint8_t busNumber,
	uint8_t devAddress, const char *serialNumber, int32_t requiredLogicRevision, int32_t requiredFirmwareVersion) {
	libusb_device_handle *devHandle = NULL;
	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(devContext, &devicesList);

	if (result >= 0) {
		// Cycle thorough all discovered devices and find a match.
		for (size_t i = 0; i < (size_t) result; i++) {
			struct libusb_device_descriptor devDesc;

			if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
				continue;
			}

			// Check if this is the device we want (VID/PID).
			if (devDesc.idVendor == devVID && devDesc.idProduct == devPID) {
				// Verify device firmware version.
				if (requiredFirmwareVersion >= 0 && U8T(devDesc.bcdDevice & 0x00FF) < U16T(requiredFirmwareVersion)) {
					caerLog(CAER_LOG_CRITICAL, __func__,
						"Device firmware version too old. You have version %" PRIu8 "; but at least version %" PRIu16 " is required. Please updated by following the Flashy upgrade documentation at 'http://inilabs.com/support/reflashing/'.",
						U8T(devDesc.bcdDevice & 0x00FF), U16T(requiredFirmwareVersion));

					continue;
				}

				// If a USB port restriction is given, honor it.
				if (busNumber > 0 && libusb_get_bus_number(devicesList[i]) != busNumber) {
					caerLog(CAER_LOG_INFO, __func__,
						"USB bus number restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8 ").",
						busNumber, libusb_get_bus_number(devicesList[i]));

					continue;
				}

				if (devAddress > 0 && libusb_get_device_address(devicesList[i]) != devAddress) {
					caerLog(CAER_LOG_INFO, __func__,
						"USB device address restriction is present (%" PRIu8 "), this device didn't match it (%" PRIu8 ").",
						devAddress, libusb_get_device_address(devicesList[i]));

					continue;
				}

				if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
					devHandle = NULL;

					continue;
				}

				// Check the serial number restriction, if any is present.
				if (serialNumber != NULL && !caerStrEquals(serialNumber, "")) {
					char deviceSerialNumber[8 + 1] = { 0 };
					int getStringDescResult = libusb_get_string_descriptor_ascii(devHandle, devDesc.iSerialNumber,
						(unsigned char *) deviceSerialNumber, 8 + 1);

					// Check serial number success and length.
					if (getStringDescResult < 0 || getStringDescResult > 8) {
						libusb_close(devHandle);
						devHandle = NULL;

						continue;
					}

					// Now check if the Serial Number matches.
					if (!caerStrEquals(serialNumber, deviceSerialNumber)) {
						libusb_close(devHandle);
						devHandle = NULL;

						caerLog(CAER_LOG_INFO, __func__,
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
					spiConfigReceive(devHandle, 6, 0, &param32);

					// Verify device logic version.
					if (param32 < U32T(requiredLogicRevision)) {
						libusb_release_interface(devHandle, 0);
						libusb_close(devHandle);
						devHandle = NULL;

						caerLog(CAER_LOG_CRITICAL, __func__,
							"Device logic revision too old. You have revision %" PRIu32 "; but at least revision %" PRIu32 " is required. Please updated by following the Flashy upgrade documentation at 'http://inilabs.com/support/reflashing/'.",
							param32, U32T(requiredLogicRevision));

						continue;
					}
				}

				// Found and configured it!
				break;
			}
		}

		libusb_free_device_list(devicesList, true);
	}

	return (devHandle);
}

void usbDeviceClose(libusb_device_handle *devHandle) {
	// Release interface 0 (default).
	libusb_release_interface(devHandle, 0);

	libusb_close(devHandle);
}

void usbAllocateTransfers(usbState state, uint32_t bufferNum, uint32_t bufferSize, uint8_t dataEndPoint) {
	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for %" PRIu32 " libusb transfers. Error: %d.",
			bufferNum, errno);
		return;
	}
	state->dataTransfersLength = bufferNum;

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate further libusb transfers (%zu of %" PRIu32 ").", i,
				bufferNum);
			continue;
		}

		// Create data buffer.
		state->dataTransfers[i]->length = (int) bufferSize;
		state->dataTransfers[i]->buffer = malloc(bufferSize);
		if (state->dataTransfers[i]->buffer == NULL) {
			caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate buffer for libusb transfer %zu. Error: %d.", i,
			errno);

			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		state->dataTransfers[i]->dev_handle = state->deviceHandle;
		state->dataTransfers[i]->endpoint = dataEndPoint;
		state->dataTransfers[i]->type = LIBUSB_TRANSFER_TYPE_BULK;
		state->dataTransfers[i]->callback = &usbDataTransferCallback;
		state->dataTransfers[i]->user_data = state;
		state->dataTransfers[i]->timeout = 0;
		state->dataTransfers[i]->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->dataTransfers[i])) == LIBUSB_SUCCESS) {
			state->activeDataTransfers++;
		}
		else {
			caerLog(CAER_LOG_CRITICAL, __func__, "Unable to submit libusb transfer %zu. Error: %s (%d).", i,
				libusb_strerror(errno), errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}
	}

	if (state->activeDataTransfers == 0) {
		// Didn't manage to allocate any USB transfers, free array memory and log failure.
		free(state->dataTransfers);
		state->dataTransfers = NULL;
		state->dataTransfersLength = 0;

		caerLog(CAER_LOG_CRITICAL, __func__, "Unable to allocate any libusb transfers.");
	}
}

void usbDeallocateTransfers(usbState state) {
	// Cancel all current transfers first.
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		if (state->dataTransfers[i] != NULL) {
			errno = libusb_cancel_transfer(state->dataTransfers[i]);
			if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
				caerLog(CAER_LOG_CRITICAL, __func__, "Unable to cancel libusb transfer %zu. Error: %s (%d).", i,
					libusb_strerror(errno), errno);
				// Proceed with trying to cancel all transfers regardless of errors.
			}
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (state->activeDataTransfers > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// The buffers and transfers have been deallocated in the callback.
	// Only the transfers array remains, which we free here.
	free(state->dataTransfers);
	state->dataTransfers = NULL;
	state->dataTransfersLength = 0;
}

void LIBUSB_CALL usbDataTransferCallback(struct libusb_transfer *transfer) {
	usbState state = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Handle data.
		(*state->userCallback)(state->userData, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter, free and exit.
	state->activeDataTransfers--;
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		// Remove from list, so we don't try to cancel it later on.
		if (state->dataTransfers[i] == transfer) {
			state->dataTransfers[i] = NULL;
		}
	}
	libusb_free_transfer(transfer);
}

void LIBUSB_CALL usbConfigSendCallback(struct libusb_transfer *transfer) {
	// We just wanted to send something, that is done, so just free here
	// without caring about transfer status.
	libusb_free_transfer(transfer);
}

void LIBUSB_CALL usbConfigReceiveCallback(struct libusb_transfer *transfer) {
	usbConfigReceiveStruct configReceive = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		uint32_t param = 0;

		param |= U32T(transfer->buffer[LIBUSB_CONTROL_SETUP_SIZE + 0] << 24);
		param |= U32T(transfer->buffer[LIBUSB_CONTROL_SETUP_SIZE + 1] << 16);
		param |= U32T(transfer->buffer[LIBUSB_CONTROL_SETUP_SIZE + 2] << 8);
		param |= U32T(transfer->buffer[LIBUSB_CONTROL_SETUP_SIZE + 3] << 0);

		// Handle data.
		(*configReceive->userCallback)(configReceive->userData, param);
	}

	libusb_free_transfer(transfer);
}
