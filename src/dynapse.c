#include "dynapse.h"
#include <unistd.h>

#define CONFIG_PARAMETER_SIZE 6
#define CONFIG_PARAMETER_MAX 85

static void dynapseLog(enum caer_log_level logLevel, dynapseHandle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static bool sendUSBCommandVerifyMultiple(dynapseHandle handle, uint8_t *config, size_t configNum);
static void dynapseEventTranslator(void *vdh, uint8_t *buffer, size_t bytesSent);
static void setSilentBiases(caerDeviceHandle cdh, uint8_t chipId);
static void setLowPowerBiases(caerDeviceHandle cdh, uint8_t chipId);

uint32_t caerDynapseGenerateCamBits(uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId, uint8_t synapseType) {
	uint32_t camBits = 0;

	camBits |= U32T(synapseType & 0x03) << 28;
	camBits |= U32T(inputNeuronAddr & 0xFF) << 20;
	camBits |= U32T((inputNeuronAddr >> 8) & 0x03) << 18;
	camBits |= U32T(0x01 << 17);
	camBits |= U32T((neuronAddr >> 8) & 0x03) << 15;
	camBits |= U32T((neuronAddr >> 4) & 0x0F) << 11;
	camBits |= U32T(camId & 0x3F) << 5;
	camBits |= U32T(neuronAddr & 0x0F) << 0;

	return (camBits);
}

uint32_t caerDynapseGenerateSramBits(uint16_t neuronAddr, uint8_t sramId, uint8_t virtualCoreId, bool sx, uint8_t dx,
bool sy, uint8_t dy, uint8_t destinationCore) {
	uint32_t sramBits = 0;

	sramBits |= U32T(virtualCoreId & 0x03) << 28;
	sramBits |= U32T(sy & 0x01) << 27;
	sramBits |= U32T(dy & 0x03) << 25;
	sramBits |= U32T(sx & 0x01) << 24;
	sramBits |= U32T(dx & 0x03) << 22;
	sramBits |= U32T(destinationCore & 0x0F) << 18;
	sramBits |= U32T(0x01 << 17);
	sramBits |= U32T((neuronAddr >> 8) & 0x03) << 15;
	sramBits |= U32T(neuronAddr & 0xFF) << 7;
	sramBits |= U32T(sramId & 0x03) << 5;
	sramBits |= U32T(0x01 << 4);

	return (sramBits);
}

uint16_t caerDynapseCoreXYToNeuronId(uint8_t coreId, uint8_t columnX, uint8_t rowY) {
	uint16_t neuronId = 0;

	neuronId |= U16T(coreId & 0x03) << 8;
	neuronId |= U16T(rowY & 0x0F) << 4;
	neuronId |= U16T(columnX & 0x0F) << 0;

	return (neuronId);
}

uint16_t caerDynapseCoreAddrToNeuronId(uint8_t coreId, uint8_t neuronAddrCore) {
	return (caerDynapseCoreXYToNeuronId(coreId, ((neuronAddrCore >> 0) & 0x0F), ((neuronAddrCore >> 4) & 0x0F)));
}

uint16_t caerDynapseSpikeEventGetX(caerSpikeEventConst event) {
	uint8_t chipId = caerSpikeEventGetChipID(event);
	uint8_t coreId = caerSpikeEventGetSourceCoreID(event);
	uint32_t neuronId = caerSpikeEventGetNeuronID(event);

	uint16_t columnId = (neuronId & 0x0F);
	bool addColumn = (coreId & 0x01);
	bool addColumnChip = ((chipId >> 2) & 0x02);
	columnId = U16T(columnId + (addColumn * DYNAPSE_CONFIG_NEUCOL) + (addColumnChip * DYNAPSE_CONFIG_XCHIPSIZE));

	return (columnId);
}

uint16_t caerDynapseSpikeEventGetY(caerSpikeEventConst event) {
	uint8_t chipId = caerSpikeEventGetChipID(event);
	uint8_t coreId = caerSpikeEventGetSourceCoreID(event);
	uint32_t neuronId = caerSpikeEventGetNeuronID(event);

	uint16_t rowId = ((neuronId >> 4) & 0x0F);
	bool addRow = (coreId & 0x02);
	bool addRowChip = ((chipId >> 2) & 0x01);
	rowId = U16T(rowId + (addRow * DYNAPSE_CONFIG_NEUROW) + (addRowChip * DYNAPSE_CONFIG_YCHIPSIZE));

	return (rowId);
}

struct caer_spike_event caerDynapseSpikeEventFromXY(uint16_t x, uint16_t y) {
	// Select chip. DYNAPSE_CONFIG_DYNAPSE_U0 default, doesn't need check.
	uint8_t chipId = DYNAPSE_CONFIG_DYNAPSE_U0;

	if (x >= DYNAPSE_CONFIG_XCHIPSIZE && y < DYNAPSE_CONFIG_YCHIPSIZE) {
		chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
		x -= DYNAPSE_CONFIG_XCHIPSIZE;
	}
	else if (x < DYNAPSE_CONFIG_XCHIPSIZE && y >= DYNAPSE_CONFIG_YCHIPSIZE) {
		chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
		y -= DYNAPSE_CONFIG_YCHIPSIZE;
	}
	else if (x >= DYNAPSE_CONFIG_XCHIPSIZE && y >= DYNAPSE_CONFIG_YCHIPSIZE) {
		chipId = DYNAPSE_CONFIG_DYNAPSE_U3;
		x -= DYNAPSE_CONFIG_XCHIPSIZE;
		y -= DYNAPSE_CONFIG_YCHIPSIZE;
	}

	// Select core. Core ID 0 default, doesn't need check.
	uint8_t coreId = 0;

	if (x >= DYNAPSE_CONFIG_NEUCOL && y < DYNAPSE_CONFIG_NEUROW) {
		coreId = 1;
		x -= DYNAPSE_CONFIG_NEUCOL;
	}
	else if (x < DYNAPSE_CONFIG_NEUCOL && y >= DYNAPSE_CONFIG_NEUROW) {
		coreId = 2;
		y -= DYNAPSE_CONFIG_NEUROW;
	}
	else if (x >= DYNAPSE_CONFIG_NEUCOL && y >= DYNAPSE_CONFIG_NEUROW) {
		coreId = 3;
		x -= DYNAPSE_CONFIG_NEUCOL;
		y -= DYNAPSE_CONFIG_NEUROW;
	}

	// Per-core neuron ID.
	uint32_t neuronId = (U32T(y) * DYNAPSE_CONFIG_NEUCOL) + U32T(x);

	// Output calculated values.
	struct caer_spike_event out;

	caerSpikeEventSetChipID(&out, chipId);
	caerSpikeEventSetSourceCoreID(&out, coreId);
	caerSpikeEventSetNeuronID(&out, neuronId);
	caerSpikeEventSetTimestamp(&out, 0);

	return (out);
}

static void dynapseLog(enum caer_log_level logLevel, dynapseHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static bool sendUSBCommandVerifyMultiple(dynapseHandle handle, uint8_t *config, size_t configNum) {
	dynapseState state = &handle->state;

	if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, U16T(configNum), 0, config,
		configNum * CONFIG_PARAMETER_SIZE)) {
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to send chip config, USB transfer failed.");
		return (false);
	}

	uint8_t check[2] = { 0 };
	bool result = usbControlTransferIn(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE, 0, 0, check,
		sizeof(check));
	if (!result || check[0] != VENDOR_REQUEST_FPGA_CONFIG_AER_MULTIPLE || check[1] != 0) {
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to send chip config, USB transfer failed on verification.");
		return (false);
	}

	return (true);
}

static inline void checkStrictMonotonicTimestamp(dynapseHandle handle) {
	if (handle->state.currentTimestamp <= handle->state.lastTimestamp) {
		dynapseLog(CAER_LOG_ALERT, handle,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIi32 ", currentTimestamp=%" PRIi32 ", difference=%" PRIi32 ".",
			handle->state.lastTimestamp, handle->state.currentTimestamp,
			(handle->state.lastTimestamp - handle->state.currentTimestamp));
	}
}

static inline void freeAllDataMemory(dynapseState state) {
	if (state->dataExchangeBuffer != NULL) {
		caerRingBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentSpikePacket != NULL) {
		free(&state->currentSpikePacket->packetHeader);
		state->currentSpikePacket = NULL;

		if (state->currentPacketContainer != NULL) {
			caerEventPacketContainerSetEventPacket(state->currentPacketContainer, DYNAPSE_SPIKE_EVENT_POS,
			NULL);
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
	atomic_store(&state->dataExchangeBufferSize, 64);
	atomic_store(&state->dataExchangeBlocking, false);
	atomic_store(&state->dataExchangeStartProducers, true);
	atomic_store(&state->dataExchangeStopProducers, true);

	// Packet settings (size (in events) and time interval (in µs)).
	atomic_store(&state->maxPacketContainerPacketSize, 8192);
	atomic_store(&state->maxPacketContainerInterval, 10000);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	atomic_store(&state->usbState.usbLogLevel, globalLogLevel);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s ID-%" PRIu16, DYNAPSE_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);
	handle->info.deviceString = usbThreadName; // Temporary, until replaced by full string.

	// Try to open a Dynap-se device on a specific USB port.
	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DYNAPSE_DEVICE_PID, busNumberRestrict,
		devAddressRestrict, serialNumberRestrict, DYNAPSE_REQUIRED_LOGIC_REVISION, DYNAPSE_REQUIRED_FIRMWARE_VERSION)) {
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to open device.");
		free(handle);

		return (NULL);
	}

	struct usb_info usbInfo = usbGenerateInfo(&state->usbState, DYNAPSE_DEVICE_NAME, deviceID);
	if (usbInfo.deviceString == NULL) {
		usbDeviceClose(&state->usbState);
		free(handle);

		return (NULL);
	}

	// Setup USB.
	usbSetDataCallback(&state->usbState, &dynapseEventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);

		free(usbInfo.deviceString);
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
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_VERSION, &param32);
	handle->info.logicVersion = I16T(param32);
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	handle->info.logicClock = I16T(param32);
	spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, DYNAPSE_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
	handle->info.chipID = I16T(param32);

	dynapseLog(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		usbInfo.busNumber, usbInfo.devAddress);

	return ((caerDeviceHandle) handle);
}

bool dynapseClose(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	dynapseLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	dynapseLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

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

static inline void setDynapseBias(caerDeviceHandle cdh, uint8_t biasAddress, uint8_t coarseValue, uint8_t fineValue,
bool biasHigh, bool typeNormal, bool sexN, bool enabled) {
	struct caer_bias_dynapse biasValue;

	biasValue.biasAddress = biasAddress;
	biasValue.coarseValue = coarseValue;
	biasValue.fineValue = fineValue;
	biasValue.enabled = enabled;
	biasValue.sexN = sexN;
	biasValue.typeNormal = typeNormal;
	biasValue.biasHigh = biasHigh;

	uint32_t biasBits = caerBiasDynapseGenerate(biasValue);

	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, biasBits);
}

static void setSilentBiases(caerDeviceHandle cdh, uint8_t chipId) {
	// Set chip ID for all subsequent bias updates.
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);

	// Core 0.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_DC_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_THR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_R2R_P, 7, 0, true, true, false, true);

	// Core 1.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_DC_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_THR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_R2R_P, 7, 0, true, true, false, true);

	// Core 2.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_DC_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_THR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_R2R_P, 7, 0, true, true, false, true);

	// Core 3.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_DC_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_THR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_R2R_P, 7, 0, true, true, false, true);

	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_BUFFER, 1, 2, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_SSP, 0, 7, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_SSN, 0, 15, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_BUFFER, 1, 2, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_SSP, 0, 7, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_SSN, 0, 15, true, true, false, true);
}

static void setLowPowerBiases(caerDeviceHandle cdh, uint8_t chipId) {
	// Set chip ID for all subsequent bias updates.
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);

	// Core 0.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P, 3, 80, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N, 3, 3, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_DC_P, 1, 30, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N, 7, 10, false, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N, 6, 100, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_THR_N, 3, 120, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N, 7, 35, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P, 3, 106, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C0_R2R_P, 4, 85, true, true, false, true);

	// Core 1.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P, 3, 80, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N, 3, 3, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_DC_P, 1, 30, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N, 7, 5, false, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N, 6, 100, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_THR_N, 4, 120, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N, 7, 35, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P, 3, 106, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C1_R2R_P, 4, 85, true, true, false, true);

	// Core 2.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P, 3, 80, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N, 3, 3, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_DC_P, 1, 30, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N, 7, 10, false, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N, 6, 100, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_THR_N, 4, 120, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N, 7, 35, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P, 3, 106, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C2_R2R_P, 4, 85, true, true, false, true);

	// Core 3.
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P, 3, 80, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N, 3, 3, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_DC_P, 1, 30, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N, 7, 5, false, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N, 6, 100, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_THR_N, 4, 120, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N, 7, 35, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P, 3, 106, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N, 7, 0, true, true, true, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P, 7, 40, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P, 7, 0, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_C3_R2R_P, 4, 85, true, true, false, true);

	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_BUFFER, 1, 2, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_SSP, 0, 7, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_D_SSN, 0, 15, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_BUFFER, 1, 2, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_SSP, 0, 7, true, true, false, true);
	setDynapseBias(cdh, DYNAPSE_CONFIG_BIAS_U_SSN, 0, 15, true, true, false, true);
}

bool dynapseSendDefaultConfig(caerDeviceHandle cdh) {
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL, false);

	// TODO: on next logic update, this will switch to be in cycles, not 125µs blocks.
	// So will need to multiply by: 125 * 30 (FX2_USB_CLOCK_FREQ).
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs time-slices (defaults to 1ms)

	// Turn on chip and AER communication for configuration.
	dynapseLog(CAER_LOG_NOTICE, (dynapseHandle) cdh, "Initializing device ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_REQ_DELAY, 30);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_REQ_EXTENSION, 30);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_RUN, true);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, true);

	// Set silent biases (no activity).
	setSilentBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U0);
	setSilentBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U1);
	setSilentBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U2);
	setSilentBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U3);

	// Clear all SRAM.
	dynapseLog(CAER_LOG_NOTICE, (dynapseHandle) cdh, "Clearing SRAM ...");
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Clearing SRAM U0 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY, 0, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Clearing SRAM U1 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY, 0, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Clearing SRAM U2 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY, 0, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Clearing SRAM U3 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY, 0, 0);

	// Set low power biases (some activity).
	setLowPowerBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U0);
	setLowPowerBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U1);
	setLowPowerBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U2);
	setLowPowerBiases(cdh, DYNAPSE_CONFIG_DYNAPSE_U3);

	// Setup SRAM for USB monitoring of spike events.
	dynapseLog(CAER_LOG_NOTICE, (dynapseHandle) cdh, "Programming default SRAM ...");
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Programming default SRAM U0 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U0, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Programming default SRAM U1 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U1, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Programming default SRAM U2 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U2, 0);
	dynapseLog(CAER_LOG_DEBUG, (dynapseHandle) cdh, "Programming default SRAM U3 ...");
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_DEFAULT_SRAM, DYNAPSE_CONFIG_DYNAPSE_U3, 0);

	// Turn off chip/AER once done.
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_RUN, false);
	dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, false);

	// Essential: wait for chip to be stable.
	sleep(1);

	dynapseLog(CAER_LOG_NOTICE, (dynapseHandle) cdh, "Device initialized.");

	return (true);
}

bool dynapseConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_USB_BUFFER_NUMBER:
					usbSetTransfersNumber(&state->usbState, param);
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					usbSetTransfersSize(&state->usbState, param);
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

					// Set USB log-level to this value too.
					atomic_store(&state->usbState.usbLogLevel, U8T(param));
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
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_MUX, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						uint8_t spiMultiConfig[2 * CONFIG_PARAMETER_SIZE] = { 0 };

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

						return (usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, 2, 0,
							spiMultiConfig, sizeof(spiMultiConfig)));
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
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_AER, paramAddr, param));
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
				case DYNAPSE_CONFIG_CHIP_REQ_DELAY:
				case DYNAPSE_CONFIG_CHIP_REQ_EXTENSION:
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_CHIP, paramAddr, param));
					break;

				case DYNAPSE_CONFIG_CHIP_CONTENT: {
					uint8_t chipConfig[CONFIG_PARAMETER_SIZE] = { 0 };

					chipConfig[0] = DYNAPSE_CONFIG_CHIP;
					chipConfig[1] = DYNAPSE_CONFIG_CHIP_CONTENT;
					chipConfig[2] = U8T(param >> 24);
					chipConfig[3] = U8T(param >> 16);
					chipConfig[4] = U8T(param >> 8);
					chipConfig[5] = U8T(param >> 0);

					// We use this function here instead of spiConfigSend() because
					// we also need to verify that the AER transaction succeeded!
					return (sendUSBCommandVerifyMultiple(handle, chipConfig, 1));
					break;
				}

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
					return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_USB, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_CLEAR_CAM: {
			uint32_t clearCamConfig[DYNAPSE_CONFIG_NUMNEURONS * DYNAPSE_CONFIG_NUMCAM];

			// Clear all CAMs on this chip.
			size_t idx = 0;
			for (uint16_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
				for (uint8_t camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId++) {
					clearCamConfig[idx++] = caerDynapseGenerateCamBits(0, neuronId, camId, 0);
				}
			}

			return (caerDynapseSendDataToUSB(cdh, clearCamConfig, idx));
			break;
		}

		case DYNAPSE_CONFIG_MONITOR_NEU: {
			if (paramAddr >= DYNAPSE_CONFIG_NUMCORES || param >= DYNAPSE_CONFIG_NUMNEURONS_CORE) {
				return (false);
			}

			uint8_t coreId = paramAddr;
			uint8_t neuronId = U8T(param);

			uint32_t neuronMonitorConfig[2] = { 0 };

			// Two commands: first reset core monitoring, then set neuron to monitor.
			neuronMonitorConfig[0] = 0x01 << 11 | U32T(coreId & 0x03) << 8;

			neuronMonitorConfig[1] = caerDynapseCoreAddrToNeuronId(coreId, neuronId);

			return (caerDynapseSendDataToUSB(cdh, neuronMonitorConfig, 2));
			break;
		}

		case DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY: {
			uint32_t sramEmptyConfig[DYNAPSE_CONFIG_NUMNEURONS * DYNAPSE_CONFIG_NUMSRAM_NEU];

			// SRAM empty routing has no different routings depending on chip, so 'paramAddr' is not used.
			size_t idx = 0;
			for (uint16_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
				for (uint8_t sramId = 0; sramId < DYNAPSE_CONFIG_NUMSRAM_NEU; sramId++) {
					sramEmptyConfig[idx++] = caerDynapseGenerateSramBits(neuronId, sramId, 0, 0, 0, 0, 0, 0);
				}
			}

			return (caerDynapseSendDataToUSB(cdh, sramEmptyConfig, idx));
			break;
		}

		case DYNAPSE_CONFIG_DEFAULT_SRAM: {
			bool sx = 0;
			uint8_t dx = 0;
			bool sy = 0;
			uint8_t dy = 0;
			uint8_t destinationCore = 0;

			// Route output neurons differently depending on the position of the chip in the board.
			// We want to route all spikes to the output south interface, and be able to tell from
			// which chip they came from. To do that, we set the destination core-id not to the
			// hot-coded format, but simply directly to the chip-id (0,4,8,12), with chip 0 actually
			// having an ID of 1 because the SRAM cannot have all zeros (or it disables routing).
			// This works because we got outside the chip system, to the FPGA, which simply gets
			// the four destination core-id bits and forwards them to the computer.
			switch (paramAddr) {
				case DYNAPSE_CONFIG_DYNAPSE_U0: {
					sx = 0;
					dx = 0;
					sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dy = 2;
					destinationCore = DYNAPSE_CONFIG_DYNAPSE_U0_OUT; // Chip-id for output.

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U1: {
					sx = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dx = 1;
					sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dy = 2;
					destinationCore = DYNAPSE_CONFIG_DYNAPSE_U1_OUT; // Chip-id for output.

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U2: {
					sx = 0;
					dx = 0;
					sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dy = 1;
					destinationCore = DYNAPSE_CONFIG_DYNAPSE_U2_OUT; // Chip-id for output.

					break;
				}

				case DYNAPSE_CONFIG_DYNAPSE_U3: {
					sx = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dx = 1;
					sy = DYNAPSE_CONFIG_SRAM_DIRECTION_NEG;
					dy = 1;
					destinationCore = DYNAPSE_CONFIG_DYNAPSE_U3_OUT; // Chip-id for output.

					break;
				}
			}

			uint32_t sramMonitorConfig[DYNAPSE_CONFIG_NUMNEURONS * DYNAPSE_CONFIG_NUMSRAM_NEU];

			size_t idx = 0;
			for (uint16_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
				for (uint8_t sramId = 0; sramId < DYNAPSE_CONFIG_NUMSRAM_NEU; sramId++) {
					// use first sram for monitoring
					if (sramId == 0) {
						uint8_t virtualCoreId = (neuronId >> 8) & 0x03;

						sramMonitorConfig[idx++] = caerDynapseGenerateSramBits(neuronId, sramId, virtualCoreId, sx, dx,
							sy, dy, destinationCore);
					}
					else {
						sramMonitorConfig[idx++] = caerDynapseGenerateSramBits(neuronId, sramId, 0, 0, 0, 0, 0, 0);
					}
				}
			}

			return (caerDynapseSendDataToUSB(cdh, sramMonitorConfig, idx));
			break;
		}

		case DYNAPSE_CONFIG_SRAM:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SYNAPSERECONFIG:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SYNAPSERECONFIG, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SPIKEGEN:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SPIKEGEN, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_POISSONSPIKEGEN:
			return (spiConfigSend(&state->usbState, DYNAPSE_CONFIG_POISSONSPIKEGEN, paramAddr, param));
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
					*param = usbGetTransfersNumber(&state->usbState);
					break;

				case CAER_HOST_CONFIG_USB_BUFFER_SIZE:
					*param = usbGetTransfersSize(&state->usbState);
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

		case DYNAPSE_CONFIG_MUX:
			switch (paramAddr) {
				case DYNAPSE_CONFIG_MUX_RUN:
				case DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN:
				case DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DYNAPSE_CONFIG_MUX_DROP_AER_ON_TRANSFER_STALL:
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_MUX, paramAddr, param));
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
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_AER, paramAddr, param));
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
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_CHIP, paramAddr, param));
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
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYSINFO, paramAddr, param));
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
					return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_USB, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DYNAPSE_CONFIG_SRAM:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SRAM, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SYNAPSERECONFIG:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SYNAPSERECONFIG, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_SPIKEGEN:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_SPIKEGEN, paramAddr, param));
			break;

		case DYNAPSE_CONFIG_POISSONSPIKEGEN:
			return (spiConfigReceive(&state->usbState, DYNAPSE_CONFIG_POISSONSPIKEGEN, paramAddr, param));
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

	usbSetShutdownCallback(&state->usbState, dataShutdownNotify, dataShutdownUserPtr);

	// Set wanted time interval to uninitialized. Getting the first TS or TS_RESET
	// will then set this correctly.
	state->currentPacketContainerCommitTimestamp = -1;

	// Initialize RingBuffer.
	state->dataExchangeBuffer = caerRingBufferInit(atomic_load(&state->dataExchangeBufferSize));
	if (state->dataExchangeBuffer == NULL) {
		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	state->currentPacketContainer = caerEventPacketContainerAllocate(DYNAPSE_EVENT_TYPES);
	if (state->currentPacketContainer == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentSpikePacket = caerSpikeEventPacketAllocate(DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentSpikePacket == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate spike event packet.");
		return (false);
	}

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(DYNAPSE_SPECIAL_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentSpecialPacket == NULL) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (atomic_load(&state->dataExchangeStartProducers)) {
		// Enable data transfer on USB end-point 2.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, true);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, true);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, true);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_RUN, true);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, true);
	}

	return (true);
}

bool dynapseDataStop(caerDeviceHandle cdh) {
	dynapseHandle handle = (dynapseHandle) cdh;
	dynapseState state = &handle->state;

	if (atomic_load(&state->dataExchangeStopProducers)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_AER, DYNAPSE_CONFIG_AER_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false); // Ensure chip turns off.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_TIMESTAMP_RUN, false); // Turn off timestamping too.
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_MUX, DYNAPSE_CONFIG_MUX_RUN, false);
		dynapseConfigSet(cdh, DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, false);
	}

	usbDataTransfersStop(&state->usbState);

	// Empty ringbuffer.
	caerEventPacketContainer container;
	while ((container = caerRingBufferGet(state->dataExchangeBuffer)) != NULL) {
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

	retry: container = caerRingBufferGet(state->dataExchangeBuffer);

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
	if (atomic_load_explicit(&state->dataExchangeBlocking, memory_order_relaxed)
		&& usbDataTransfersAreRunning(&state->usbState)) {
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
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		dynapseLog(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of two.", bytesSent);
		bytesSent &= (size_t) ~0x01;
	}

	for (size_t i = 0; i < bytesSent; i += 2) {
		// Allocate new packets for next iteration as needed.
		if (state->currentPacketContainer == NULL) {
			state->currentPacketContainer = caerEventPacketContainerAllocate(
			DYNAPSE_EVENT_TYPES);
			if (state->currentPacketContainer == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
				return;
			}
		}

		if (state->currentSpikePacket == NULL) {
			state->currentSpikePacket = caerSpikeEventPacketAllocate(
			DYNAPSE_SPIKE_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpikePacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate spike event packet.");
				return;
			}
		}
		else if (state->currentSpikePacketPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentSpikePacket)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpikeEventPacket grownPacket = (caerSpikeEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentSpikePacket, state->currentSpikePacketPosition * 2);
			if (grownPacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to grow spike event packet.");
				return;
			}

			state->currentSpikePacket = grownPacket;
		}

		if (state->currentSpecialPacket == NULL) {
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(
			DYNAPSE_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->wrapOverflow);
			if (state->currentSpecialPacket == NULL) {
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
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
				dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
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
							dynapseLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							state->wrapOverflow = 0;
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;
							state->currentPacketContainerCommitTimestamp = -1;
							initContainerCommitTimestamp(state);

							dynapseLog(CAER_LOG_INFO, handle, "Timestamp reset event received.");

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							break;
						}

						default:
							dynapseLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.", data);
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

					// On output via SRAM routing->FPGA->USB, the chip ID for
					// chip 0 is set to 1 so that it can work with the SRAM.
					// But this is then inconsistent with the chip IDs as used
					// everywhere else, so we reset this here for all users.
					if (chipID == DYNAPSE_CONFIG_DYNAPSE_U0_OUT) {
						chipID = DYNAPSE_CONFIG_DYNAPSE_U0;
					}

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

						dynapseLog(CAER_LOG_DEBUG, handle,
							"Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					}

					break;
				}

				default:
					dynapseLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
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
				if (!caerRingBufferPut(state->dataExchangeBuffer, state->currentPacketContainer)) {
					// Failed to forward packet container, just drop it, it doesn't contain
					// any critical information anyway.
					dynapseLog(CAER_LOG_NOTICE, handle, "Dropped EventPacket Container because ring-buffer full!");

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
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset event packet container.");
					return;
				}

				// Allocate special packet just for this event.
				caerSpecialEventPacket tsResetPacket = caerSpecialEventPacketAllocate(1, I16T(handle->info.deviceID),
					state->wrapOverflow);
				if (tsResetPacket == NULL) {
					dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to allocate tsReset special event packet.");
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
				while (!caerRingBufferPut(state->dataExchangeBuffer, tsResetContainer)) {
					// Prevent dead-lock if shutdown is requested and nothing is consuming
					// data anymore, but the ring-buffer is full (and would thus never empty),
					// thus blocking the USB handling thread in this loop.
					if (!usbDataTransfersAreRunning(&state->usbState)) {
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

bool caerDynapseSendDataToUSB(caerDeviceHandle cdh, const uint32_t *pointer, size_t numConfig) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	// Allocate memory for configuration parameters.
	uint8_t *spiMultiConfig = calloc(numConfig, CONFIG_PARAMETER_SIZE);
	if (spiMultiConfig == NULL) {
		return (false);
	}

	for (size_t i = 0; i < numConfig; i++) {
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 0] = DYNAPSE_CONFIG_CHIP;
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 1] = DYNAPSE_CONFIG_CHIP_CONTENT;
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 2] = U8T((pointer[i] >> 24) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 3] = U8T((pointer[i] >> 16) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 4] = U8T((pointer[i] >> 8) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 5] = U8T((pointer[i] >> 0) & 0x0FF);
	}

	size_t idxConfig = 0;

	while (numConfig > 0) {
		size_t configNum = (numConfig > CONFIG_PARAMETER_MAX) ? (CONFIG_PARAMETER_MAX) : (numConfig);
		size_t configSize = configNum * CONFIG_PARAMETER_SIZE;

		if (!sendUSBCommandVerifyMultiple(handle, spiMultiConfig + idxConfig, configNum)) {
			free(spiMultiConfig);
			return (false);
		}

		numConfig -= configNum;
		idxConfig += configSize;
	}

	free(spiMultiConfig);
	return (true);
}

bool caerDynapseWriteSramWords(caerDeviceHandle cdh, const uint16_t *data, uint32_t baseAddr, size_t numWords) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	dynapseState state = &handle->state;

	// Handle even and odd numbers of words to write.
	if ((numWords & 0x01) != 0) {
		// Handle the case where we have one trailing word
		// by just writing it manually.
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_WRITEDATA, data[numWords - 1]);
		spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_ADDRESS,
			baseAddr + (U32T(numWords) - 1));

		// Reduce numWords to the, now even, number of remaining words.
		// Otherwise the spiMultiConfig array filling loop will be incorrect!
		numWords--;
	}

	// Return if there was only one word to write, or none.
	if (numWords == 0) {
		return (true);
	}

	size_t numConfig = numWords / 2;

	// We need malloc because allocating dynamically sized arrays on the stack is not allowed.
	uint8_t *spiMultiConfig = calloc(numConfig, CONFIG_PARAMETER_SIZE);
	if (spiMultiConfig == NULL) {
		return (false);
	}

	for (size_t i = 0; i < numConfig; i++) {
		// Data word configuration.
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 0] = DYNAPSE_CONFIG_SRAM;
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 1] = DYNAPSE_CONFIG_SRAM_WRITEDATA;
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 2] = U8T((data[i * 2 + 1] >> 8) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 3] = U8T((data[i * 2 + 1] >> 0) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 4] = U8T((data[i * 2] >> 8) & 0x0FF);
		spiMultiConfig[(i * CONFIG_PARAMETER_SIZE) + 5] = U8T((data[i * 2] >> 0) & 0x0FF);
	}

	// Prepare the SRAM controller for writing.
	// First we write the base address by writing a spoof word to it (value zero).
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_RWCOMMAND, DYNAPSE_CONFIG_SRAM_WRITE);
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_WRITEDATA, 0);
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_ADDRESS, baseAddr);

	// Then we enable burst mode for faster writing.
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_BURSTMODE, 1);

	size_t idxConfig = 0;

	while (numConfig > 0) {
		size_t configNum = (numConfig > CONFIG_PARAMETER_MAX) ? (CONFIG_PARAMETER_MAX) : (numConfig);
		size_t configSize = configNum * CONFIG_PARAMETER_SIZE;

		if (!usbControlTransferOut(&state->usbState, VENDOR_REQUEST_FPGA_CONFIG_MULTIPLE, U16T(configNum), 0,
			spiMultiConfig + idxConfig, configSize)) {
			dynapseLog(CAER_LOG_CRITICAL, handle, "Failed to send SRAM burst data, USB transfer failed.");

			free(spiMultiConfig);
			return (false);
		}

		numConfig -= configNum;
		idxConfig += configSize;
	}

	// Disable burst mode again or things will go wrong when accessing the SRAM in the future.
	spiConfigSend(&state->usbState, DYNAPSE_CONFIG_SRAM, DYNAPSE_CONFIG_SRAM_BURSTMODE, 0);

	free(spiMultiConfig);
	return (true);
}

bool caerDynapseWriteCam(caerDeviceHandle cdh, uint16_t inputNeuronAddr, uint16_t neuronAddr, uint8_t camId,
	uint8_t synapseType) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	uint32_t camBits = caerDynapseGenerateCamBits(inputNeuronAddr, neuronAddr, camId, synapseType);

	return (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, camBits));
}

bool caerDynapseWriteSram(caerDeviceHandle cdh, uint8_t coreId, uint8_t neuronAddrCore, uint8_t virtualCoreId, bool sx,
	uint8_t dx, bool sy, uint8_t dy, uint8_t sramId, uint8_t destinationCore) {
	uint16_t neuronAddr = caerDynapseCoreAddrToNeuronId(coreId, neuronAddrCore);

	return (caerDynapseWriteSramN(cdh, neuronAddr, sramId, virtualCoreId, sx, dx, sy, dy, destinationCore));
}

bool caerDynapseWriteSramN(caerDeviceHandle cdh, uint16_t neuronAddr, uint8_t sramId, uint8_t virtualCoreId, bool sx,
	uint8_t dx, bool sy, uint8_t dy, uint8_t destinationCore) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	uint32_t sramBits = caerDynapseGenerateSramBits(neuronAddr, sramId, virtualCoreId, sx, dx, sy, dy, destinationCore);

	return (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, sramBits));
}

bool caerDynapseWritePoissonSpikeRate(caerDeviceHandle cdh, uint16_t neuronAddr, float rateHz) {
	dynapseHandle handle = (dynapseHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		return (false);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DYNAPSE) {
		return (false);
	}

	// Convert from Hz to device units with magic conversion constant for current Dynap-se hardware
	// (clock_rate/(wait_cycles*num_sources))/(UINT16_MAX-1) = size of frequency resolution steps
	uint16_t deviceRate = U16T(rateHz / 0.06706f);

	// Ready the data for programming (put it in data register).
	if (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_POISSONSPIKEGEN, DYNAPSE_CONFIG_POISSONSPIKEGEN_WRITEDATA,
		deviceRate) == false) {
		return (false);
	}

	// Trigger the write by writing the address register.
	if (caerDeviceConfigSet(cdh, DYNAPSE_CONFIG_POISSONSPIKEGEN, DYNAPSE_CONFIG_POISSONSPIKEGEN_WRITEADDRESS,
		neuronAddr) == false) {
		return (false);
	}

	// Everything's good!
	return (true);
}

static inline uint8_t coarseValueReverse(uint8_t coarseValue) {
	uint8_t coarseRev = 0;

	/*same as: sum(1 << (2 - i) for i in range(3) if 2 >> i & 1)*/
	if (coarseValue == 0) {
		coarseRev = 0;
	}
	else if (coarseValue == 1) {
		coarseRev = 4;
	}
	else if (coarseValue == 2) {
		coarseRev = 2;
	}
	else if (coarseValue == 3) {
		coarseRev = 6;
	}
	else if (coarseValue == 4) {
		coarseRev = 1;
	}
	else if (coarseValue == 5) {
		coarseRev = 5;
	}
	else if (coarseValue == 6) {
		coarseRev = 3;
	}
	else if (coarseValue == 7) {
		coarseRev = 7;
	}

	return (coarseRev);
}

uint32_t caerBiasDynapseGenerate(const struct caer_bias_dynapse dynapseBias) {
	// Build up bias value from all its components.
	uint32_t biasValue = U32T((dynapseBias.biasAddress & 0x7F) << 18) | U32T(0x01 << 16);

	// SSN and SSP are different.
	if (dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_U_SSP || dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_U_SSN
		|| dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_D_SSP || dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_D_SSN) {
		// Special (bit 15) is always enabled for Shifted-Source biases.
		// For all other bias types we keep it disabled, as it is not useful for users.
		biasValue |= U32T(0x3F << 10) | U32T((dynapseBias.fineValue & 0x3F) << 4);
	}
	// So are the Buffer biases.
	else if (dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_U_BUFFER
		|| dynapseBias.biasAddress == DYNAPSE_CONFIG_BIAS_D_BUFFER) {
		biasValue |= U32T(
			(coarseValueReverse(dynapseBias.coarseValue) & 0x07) << 12) | U32T((dynapseBias.fineValue & 0xFF) << 4);
	}
	// Standard coarse-fine biases.
	else {
		if (dynapseBias.enabled) {
			biasValue |= 0x01;
		}
		if (dynapseBias.sexN) {
			biasValue |= 0x02;
		}
		if (dynapseBias.typeNormal) {
			biasValue |= 0x04;
		}
		if (dynapseBias.biasHigh) {
			biasValue |= 0x08;
		}

		biasValue |= U32T(
			(coarseValueReverse(dynapseBias.coarseValue) & 0x07) << 12) | U32T((dynapseBias.fineValue & 0xFF) << 4);
	}

	return (biasValue);
}

struct caer_bias_dynapse caerBiasDynapseParse(const uint32_t dynapseBias) {
	struct caer_bias_dynapse biasValue = { 0, 0, 0, false, false, false, false };

	// Decompose bias integer into its parts.
	biasValue.biasAddress = (dynapseBias >> 18) & 0x7F;

	// SSN and SSP are different.
	if (biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_U_SSP || biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_U_SSN
		|| biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_D_SSP || biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_D_SSN) {
		// Special (bit 15) is always enabled for Shifted-Source biases.
		// For all other bias types we keep it disabled, as it is not useful for users.
		biasValue.fineValue = (dynapseBias >> 4) & 0x3F;
	}
	// So are the Buffer biases.
	else if (biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_U_BUFFER
		|| biasValue.biasAddress == DYNAPSE_CONFIG_BIAS_D_BUFFER) {
		biasValue.coarseValue = coarseValueReverse((dynapseBias >> 12) & 0x07);
		biasValue.fineValue = (dynapseBias >> 4) & 0xFF;
	}
	// Standard coarse-fine biases.
	else {
		biasValue.enabled = (dynapseBias & 0x01);
		biasValue.sexN = (dynapseBias & 0x02);
		biasValue.typeNormal = (dynapseBias & 0x04);
		biasValue.biasHigh = (dynapseBias & 0x08);

		biasValue.coarseValue = coarseValueReverse((dynapseBias >> 12) & 0x07);
		biasValue.fineValue = (dynapseBias >> 4) & 0xFF;
	}

	return (biasValue);
}
