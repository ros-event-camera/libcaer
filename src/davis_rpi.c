#include "davis_rpi.h"
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define PIZERO_PERI_BASE 0x20000000
#define GPIO_REG_BASE (PIZERO_PERI_BASE + 0x200000) /* GPIO controller */
#define GPIO_REG_LEN  0xB4

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define GPIO_INP(gpioReg, gpioId) gpioReg[(gpioId)/10] &= U32T(~(7 << (((gpioId)%10)*3)))
#define GPIO_OUT(gpioReg, gpioId) gpioReg[(gpioId)/10] |= U32T(1 << (((gpioId)%10)*3))
#define GPIO_ALT(gpioReg, gpioId, altFunc) gpioReg[(gpioId)/10] |= U32T(((altFunc)<=3?(altFunc)+4:(altFunc)==4?3:2) << (((gpioId)%10)*3))

#define GPIO_SET(gpioReg, gpioId) gpioReg[7] = U32T(1 << (gpioId))  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR(gpioReg, gpioId) gpioReg[10] = U32T(1 << (gpioId)) // clears bits which are 1 ignores bits which are 0

#define GPIO_GET(gpioReg, gpioId) (gpioReg[13] & U32T(1 << (gpioId))) // 0 if LOW, (1<<g) if HIGH

#define SPI_DEVICE0_CS0 "/dev/spidev0.0"
#define SPI_BITS_PER_WORD 8
#define SPI_SPEED_HZ (8 * 1000 * 1000)

#define GPIO_AER_REQ 5
#define GPIO_AER_ACK 3

// Data is in GPIOs 12-27, so just shift and mask (by cast to 16bit).
#define GPIO_AER_DATA(gpioReg) U16T(gpioReg[13] >> 12)

static void davisRPiLog(enum caer_log_level logLevel, davisRPiHandle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static bool davisRPiSendDefaultFPGAConfig(caerDeviceHandle cdh);
static bool davisRPiSendDefaultChipConfig(caerDeviceHandle cdh);
static bool gpioThreadStart(davisRPiHandle handle);
static void gpioThreadStop(davisRPiHandle handle);
static bool initRPi(davisRPiHandle handle);
static void closeRPi(davisRPiHandle handle);
static int gpioThreadRun(void *handlePtr);
static bool spiInit(davisRPiState state);
static void spiClose(davisRPiState state);
static bool spiConfigSend(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param);
static bool spiConfigReceive(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param);
static bool handleChipBiasSend(davisRPiState state, uint8_t paramAddr, uint32_t param);
static bool handleChipBiasReceive(davisRPiState state, uint8_t paramAddr, uint32_t *param);
static void davisRPiDataTranslator(davisRPiHandle handle, const uint16_t *buffer, size_t bufferSize);
bool davisRPiROIConfigure(caerDeviceHandle cdh, uint8_t roiRegion, bool enable, uint16_t startX, uint16_t startY,
	uint16_t endX, uint16_t endY);

#if DAVIS_RPI_BENCHMARK == 1
static void setupGPIOTest(davisRPiHandle handle, enum benchmarkMode mode);
static void shutdownGPIOTest(davisRPiHandle handle);
#endif

static bool initRPi(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	// Ensure global FDs are always uninitialized.
	state->gpio.spiFd = -1;

	int devGpioMemFd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (devGpioMemFd < 0) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to open '/dev/gpiomem'.");
		return (false);
	}

	state->gpio.gpioReg = mmap(NULL, GPIO_REG_LEN, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, devGpioMemFd,
		GPIO_REG_BASE);

	close(devGpioMemFd);

	if (state->gpio.gpioReg == MAP_FAILED) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to map GPIO memory region.");
		return (false);
	}

	// Setup SPI. Upload done by separate tool, at boot.
	if (!spiInit(state)) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to initialize SPI.");
		closeRPi(handle);
		return (false);
	}

	// Initialize SPI lock last! This avoids having to track its initialization status
	// separately. We also don't have to destroy it in closeRPi(), but only later in exit.
	if (mtx_init(&state->gpio.spiLock, mtx_plain) != thrd_success) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to initialize SPI lock.");
		closeRPi(handle);
		return (false);
	}

#if DAVIS_RPI_BENCHMARK == 0
	// After CPLD reset, query logic version.
	uint32_t param = 0;
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_VERSION, &param);

	if (param < DAVIS_RPI_REQUIRED_LOGIC_REVISION) {
		davisRPiLog(CAER_LOG_CRITICAL, handle,
			"Device logic revision too old. You have revision %" PRIu32 "; but at least revision %" PRIu32 " is required. Please updated by following the Flashy upgrade documentation at 'http://inilabs.com/support/reflashing/'.",
			param, DAVIS_RPI_REQUIRED_LOGIC_REVISION);

		closeRPi(handle);
		mtx_destroy(&state->gpio.spiLock);
		return (false);
	}
#endif

	return (true);
}

static void closeRPi(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	// SPI lock mutex destroyed in main exit.
	spiClose(state);

	// Unmap GPIO memory region.
	munmap((void *) state->gpio.gpioReg, GPIO_REG_LEN);
}

static int gpioThreadRun(void *handlePtr) {
	davisRPiHandle handle = handlePtr;
	davisRPiState state = &handle->state;

	davisRPiLog(CAER_LOG_DEBUG, handle, "Starting GPIO communication thread ...");

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char threadName[MAX_THREAD_NAME_LENGTH + 1]; // +1 for terminating NUL character.
	strncpy(threadName, handle->info.deviceString, MAX_THREAD_NAME_LENGTH);
	threadName[MAX_THREAD_NAME_LENGTH] = '\0';

	thrd_set_name(threadName);

	// Allocate data memory. Up to two data points per transaction.
	uint16_t *data = malloc(DAVIS_RPI_MAX_TRANSACTION_NUM * 2 * sizeof(uint16_t));
	if (data == NULL) {
		davisRPiLog(CAER_LOG_DEBUG, handle, "Failed to allocate memory for GPIO communication.");

		atomic_store(&state->gpio.threadState, THR_EXITED);
		return (EXIT_FAILURE);
	}

	// Signal data thread ready back to start function.
	atomic_store(&state->gpio.threadState, THR_RUNNING);

	davisRPiLog(CAER_LOG_DEBUG, handle, "GPIO communication thread running.");

#if DAVIS_RPI_BENCHMARK == 1
	// Start GPIO testing.
	spiConfigSend(state, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, true);
	setupGPIOTest(handle, ZEROS);
#endif

	// Handle GPIO port reading.
	while (atomic_load_explicit(&state->gpio.threadState, memory_order_relaxed) == THR_RUNNING) {
		size_t readTransactions = DAVIS_RPI_MAX_TRANSACTION_NUM;
		size_t dataSize = 0;

		while (readTransactions-- > 0) {
			// Do transaction via DDR-AER. Is there a request?
			size_t noReqCount = 0;
			while (GPIO_GET(state->gpio.gpioReg, GPIO_AER_REQ) != 0) {
				// Track failed wait on requests, and simply break early once
				// the maximum is reached, to avoid dead-locking in here.
				noReqCount++;
				if (noReqCount == DAVIS_RPI_MAX_WAIT_REQ_COUNT) {
					goto processData;
				}
			}

			// Request is present, latch data.
			data[dataSize] = GPIO_AER_DATA(state->gpio.gpioReg);
			dataSize++;

			// ACK ACK! (active-low, so clear).
			GPIO_CLR(state->gpio.gpioReg, GPIO_AER_ACK);

			// Wait for REQ to go back off (high).
			while (GPIO_GET(state->gpio.gpioReg, GPIO_AER_REQ) == 0) {
				;
			}

			// Latch data again.
			data[dataSize] = GPIO_AER_DATA(state->gpio.gpioReg);
			dataSize++;

			// ACK ACK off! (active-low, so set).
			GPIO_SET(state->gpio.gpioReg, GPIO_AER_ACK);
		}

		// Translate data. Support testing/benchmarking.
processData:
		if (dataSize > 0) {
			davisRPiDataTranslator(handle, data, dataSize);
		}

#if DAVIS_RPI_BENCHMARK == 1
		if (state->benchmark.dataCount >= DAVIS_RPI_BENCHMARK_LIMIT_EVENTS) {
			shutdownGPIOTest(handle);

			if (state->benchmark.testMode == ALTERNATING) {
				// Last test just done.
				spiConfigSend(state, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, false);

				// SPECIAL OUT: call exceptional shut-down callback and exit.
				if (state->gpio.shutdownCallback != NULL) {
					state->gpio.shutdownCallback(state->gpio.shutdownCallbackPtr);
				}
				break;
			}

			setupGPIOTest(handle, state->benchmark.testMode + 1);
		}
#endif
	}

	free(data);

	// Ensure threadRun is false on termination.
	atomic_store(&state->gpio.threadState, THR_EXITED);

	davisRPiLog(CAER_LOG_DEBUG, handle, "GPIO communication thread shut down.");

	return (EXIT_SUCCESS);
}

static bool spiInit(davisRPiState state) {
	state->gpio.spiFd = open(SPI_DEVICE0_CS0, O_RDWR | O_SYNC);
	if (state->gpio.spiFd < 0) {
		return (false);
	}

	uint8_t spiMode = SPI_MODE_0;

	if ((ioctl(state->gpio.spiFd, SPI_IOC_WR_MODE, &spiMode) < 0)
		|| (ioctl(state->gpio.spiFd, SPI_IOC_RD_MODE, &spiMode) < 0)) {
		return (false);
	}

	uint8_t spiBitsPerWord = SPI_BITS_PER_WORD;

	if ((ioctl(state->gpio.spiFd, SPI_IOC_WR_BITS_PER_WORD, &spiBitsPerWord) < 0)
		|| (ioctl(state->gpio.spiFd, SPI_IOC_RD_BITS_PER_WORD, &spiBitsPerWord) < 0)) {
		return (false);
	}

	uint32_t spiSpeedHz = SPI_SPEED_HZ;

	if ((ioctl(state->gpio.spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeedHz) < 0)
		|| (ioctl(state->gpio.spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &spiSpeedHz) < 0)) {
		return (false);
	}

	return (true);
}

static void spiClose(davisRPiState state) {
	if (state->gpio.spiFd >= 0) {
		close(state->gpio.spiFd);
	}
}

static inline bool spiTransfer(davisRPiState state, uint8_t *spiOutput, uint8_t *spiInput) {
	struct spi_ioc_transfer spiTransfer;
	memset(&spiTransfer, 0, sizeof(struct spi_ioc_transfer));

	spiTransfer.tx_buf = (__u64) spiOutput;
	spiTransfer.rx_buf = (__u64) spiInput;
	spiTransfer.len = SPI_CONFIG_MSG_SIZE;
	spiTransfer.speed_hz = SPI_SPEED_HZ;
	spiTransfer.bits_per_word = SPI_BITS_PER_WORD;
	spiTransfer.cs_change = false; // Documentation is misleading, see 'https://github.com/beagleboard/kernel/issues/85#issuecomment-32304365'.

	mtx_lock(&state->gpio.spiLock);

	int result = ioctl(state->gpio.spiFd, SPI_IOC_MESSAGE(1), &spiTransfer);

	mtx_unlock(&state->gpio.spiLock);

	return ((result < 0) ? (false) : (true));
}

static bool spiSend(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	uint8_t spiOutput[SPI_CONFIG_MSG_SIZE] = { 0 };

	// Highest bit of first byte is zero to indicate write operation.
	spiOutput[0] = (moduleAddr & 0x7F);
	spiOutput[1] = paramAddr;
	spiOutput[2] = (uint8_t) (param >> 24);
	spiOutput[3] = (uint8_t) (param >> 16);
	spiOutput[4] = (uint8_t) (param >> 8);
	spiOutput[5] = (uint8_t) (param >> 0);

	return (spiTransfer(state, spiOutput, NULL));
}

static bool spiReceive(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param) {
	uint8_t spiOutput[SPI_CONFIG_MSG_SIZE] = { 0 };
	uint8_t spiInput[SPI_CONFIG_MSG_SIZE] = { 0 };

	// Highest bit of first byte is one to indicate read operation.
	spiOutput[0] = (moduleAddr | 0x80);
	spiOutput[1] = paramAddr;

	if (!spiTransfer(state, spiOutput, spiInput)) {
		return (false);
	}

	*param = 0;
	*param |= U32T(spiInput[2] << 24);
	*param |= U32T(spiInput[3] << 16);
	*param |= U32T(spiInput[4] << 8);
	*param |= U32T(spiInput[5] << 0);

	return (true);
}

static bool spiConfigSend(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	// Handle biases/chip config separately.
	if (moduleAddr == DAVIS_CONFIG_BIAS) {
		return (handleChipBiasSend(state, paramAddr, param));
	}

	// Standard SPI send.
	return (spiSend(state, moduleAddr, paramAddr, param));
}

static bool spiConfigReceive(davisRPiState state, uint8_t moduleAddr, uint8_t paramAddr, uint32_t *param) {
	// Handle biases/chip config separately.
	if (moduleAddr == DAVIS_CONFIG_BIAS) {
		return (handleChipBiasReceive(state, paramAddr, param));
	}

	// Standard SPI receive.
	return (spiReceive(state, moduleAddr, paramAddr, param));
}

static inline uint8_t setBitInByte(uint8_t byteIn, uint8_t idx, bool value) {
	if (value) {
		// Flip bit on if enabled.
		return (U8T(byteIn | U8T(0x01 << idx)));
	}
	else {
		// Flip bit off if disabled.
		return (U8T(byteIn & U8T(~(0x01 << idx))));
	}
}

static inline uint8_t getBitInByte(uint8_t byteIn, uint8_t idx) {
	return ((byteIn >> idx) & 0x01);
}

static bool handleChipBiasSend(davisRPiState state, uint8_t paramAddr, uint32_t param) {
	// All addresses below 128 are biases, 128 and up are chip configuration register elements.

	// Entry delay.
	usleep(500);

	if (paramAddr <= DAVIS_BIAS_ADDRESS_MAX) {
		// Handle biases.
		if ((state->biasing.currentBiasArray[paramAddr][0] == U8T(param >> 8))
			&& (state->biasing.currentBiasArray[paramAddr][1] == U8T(param >> 0))) {
			// No changes, return right away.
			return (true);
		}

		// Store new values.
		state->biasing.currentBiasArray[paramAddr][0] = U8T(param >> 8);
		state->biasing.currentBiasArray[paramAddr][1] = U8T(param >> 0);

		uint8_t biasVal0, biasVal1;

		if (paramAddr < 8) {
			// Flip and reverse coarse bits, due to an on-chip routing mistake.
			biasVal0 = ((((state->biasing.currentBiasArray[paramAddr][0] & 0x01) ^ 0x01) << 4) & 0x10);
			biasVal0 = (uint8_t) (biasVal0
				| ((((state->biasing.currentBiasArray[paramAddr][1] & 0x80) ^ 0x80) >> 2) & 0x20));
			biasVal0 = (uint8_t) (biasVal0 | (((state->biasing.currentBiasArray[paramAddr][1] & 0x40) ^ 0x40) & 0x40));

			biasVal1 = state->biasing.currentBiasArray[paramAddr][1] & 0x3F;
		}
		else if (paramAddr < 35) {
			// The first byte of a coarse/fine bias needs to have the coarse bits
			// flipped and reversed, due to an on-chip routing mistake.
			biasVal0 = state->biasing.currentBiasArray[paramAddr][0] ^ 0x70;
			biasVal0 = (uint8_t) ((biasVal0 & ~0x50) | ((biasVal0 & 0x40) >> 2) | ((biasVal0 & 0x10) << 2));

			biasVal1 = state->biasing.currentBiasArray[paramAddr][1];
		}
		else {
			// SSN/SSP are fine as-is.
			biasVal0 = state->biasing.currentBiasArray[paramAddr][0];
			biasVal1 = state->biasing.currentBiasArray[paramAddr][1];
		}

		// Write bias: 8bit address + 16bit value to register 0.
		uint32_t value = 0;
		value |= U32T(paramAddr << 16);
		value |= U32T(biasVal0 << 8);
		value |= U32T(biasVal1 << 0);

		if (!spiSend(state, DAVIS_CONFIG_BIAS, 0, value)) {
			return (false);
		}

		// Wait 480us for bias write to complete, so sleep for 1ms.
		usleep(480 * 2);

		return (true);
	}
	else {
		// Handle chip configuration.
		// Store old value for later change detection.
		uint8_t oldChipRegister[DAVIS_CHIP_REG_LENGTH] = { 0 };
		memcpy(oldChipRegister, state->biasing.currentChipRegister, DAVIS_CHIP_REG_LENGTH);

		switch (paramAddr) {
			case 128: // DigitalMux0
				state->biasing.currentChipRegister[5] = (uint8_t) ((state->biasing.currentChipRegister[5] & 0xF0)
					| (U8T(param) & 0x0F));
				break;

			case 129: // DigitalMux1
				state->biasing.currentChipRegister[5] = (uint8_t) ((state->biasing.currentChipRegister[5] & 0x0F)
					| ((U8T(param) << 4) & 0xF0));
				break;

			case 130: // DigitalMux2
				state->biasing.currentChipRegister[6] = (uint8_t) ((state->biasing.currentChipRegister[6] & 0xF0)
					| (U8T(param) & 0x0F));
				break;

			case 131: // DigitalMux3
				state->biasing.currentChipRegister[6] = (uint8_t) ((state->biasing.currentChipRegister[6] & 0x0F)
					| ((U8T(param) << 4) & 0xF0));
				break;

			case 132: // AnalogMux0
				state->biasing.currentChipRegister[0] = (uint8_t) ((state->biasing.currentChipRegister[0] & 0x0F)
					| ((U8T(param) << 4) & 0xF0));
				break;

			case 133: // AnalogMux1
				state->biasing.currentChipRegister[1] = (uint8_t) ((state->biasing.currentChipRegister[1] & 0xF0)
					| (U8T(param) & 0x0F));
				break;

			case 134: // AnalogMux2
				state->biasing.currentChipRegister[1] = (uint8_t) ((state->biasing.currentChipRegister[1] & 0x0F)
					| ((U8T(param) << 4) & 0xF0));
				break;

			case 135: // BiasMux0
				state->biasing.currentChipRegister[0] = (uint8_t) ((state->biasing.currentChipRegister[0] & 0xF0)
					| (U8T(param) & 0x0F));
				break;

			case 136: // ResetCalibNeuron
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 0,
					(U8T(param) & 0x01));
				break;

			case 137: // TypeNCalibNeuron
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 1,
					(U8T(param) & 0x01));
				break;

			case 138: // ResetTestPixel
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 2,
					(U8T(param) & 0x01));
				break;

			case 140: // AERnArow
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 4,
					(U8T(param) & 0x01));
				break;

			case 141: // UseAOut
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 5,
					(U8T(param) & 0x01));
				break;

			case 142: // GlobalShutter
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 6,
					(U8T(param) & 0x01));
				break;

			case 143: // SelectGrayCounter
				state->biasing.currentChipRegister[2] = setBitInByte(state->biasing.currentChipRegister[2], 7,
					(U8T(param) & 0x01));
				break;

			default:
				return (false);
		}

		// Check if value changed, only send out if it did.
		if (memcmp(oldChipRegister, state->biasing.currentChipRegister, DAVIS_CHIP_REG_LENGTH) == 0) {
			return (true);
		}

		// Write config chain lower bits: 32bits to register 1.
		uint32_t value = 0;
		value |= U32T(state->biasing.currentChipRegister[3] << 24);
		value |= U32T(state->biasing.currentChipRegister[2] << 16);
		value |= U32T(state->biasing.currentChipRegister[1] << 8);
		value |= U32T(state->biasing.currentChipRegister[0] << 0);

		if (!spiSend(state, DAVIS_CONFIG_BIAS, 1, value)) {
			return (false);
		}

		// Write config chain upper bits: 24bits to register 2.
		value = 0;
		value |= U32T(state->biasing.currentChipRegister[6] << 16);
		value |= U32T(state->biasing.currentChipRegister[5] << 8);
		value |= U32T(state->biasing.currentChipRegister[4] << 0);

		if (!spiSend(state, DAVIS_CONFIG_BIAS, 2, value)) {
			return (false);
		}

		// Wait 700us for chip configuration write to complete, so sleep for 2ms.
		usleep(700 * 2);

		return (true);
	}
}

static bool handleChipBiasReceive(davisRPiState state, uint8_t paramAddr, uint32_t *param) {
	// All addresses below 128 are biases, 128 and up are chip configuration register elements.
	*param = 0;

	if (paramAddr <= DAVIS_BIAS_ADDRESS_MAX) {
		// Handle biases.
		// Get value directly from FX3 memory. Device doesn't support reads.
		*param |= U32T(state->biasing.currentBiasArray[paramAddr][0] << 8);
		*param |= U32T(state->biasing.currentBiasArray[paramAddr][1] << 0);

		return (true);
	}
	else {
		// Handle chip configuration.
		// Get value directly from FX3 memory. Device doesn't support reads.
		switch (paramAddr) {
			case 128: // DigitalMux0
				*param |= (state->biasing.currentChipRegister[5] & 0x0F);
				break;

			case 129: // DigitalMux1
				*param |= ((state->biasing.currentChipRegister[5] >> 4) & 0x0F);
				break;

			case 130: // DigitalMux2
				*param |= (state->biasing.currentChipRegister[6] & 0x0F);
				break;

			case 131: // DigitalMux3
				*param |= ((state->biasing.currentChipRegister[6] >> 4) & 0x0F);
				break;

			case 132: // AnalogMux0
				*param |= ((state->biasing.currentChipRegister[0] >> 4) & 0x0F);
				break;

			case 133: // AnalogMux1
				*param |= (state->biasing.currentChipRegister[1] & 0x0F);
				break;

			case 134: // AnalogMux2
				*param |= ((state->biasing.currentChipRegister[1] >> 4) & 0x0F);
				break;

			case 135: // BiasMux0
				*param |= (state->biasing.currentChipRegister[0] & 0x0F);
				break;

			case 136: // ResetCalibNeuron
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 0);
				break;

			case 137: // TypeNCalibNeuron
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 1);
				break;

			case 138: // ResetTestPixel
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 2);
				break;

			case 140: // AERnArow
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 4);
				break;

			case 141: // UseAOut
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 5);
				break;

			case 142: // GlobalShutter
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 6);
				break;

			case 143: // SelectGrayCounter
				*param |= (uint8_t) getBitInByte(state->biasing.currentChipRegister[2], 7);
				break;

			default:
				return (false);
		}

		return (true);
	}
}

static void davisRPiLog(enum caer_log_level logLevel, davisRPiHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(caerLogFileDescriptorsGetFirst(), caerLogFileDescriptorsGetSecond(),
		atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel, handle->info.deviceString,
		format, argumentList);
	va_end(argumentList);
}

static inline bool apsPixelIsActive(davisRPiState state, uint16_t x, uint16_t y) {
	for (size_t i = 0; i < APS_ROI_REGIONS; i++) {
		// Skip disabled ROI regions.
		if (!state->aps.roi.enabled[i]) {
			continue;
		}

		if ((x >= state->aps.roi.positionX[i]) && (x < (state->aps.roi.positionX[i] + state->aps.roi.sizeX[i]))
			&& (y >= state->aps.roi.positionY[i]) && (y < (state->aps.roi.positionY[i] + state->aps.roi.sizeY[i]))) {
			return (true);
		}
	}

	return (false);
}

static inline void apsCalculateIndexes(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	// Recalculate the index inside of pixels[] where each successive
	// pixel value gotten from the device goes to.
	uint16_t x = (state->aps.flipX) ? U16T(state->aps.sizeX - 1) : (0);
	uint16_t y = (state->aps.flipY) ? U16T(state->aps.sizeY - 1) : (0);

	// CDAVIS support.
	bool cDavisOffsetDirection = false;
	int16_t cDavisOffset = 0;

	state->aps.expectedCountX = 0;
	memset(state->aps.expectedCountY, 0, (size_t) state->aps.sizeX * sizeof(uint16_t));

	size_t index = 0;

	for (uint16_t i = 0; i < state->aps.sizeX; i++) {
		uint16_t activePixels = 0;

		for (uint16_t j = 0; j < state->aps.sizeY; j++) {
			uint16_t xDest = x;
			uint16_t yDest = y;

			// CDAVIS support: first 320 pixels are even, then odd.
			if (IS_DAVISRGB(handle->info.chipID)) {
				if (state->aps.flipY) {
					yDest = U16T(yDest - cDavisOffset);
				}
				else {
					yDest = U16T(yDest + cDavisOffset);
				}

				if (!cDavisOffsetDirection) { // Increasing
					cDavisOffset++;

					if (cDavisOffset == 320) {
						// Switch to decreasing after last even pixel.
						cDavisOffsetDirection = true;
						cDavisOffset = 319;
					}
				}
				else { // Decreasing
					cDavisOffset = I16T(cDavisOffset - 3);
				}
			}

			if (state->aps.invertXY) {
				SWAP_VAR(uint16_t, xDest, yDest);
			}

			if (apsPixelIsActive(state, xDest, yDest)) {
				// pixelIndexes is laid out in column order because that's how
				// frame update will access it naturally later.
				state->aps.frame.pixelIndexes[index++] = (size_t) ((yDest * handle->info.apsSizeX) + xDest);
				activePixels++;
			}

			if (state->aps.flipY) {
				y--;
			}
			else {
				y++;
			}
		}

		if (activePixels > 0) {
			state->aps.expectedCountY[state->aps.expectedCountX] = activePixels;
			state->aps.expectedCountX++;
		}

		// Reset Y for next iteration.
		y = (state->aps.flipY) ? U16T(state->aps.sizeY - 1) : (0);

		// CDAVIS support: reset for next iteration.
		if (IS_DAVISRGB(handle->info.chipID)) {
			cDavisOffsetDirection = false;
			cDavisOffset = 0;
		}

		if (state->aps.flipX) {
			x--;
		}
		else {
			x++;
		}
	}

	davisRPiLog(CAER_LOG_DEBUG, handle, "Recalculated APS ROI indexes.");
}

static inline void apsROIUpdateSizes(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	bool recalculateIndexes = false;

	// Calculate APS ROI sizes for each region.
	for (size_t i = 0; i < APS_ROI_REGIONS; i++) {
		uint16_t startColumn = state->aps.roi.startColumn[i];
		uint16_t startRow = state->aps.roi.startRow[i];
		uint16_t endColumn = state->aps.roi.endColumn[i];
		uint16_t endRow = state->aps.roi.endRow[i];

		// Check that ROI region is enabled and Start <= End.
		bool roiEnabledCol = (startColumn < state->aps.sizeX) && (endColumn < state->aps.sizeX);
		bool roiEnabledRow = (startRow < state->aps.sizeY) && (endRow < state->aps.sizeY);
		bool roiValidColRow = (startColumn <= endColumn) && (startRow <= endRow);

		if (state->aps.roi.deviceEnabled[i] && roiEnabledCol && roiEnabledRow && roiValidColRow) {
			state->aps.roi.enabled[i] = true;

			uint16_t newPositionX = startColumn;
			uint16_t newPositionY = startRow;

			uint16_t newSizeX = U16T(endColumn + 1 - startColumn);
			uint16_t newSizeY = U16T(endRow + 1 - startRow);

			if (state->aps.invertXY) {
				SWAP_VAR(uint16_t, newPositionX, newPositionY);
				SWAP_VAR(uint16_t, newSizeX, newSizeY);
			}

			if ((state->aps.roi.positionX[i] != newPositionX) || (state->aps.roi.positionY[i] != newPositionY)
				|| (state->aps.roi.sizeX[i] != newSizeX) || (state->aps.roi.sizeY[i] != newSizeY)) {
				state->aps.roi.positionX[i] = newPositionX;
				state->aps.roi.positionY[i] = newPositionY;

				state->aps.roi.sizeX[i] = newSizeX;
				state->aps.roi.sizeY[i] = newSizeY;

				recalculateIndexes = true;
			}

			davisRPiLog(CAER_LOG_DEBUG, handle, "APS ROI region %zu enabled - posX=%d, posY=%d, sizeX=%d, sizeY=%d.", i,
				state->aps.roi.positionX[i], state->aps.roi.positionY[i], state->aps.roi.sizeX[i],
				state->aps.roi.sizeY[i]);
		}
		else {
			// If was enabled but now isn't, must recalculate indexes.
			if (state->aps.roi.enabled[i]) {
				recalculateIndexes = true;
			}

			// Turn off this ROI region for sure, can be because disabled OR wrong col/row values.
			state->aps.roi.enabled[i] = false;

			state->aps.roi.positionX[i] = state->aps.roi.sizeX[i] = U16T(handle->info.apsSizeX);
			state->aps.roi.positionY[i] = state->aps.roi.sizeY[i] = U16T(handle->info.apsSizeY);

			davisRPiLog(CAER_LOG_DEBUG, handle, "APS ROI region %zu disabled.", i);
		}
	}

	if (recalculateIndexes) {
		// Calculate where pixels should go.
		apsCalculateIndexes(handle);
	}
}

static inline void apsInitFrame(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	state->aps.currentReadoutType = APS_READOUT_RESET;
	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		state->aps.countX[i] = 0;
		state->aps.countY[i] = 0;

		state->aps.frame.pixelIndexesPosition[i] = 0;
	}

	// Update ROI region data (position, size).
	apsROIUpdateSizes(handle);

	// Write out start of frame timestamp.
	state->aps.frame.tsStartFrame = state->timestamps.current;

	// Send APS info event out (as special event).
	caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(state->currentPackets.special,
		state->currentPackets.specialPosition);
	caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
	caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_START);
	caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
	state->currentPackets.specialPosition++;
}

static inline void apsUpdateFrame(davisRPiHandle handle, uint16_t data) {
	davisRPiState state = &handle->state;

	size_t pixelPosition = state->aps.frame.pixelIndexes[state->aps.frame.pixelIndexesPosition[state->aps
		.currentReadoutType]];
	state->aps.frame.pixelIndexesPosition[state->aps.currentReadoutType]++;

	// Separate debug support.
#if APS_DEBUG_FRAME == 1
	// Check for overflow.
	data = (data > 1023) ? (1023) : (data);

	// Normalize the ADC value to 16bit generic depth. This depends on ADC used.
	data = U16T(data << (16 - APS_ADC_DEPTH));

	// Reset read, put into resetPixels here.
	if (state->aps.currentReadoutType == APS_READOUT_RESET) {
		state->aps.frame.resetPixels[pixelPosition] = data;
	}

	// Signal read, put into pixels here.
	if (state->aps.currentReadoutType == APS_READOUT_SIGNAL) {
		state->aps.frame.pixels[pixelPosition] = data;
	}
#else
	// Standard CDS support.
	bool isCDavisGS = (IS_DAVISRGB(handle->info.chipID) && state->aps.globalShutter);

	if (((state->aps.currentReadoutType == APS_READOUT_RESET) && (!isCDavisGS))
		|| ((state->aps.currentReadoutType == APS_READOUT_SIGNAL) && isCDavisGS)) {
		state->aps.frame.resetPixels[pixelPosition] = data;
	}
	else {
		uint16_t resetValue = 0;
		uint16_t signalValue = 0;

		if (isCDavisGS) {
			// DAVIS RGB GS has inverted samples, signal read comes first
			// and was stored above inside state->aps.currentResetFrame.
			resetValue = data;
			signalValue = state->aps.frame.resetPixels[pixelPosition];
		}
		else {
			resetValue = state->aps.frame.resetPixels[pixelPosition];
			signalValue = data;
		}

		int32_t pixelValue = 0;

		if ((resetValue < 384) || (signalValue == 0)) {
			// If the signal value is 0, that is only possible if the camera
			// has seen tons of light. In that case, the photo-diode current
			// may be greater than the reset current, and the reset value
			// never goes back up fully, which results in black spots where
			// there is too much light. This confuses algorithms, so we filter
			// this out here by setting the pixel to white in that case.
			// Another effect of the same thing is the reset value not going
			// back up to a decent value, so we also filter that out here.
			pixelValue = 1023;
		}
		else {
			// Do CDS.
			pixelValue = resetValue - signalValue;

			// Check for underflow.
			pixelValue = (pixelValue < 0) ? (0) : (pixelValue);

			// Check for overflow.
			pixelValue = (pixelValue > 1023) ? (1023) : (pixelValue);
		}

		// Normalize the ADC value to 16bit generic depth. This depends on ADC used.
		pixelValue = pixelValue << (16 - APS_ADC_DEPTH);

		state->aps.frame.pixels[pixelPosition] = htole16(U16T(pixelValue));
	}
#endif

	davisRPiLog(CAER_LOG_DEBUG, handle,
		"APS ADC Sample: column=%" PRIu16 ", row=%" PRIu16 ", index=%zu, data=%" PRIu16 ".",
		state->aps.countX[state->aps.currentReadoutType], state->aps.countY[state->aps.currentReadoutType],
		pixelPosition, data);
}

static inline bool apsEndFrame(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	bool validFrame = true;

	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		int32_t checkValue = state->aps.expectedCountX;

		// Check main reset read against zero if disabled.
		if ((i == APS_READOUT_RESET) && (!state->aps.resetRead)) {
			checkValue = 0;
		}

		davisRPiLog(CAER_LOG_DEBUG, handle, "APS Frame End: CountX[%zu] is %d.", i, state->aps.countX[i]);

		if (state->aps.countX[i] != checkValue) {
			davisRPiLog(CAER_LOG_ERROR, handle, "APS Frame End - %zu: wrong column count %d detected, expected %d.", i,
				state->aps.countX[i], checkValue);
			validFrame = false;
		}
	}

	// Send APS info event out (as special event).
	caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(state->currentPackets.special,
		state->currentPackets.specialPosition);
	caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
	caerSpecialEventSetType(currentSpecialEvent, APS_FRAME_END);
	caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
	state->currentPackets.specialPosition++;

	return (validFrame);
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +-2 g - 16384 LSB/g
	// 1 - +-4 g - 8192 LSB/g
	// 2 - +-8 g - 4096 LSB/g
	// 3 - +-16 g - 2048 LSB/g
	float accelScale = 65536.0F / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Gyroscope scale is:
	// 0 - +-250 °/s - 131 LSB/°/s
	// 1 - +-500 °/s - 65.5 LSB/°/s
	// 2 - +-1000 °/s - 32.8 LSB/°/s
	// 3 - +-2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0F / (float) U32T(500 * (1 << imuGyroScale));

	return (gyroScale);
}

static inline void freeAllDataMemory(davisRPiState state) {
	dataExchangeDestroy(&state->dataExchange);

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPackets.polarity != NULL) {
		free(&state->currentPackets.polarity->packetHeader);
		state->currentPackets.polarity = NULL;

		containerGenerationSetPacket(&state->container, POLARITY_EVENT, NULL);
	}

	if (state->currentPackets.special != NULL) {
		free(&state->currentPackets.special->packetHeader);
		state->currentPackets.special = NULL;

		containerGenerationSetPacket(&state->container, SPECIAL_EVENT, NULL);
	}

	if (state->currentPackets.frame != NULL) {
		free(&state->currentPackets.frame->packetHeader);
		state->currentPackets.frame = NULL;

		containerGenerationSetPacket(&state->container, FRAME_EVENT, NULL);
	}

	if (state->currentPackets.imu6 != NULL) {
		free(&state->currentPackets.imu6->packetHeader);
		state->currentPackets.imu6 = NULL;

		containerGenerationSetPacket(&state->container, IMU6_EVENT, NULL);
	}

	containerGenerationDestroy(&state->container);

	if (state->aps.frame.pixels != NULL) {
		free(state->aps.frame.pixels);
		state->aps.frame.pixels = NULL;
	}

	if (state->aps.frame.resetPixels != NULL) {
		free(state->aps.frame.resetPixels);
		state->aps.frame.resetPixels = NULL;
	}

	if (state->aps.frame.pixelIndexes != NULL) {
		free(state->aps.frame.pixelIndexes);
		state->aps.frame.pixelIndexes = NULL;
	}

	if (state->aps.expectedCountY != NULL) {
		free(state->aps.expectedCountY);
		state->aps.expectedCountY = NULL;
	}
}

caerDeviceHandle davisRPiOpen(uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict,
	const char *serialNumberRestrict) {
	(void) (busNumberRestrict);
	(void) (devAddressRestrict);
	(void) (serialNumberRestrict);

	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DAVIS_RPI_DEVICE_NAME);

	davisRPiHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DAVIS_RPI;

	davisRPiState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	dataExchangeSettingsInit(&state->dataExchange);

	// Packet settings (size (in events) and time interval (in µs)).
	containerGenerationSettingsInit(&state->container);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);

	// Set device string.
	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s ID-%" PRIu16, DAVIS_RPI_DEVICE_NAME, deviceID);

	char *fullLogString = malloc(fullLogStringLength + 1);
	if (fullLogString == NULL) {
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device string.");
		free(handle);

		return (NULL);
	}

	snprintf(fullLogString, fullLogStringLength + 1, "%s ID-%" PRIu16, DAVIS_RPI_DEVICE_NAME, deviceID);

	handle->info.deviceString = fullLogString;

	// Open the DAVIS device on the Raspberry Pi.
	if (!initRPi(handle)) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to open device.");
		free(handle->info.deviceString);
		free(handle);

		return (NULL);
	}

	// Populate info variables based on data from device.
	uint32_t param32 = 0;

	handle->info.deviceID = I16T(deviceID);
	strncpy(handle->info.deviceSerialNumber, "0001", 4 + 1);
	handle->info.deviceUSBBusNumber = 0;
	handle->info.deviceUSBDeviceAddress = 0;
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_VERSION, &param32);
	handle->info.logicVersion = I16T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
	handle->info.deviceIsMaster = param32;
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	handle->info.logicClock = I16T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_ADC_CLOCK, &param32);
	handle->info.adcClock = I16T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
	handle->info.chipID = I16T(param32);

	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_PIXEL_FILTER, &param32);
	handle->info.dvsHasPixelFilter = param32;
	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_BACKGROUND_ACTIVITY_FILTER, &param32);
	handle->info.dvsHasBackgroundActivityFilter = param32;
	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_TEST_EVENT_GENERATOR, &param32);
	handle->info.dvsHasTestEventGenerator = param32;
	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_ROI_FILTER, &param32);
	handle->info.dvsHasROIFilter = param32;
	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_HAS_STATISTICS, &param32);
	handle->info.dvsHasStatistics = param32;

	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_COLOR_FILTER, &param32);
	handle->info.apsColorFilter = U8T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_GLOBAL_SHUTTER, &param32);
	handle->info.apsHasGlobalShutter = param32;
	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_QUAD_ROI, &param32);
	handle->info.apsHasQuadROI = param32;
	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_HAS_INTERNAL_ADC, &param32);
	handle->info.apsHasInternalADC = param32;
	handle->info.apsHasExternalADC = !handle->info.apsHasInternalADC;

	spiConfigReceive(state, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_HAS_GENERATOR, &param32);
	handle->info.extInputHasGenerator = param32;
	spiConfigReceive(state, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_HAS_EXTRA_DETECTORS, &param32);
	handle->info.extInputHasExtraDetectors = param32;

	spiConfigReceive(state, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_HAS_STATISTICS, &param32);
	handle->info.muxHasStatistics = param32;

	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_COLUMNS, &param32);
	state->dvs.sizeX = I16T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_SIZE_ROWS, &param32);
	state->dvs.sizeY = I16T(param32);

	spiConfigReceive(state, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ORIENTATION_INFO, &param32);
	state->dvs.invertXY = param32 & 0x04;

	davisRPiLog(CAER_LOG_DEBUG, handle, "DVS Size X: %d, Size Y: %d, Invert: %d.", state->dvs.sizeX, state->dvs.sizeY,
		state->dvs.invertXY);

	if (state->dvs.invertXY) {
		handle->info.dvsSizeX = state->dvs.sizeY;
		handle->info.dvsSizeY = state->dvs.sizeX;
	}
	else {
		handle->info.dvsSizeX = state->dvs.sizeX;
		handle->info.dvsSizeY = state->dvs.sizeY;
	}

	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_COLUMNS, &param32);
	state->aps.sizeX = I16T(param32);
	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SIZE_ROWS, &param32);
	state->aps.sizeY = I16T(param32);

	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ORIENTATION_INFO, &param32);
	state->aps.invertXY = param32 & 0x04;
	state->aps.flipX = param32 & 0x02;
	state->aps.flipY = param32 & 0x01;

	davisRPiLog(CAER_LOG_DEBUG, handle, "APS Size X: %d, Size Y: %d, Invert: %d, Flip X: %d, Flip Y: %d.",
		state->aps.sizeX, state->aps.sizeY, state->aps.invertXY, state->aps.flipX, state->aps.flipY);

	if (state->aps.invertXY) {
		handle->info.apsSizeX = state->aps.sizeY;
		handle->info.apsSizeY = state->aps.sizeX;
	}
	else {
		handle->info.apsSizeX = state->aps.sizeX;
		handle->info.apsSizeY = state->aps.sizeY;
	}

	spiConfigReceive(state, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ORIENTATION_INFO, &param32);
	state->imu.flipX = param32 & 0x04;
	state->imu.flipY = param32 & 0x02;
	state->imu.flipZ = param32 & 0x01;

	davisRPiLog(CAER_LOG_DEBUG, handle, "IMU Flip X: %d, Flip Y: %d, Flip Z: %d.", state->imu.flipX, state->imu.flipY,
		state->imu.flipZ);

	davisRPiLog(CAER_LOG_DEBUG, handle, "Initialized device successfully.");

	return ((caerDeviceHandle) handle);
}

bool davisRPiClose(caerDeviceHandle cdh) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

	davisRPiLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Close the device fully.
	// Destroy SPI mutex here, as it is initialized for sure.
	closeRPi(handle);
	mtx_destroy(&state->gpio.spiLock);

	davisRPiLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	return (true);
}

bool davisRPiSendDefaultConfig(caerDeviceHandle cdh) {
	// First send default chip/bias config.
	if (!davisRPiSendDefaultChipConfig(cdh)) {
		return (false);
	}

	// Send default FPGA config.
	if (!davisRPiSendDefaultFPGAConfig(cdh)) {
		return (false);
	}

	return (true);
}

static bool davisRPiSendDefaultFPGAConfig(caerDeviceHandle cdh) {
	davisRPiHandle handle = (davisRPiHandle) cdh;

	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL, false);

	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_ROW, 4); // in cycles @ LogicClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN, 0); // in cycles @ LogicClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW, 1); // in cycles @ LogicClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN, 0); // in cycles @ LogicClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL, false);
	if (handle->info.dvsHasPixelFilter) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, U32T(handle->info.dvsSizeX));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, U32T(handle->info.dvsSizeY));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, U32T(handle->info.dvsSizeX));
	}
	if (handle->info.dvsHasBackgroundActivityFilter) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, 80); // in 250µs blocks (so 20ms)
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, 2); // in 250µs blocks (so 500µs)
	}
	if (handle->info.dvsHasTestEventGenerator) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE, false);
	}
	if (handle->info.dvsHasROIFilter) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN,
			U32T(handle->info.dvsSizeX - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, U32T(handle->info.dvsSizeY - 1));
	}

	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, handle->info.apsHasGlobalShutter);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, U32T(handle->info.apsSizeX - 1));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, U32T(handle->info.apsSizeY - 1));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROI0_ENABLED, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4000); // in µs, converted to cycles @ ADCClock later
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_DELAY, 1000); // in µs, converted to cycles @ ADCClock later
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROW_SETTLE, U32T(handle->info.adcClock / 3)); // in cycles @ ADCClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_SETTLE, U32T(handle->info.adcClock)); // in cycles @ ADCClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_NULL_SETTLE, U32T(handle->info.adcClock / 10)); // in cycles @ ADCClock

	if (handle->info.apsHasQuadROI) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_1, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_1, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_1, U32T(handle->info.apsSizeX - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_1, U32T(handle->info.apsSizeY - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_2, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_2, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_2, U32T(handle->info.apsSizeX - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_2, U32T(handle->info.apsSizeY - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_3, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_3, 0);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_3, U32T(handle->info.apsSizeX - 1));
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_3, U32T(handle->info.apsSizeY - 1));

		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROI1_ENABLED, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROI2_ENABLED, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ROI3_ENABLED, false);
	}
	if (handle->info.apsHasInternalADC) {
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_ENABLE, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SAMPLE_SETTLE, U32T(handle->info.adcClock * 2)); // in cycles @ ADCClock
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_RESET, U32T(handle->info.adcClock / 3)); // in cycles @ ADCClock
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RAMP_SHORT_RESET, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_ADC_TEST_MODE, false);
	}

	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_TEMP_STANDBY, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_STANDBY, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_STANDBY, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_CYCLE, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_LP_WAKEUP, 1);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, 1);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, 1);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, 1);

	davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
		U32T(handle->info.logicClock)); // in cycles @ LogicClock

	davisRPiConfigSet(cdh, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_RUN, false); // Microphones disabled by default.
	davisRPiConfigSet(cdh, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY, 32); // 48 KHz sampling frequency.

	if (handle->info.extInputHasGenerator) {
		// Disable generator by default. Has to be enabled manually after sendDefaultConfig() by user!
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
			U32T(handle->info.logicClock / 2)); // in cycles @ LogicClock
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);
	}

	if (handle->info.extInputHasExtraDetectors) {
		// Disable extra detectors by default. Have to be enabled manually after sendDefaultConfig() by user!
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock

		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2,
			U32T(handle->info.logicClock)); // in cycles @ LogicClock
	}

	davisRPiConfigSet(cdh, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_REQ_DELAY, 1); // in cycles @ LogicClock
	davisRPiConfigSet(cdh, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_ACK_DELAY, 1); // in cycles @ LogicClock

	return (true);
}

#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_N_TYPE_CAS(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = true, \
	.typeNormal = false, .currentLevelNormal = true }

/*
 * #define CF_P_TYPE_CAS(COARSE, FINE) (struct caer_bias_coarsefine) \
 *	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = true, .sexN = false, \
 *	.typeNormal = false, .currentLevelNormal = true }
 */

#define CF_N_TYPE_OFF(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = true, \
	.typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE_OFF(COARSE, FINE) (struct caer_bias_coarsefine) \
	{ .coarseValue = COARSE, .fineValue = FINE, .enabled = false, .sexN = false, \
	.typeNormal = true, .currentLevelNormal = true }

#define SHIFTSOURCE(REF, REG, OPMODE) (struct caer_bias_shiftedsource) \
	{ .refValue = REF, .regValue = REG, .operatingMode = OPMODE, .voltageLevel = SPLIT_GATE }

#define VDAC(VOLT, CURR) (struct caer_bias_vdac) \
	{ .voltageValue = VOLT, .currentValue = CURR }

static bool davisRPiSendDefaultChipConfig(caerDeviceHandle cdh) {
	// Default bias configuration.
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL, caerBiasVDACGenerate(VDAC(27, 6)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSCAS, caerBiasVDACGenerate(VDAC(21, 6)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFHIGH, caerBiasVDACGenerate(VDAC(32, 7)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCREFLOW, caerBiasVDACGenerate(VDAC(1, 7)));

	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LOCALBUFBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PADFOLLBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE_OFF(7, 215)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DIFFBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(4, 39)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ONBN, caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_OFFBN, caerBiasCoarseFineGenerate(CF_N_TYPE(4, 1)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PIXINVBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(CF_P_TYPE(2, 58)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_PRSFBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(1, 16)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_REFRBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(4, 25)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_READOUTBUFBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_APSROSFBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_ADCCOMPBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_COLSELLOWBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_DACBUFBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPDBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUXBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_AEPUYBP,
		caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFREFRBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_IFTHRBN,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));

	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_BIASBUFFER,
		caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));

	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSP,
		caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));
	davisRPiConfigSet(cdh, DAVIS_CONFIG_BIAS, DAVIS128_CONFIG_BIAS_SSN,
		caerBiasShiftedSourceGenerate(SHIFTSOURCE(1, 33, SHIFTED_SOURCE)));

	// Default chip configuration.
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX0, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX1, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX2, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_DIGITALMUX3, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX0, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX1, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_ANALOGMUX2, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_BIASMUX0, 0);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETCALIBNEURON, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON, false);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_RESETTESTPIXEL, true);
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_AERNAROW, false); // Use nArow in the AER state machine.
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_USEAOUT, false); // Enable analog pads for aMUX output (testing).

	// No GlobalShutter flag set here, we already set it above for the APS GS flag,
	// and that is automatically propagated to the chip config shift-register in
	// configSet() and kept in sync.

	// Select which gray counter to use with the internal ADC: '0' means the external gray counter is used, which
	// has to be supplied off-chip. '1' means the on-chip gray counter is used instead.
	davisRPiConfigSet(cdh, DAVIS_CONFIG_CHIP, DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER, 1);

	return (true);
}

bool davisRPiConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigSet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigSet(&state->container, paramAddr, param));
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

		case DAVIS_CONFIG_MUX:
			switch (paramAddr) {
				case DAVIS_CONFIG_MUX_RUN:
				case DAVIS_CONFIG_MUX_TIMESTAMP_RUN:
				case DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL:
					return (spiConfigSend(state, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						spiConfigSend(state, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 0x01);
						spiConfigSend(state, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 0x00);
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_DVS:
			switch (paramAddr) {
				case DAVIS_CONFIG_DVS_RUN:
				case DAVIS_CONFIG_DVS_ACK_DELAY_ROW:
				case DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN:
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
					return (spiConfigSend(state, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						return (spiConfigSend(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigSend(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE:
					if (handle->info.dvsHasTestEventGenerator) {
						return (spiConfigSend(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW:
					if (handle->info.dvsHasROIFilter) {
						return (spiConfigSend(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_APS:
			switch (paramAddr) {
				case DAVIS_CONFIG_APS_RUN:
				case DAVIS_CONFIG_APS_RESET_READ:
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_ROW_SETTLE:
				case DAVIS_CONFIG_APS_START_COLUMN_0:
				case DAVIS_CONFIG_APS_START_ROW_0:
				case DAVIS_CONFIG_APS_END_COLUMN_0:
				case DAVIS_CONFIG_APS_END_ROW_0:
				case DAVIS_CONFIG_APS_ROI0_ENABLED:
				case DAVIS_CONFIG_APS_RESET_SETTLE:
				case DAVIS_CONFIG_APS_NULL_SETTLE:
					return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Exposure and Frame Delay are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					if (!atomic_load(&state->aps.autoExposure.enabled)) {
						state->aps.autoExposure.lastSetExposure = param;

						uint32_t exposureCC = U32T(param * U16T(handle->info.adcClock));
						return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, exposureCC));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_FRAME_DELAY: {
					// Exposure and Frame Delay are in µs, must be converted to native FPGA cycles
					// by multiplying with ADC clock value.
					uint32_t delayCC = U32T(param * U16T(handle->info.adcClock));
					return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, delayCC));
					break;
				}

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						// Keep in sync with chip config module GlobalShutter parameter.
						if (!spiConfigSend(state, DAVIS_CONFIG_CHIP,
						DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER, param)) {
							return (false);
						}

						return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_1:
				case DAVIS_CONFIG_APS_END_COLUMN_1:
				case DAVIS_CONFIG_APS_START_COLUMN_2:
				case DAVIS_CONFIG_APS_END_COLUMN_2:
				case DAVIS_CONFIG_APS_START_COLUMN_3:
				case DAVIS_CONFIG_APS_END_COLUMN_3:
				case DAVIS_CONFIG_APS_START_ROW_1:
				case DAVIS_CONFIG_APS_END_ROW_1:
				case DAVIS_CONFIG_APS_START_ROW_2:
				case DAVIS_CONFIG_APS_END_ROW_2:
				case DAVIS_CONFIG_APS_START_ROW_3:
				case DAVIS_CONFIG_APS_END_ROW_3:
				case DAVIS_CONFIG_APS_ROI1_ENABLED:
				case DAVIS_CONFIG_APS_ROI2_ENABLED:
				case DAVIS_CONFIG_APS_ROI3_ENABLED:
					if (handle->info.apsHasQuadROI) {
						return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SAMPLE_ENABLE:
				case DAVIS_CONFIG_APS_SAMPLE_SETTLE:
				case DAVIS_CONFIG_APS_RAMP_RESET:
				case DAVIS_CONFIG_APS_RAMP_SHORT_RESET:
				case DAVIS_CONFIG_APS_ADC_TEST_MODE:
					if (handle->info.apsHasInternalADC) {
						return (spiConfigSend(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SNAPSHOT: {
					// Use multi-command VR for more efficient implementation of snapshot,
					// that also guarantees returning to the default state (not running).
					if (param) {
						spiConfigSend(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, 0x01);
						spiConfigSend(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, 0x00);
					}
					break;
				}

				case DAVIS_CONFIG_APS_AUTOEXPOSURE:
					atomic_store(&state->aps.autoExposure.enabled, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN:
				case DAVIS_CONFIG_IMU_TEMP_STANDBY:
				case DAVIS_CONFIG_IMU_ACCEL_STANDBY:
				case DAVIS_CONFIG_IMU_GYRO_STANDBY:
				case DAVIS_CONFIG_IMU_LP_CYCLE:
				case DAVIS_CONFIG_IMU_LP_WAKEUP:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigSend(state, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH:
					return (spiConfigSend(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigSend(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1:
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2:
					if (handle->info.extInputHasExtraDetectors) {
						return (spiConfigSend(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_BIAS: // Also DAVIS_CONFIG_CHIP (starts at address 128).
			if (paramAddr < 128) {
				// BIASING (DAVIS_CONFIG_BIAS).
				// All new DAVISes use the new bias generator with 37 branches.
				switch (paramAddr) {
					// Same and shared between all of the above chips.
					case DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL:
					case DAVIS128_CONFIG_BIAS_APSCAS:
					case DAVIS128_CONFIG_BIAS_ADCREFHIGH:
					case DAVIS128_CONFIG_BIAS_ADCREFLOW:
					case DAVIS128_CONFIG_BIAS_LOCALBUFBN:
					case DAVIS128_CONFIG_BIAS_PADFOLLBN:
					case DAVIS128_CONFIG_BIAS_DIFFBN:
					case DAVIS128_CONFIG_BIAS_ONBN:
					case DAVIS128_CONFIG_BIAS_OFFBN:
					case DAVIS128_CONFIG_BIAS_PIXINVBN:
					case DAVIS128_CONFIG_BIAS_PRBP:
					case DAVIS128_CONFIG_BIAS_PRSFBP:
					case DAVIS128_CONFIG_BIAS_REFRBP:
					case DAVIS128_CONFIG_BIAS_READOUTBUFBP:
					case DAVIS128_CONFIG_BIAS_APSROSFBN:
					case DAVIS128_CONFIG_BIAS_ADCCOMPBP:
					case DAVIS128_CONFIG_BIAS_COLSELLOWBN:
					case DAVIS128_CONFIG_BIAS_DACBUFBP:
					case DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN:
					case DAVIS128_CONFIG_BIAS_AEPDBN:
					case DAVIS128_CONFIG_BIAS_AEPUXBP:
					case DAVIS128_CONFIG_BIAS_AEPUYBP:
					case DAVIS128_CONFIG_BIAS_IFREFRBN:
					case DAVIS128_CONFIG_BIAS_IFTHRBN:
					case DAVIS128_CONFIG_BIAS_BIASBUFFER:
					case DAVIS128_CONFIG_BIAS_SSP:
					case DAVIS128_CONFIG_BIAS_SSN:
						return (spiConfigSend(state, DAVIS_CONFIG_BIAS, paramAddr, param));
						break;

					default:
						return (false);
						break;
				}
			}
			else {
				// CHIP CONFIGURATION (DAVIS_CONFIG_CHIP).
				switch (paramAddr) {
					// Chip configuration common to all chips.
					case DAVIS128_CONFIG_CHIP_DIGITALMUX0:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX1:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX2:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX3:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX0:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX1:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX2:
					case DAVIS128_CONFIG_CHIP_BIASMUX0:
					case DAVIS128_CONFIG_CHIP_RESETCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_RESETTESTPIXEL:
					case DAVIS128_CONFIG_CHIP_AERNAROW:
					case DAVIS128_CONFIG_CHIP_USEAOUT:
					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						return (spiConfigSend(state, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							// Keep in sync with APS module GlobalShutter parameter.
							if (!spiConfigSend(state, DAVIS_CONFIG_APS,
							DAVIS_CONFIG_APS_GLOBAL_SHUTTER, param)) {
								return (false);
							}

							return (spiConfigSend(state, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					default:
						return (false);
						break;
				}
			}

			return (false);
			break;

		case DAVIS_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DAVIS_CONFIG_DDRAER:
			switch (paramAddr) {
				case DAVIS_CONFIG_DDRAER_RUN:
				case DAVIS_CONFIG_DDRAER_REQ_DELAY:
				case DAVIS_CONFIG_DDRAER_ACK_DELAY:
					return (spiConfigSend(state, DAVIS_CONFIG_DDRAER, paramAddr, param));
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

bool davisRPiConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigGet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigGet(&state->container, paramAddr, param));
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

		case DAVIS_CONFIG_MUX:
			switch (paramAddr) {
				case DAVIS_CONFIG_MUX_RUN:
				case DAVIS_CONFIG_MUX_TIMESTAMP_RUN:
				case DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE:
				case DAVIS_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_APS_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_IMU_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_MUX_DROP_MIC_ON_TRANSFER_STALL:
					return (spiConfigReceive(state, DAVIS_CONFIG_MUX, paramAddr, param));
					break;

				case DAVIS_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DAVIS_CONFIG_MUX_STATISTICS_DVS_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_DVS_DROPPED + 1:
				case DAVIS_CONFIG_MUX_STATISTICS_APS_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_APS_DROPPED + 1:
				case DAVIS_CONFIG_MUX_STATISTICS_IMU_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_IMU_DROPPED + 1:
				case DAVIS_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED + 1:
				case DAVIS_CONFIG_MUX_STATISTICS_MIC_DROPPED:
				case DAVIS_CONFIG_MUX_STATISTICS_MIC_DROPPED + 1:
					if (handle->info.muxHasStatistics) {
						return (spiConfigReceive(state, DAVIS_CONFIG_MUX, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_DVS:
			switch (paramAddr) {
				case DAVIS_CONFIG_DVS_SIZE_COLUMNS:
				case DAVIS_CONFIG_DVS_SIZE_ROWS:
				case DAVIS_CONFIG_DVS_ORIENTATION_INFO:
				case DAVIS_CONFIG_DVS_RUN:
				case DAVIS_CONFIG_DVS_ACK_DELAY_ROW:
				case DAVIS_CONFIG_DVS_ACK_DELAY_COLUMN:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_ROW:
				case DAVIS_CONFIG_DVS_ACK_EXTENSION_COLUMN:
				case DAVIS_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_DVS_FILTER_ROW_ONLY_EVENTS:
				case DAVIS_CONFIG_DVS_EXTERNAL_AER_CONTROL:
				case DAVIS_CONFIG_DVS_HAS_PIXEL_FILTER:
				case DAVIS_CONFIG_DVS_HAS_BACKGROUND_ACTIVITY_FILTER:
				case DAVIS_CONFIG_DVS_HAS_TEST_EVENT_GENERATOR:
					return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					break;

				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN:
					if (handle->info.dvsHasPixelFilter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME:
					if (handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_TEST_EVENT_GENERATOR_ENABLE:
					if (handle->info.dvsHasTestEventGenerator) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN:
				case DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW:
					if (handle->info.dvsHasROIFilter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_ROW:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_ROW + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_COLUMN:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_COLUMN + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_DROPPED:
				case DAVIS_CONFIG_DVS_STATISTICS_EVENTS_DROPPED + 1:
					if (handle->info.dvsHasStatistics) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_PIXELS:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_PIXELS + 1:
					if (handle->info.dvsHasStatistics && handle->info.dvsHasPixelFilter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY + 1:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD:
				case DAVIS_CONFIG_DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD + 1:
					if (handle->info.dvsHasStatistics && handle->info.dvsHasBackgroundActivityFilter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_APS:
			switch (paramAddr) {
				case DAVIS_CONFIG_APS_SIZE_COLUMNS:
				case DAVIS_CONFIG_APS_SIZE_ROWS:
				case DAVIS_CONFIG_APS_ORIENTATION_INFO:
				case DAVIS_CONFIG_APS_COLOR_FILTER:
				case DAVIS_CONFIG_APS_RUN:
				case DAVIS_CONFIG_APS_RESET_READ:
				case DAVIS_CONFIG_APS_WAIT_ON_TRANSFER_STALL:
				case DAVIS_CONFIG_APS_ROW_SETTLE:
				case DAVIS_CONFIG_APS_HAS_GLOBAL_SHUTTER:
				case DAVIS_CONFIG_APS_HAS_QUAD_ROI:
				case DAVIS_CONFIG_APS_HAS_INTERNAL_ADC:
				case DAVIS_CONFIG_APS_START_COLUMN_0:
				case DAVIS_CONFIG_APS_END_COLUMN_0:
				case DAVIS_CONFIG_APS_START_ROW_0:
				case DAVIS_CONFIG_APS_END_ROW_0:
				case DAVIS_CONFIG_APS_ROI0_ENABLED:
				case DAVIS_CONFIG_APS_RESET_SETTLE:
				case DAVIS_CONFIG_APS_NULL_SETTLE:
					return (spiConfigReceive(state, DAVIS_CONFIG_APS, paramAddr, param));
					break;

				case DAVIS_CONFIG_APS_EXPOSURE:
					// Use stored value, no need to call out to USB for this one.
					*param = state->aps.autoExposure.lastSetExposure;
					break;

				case DAVIS_CONFIG_APS_FRAME_DELAY: {
					// Exposure and Frame Delay are in µs, must be converted from native FPGA cycles
					// by dividing with ADC clock value.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(state, DAVIS_CONFIG_APS, paramAddr, &cyclesValue)) {
						return (false);
					}

					*param = U32T(cyclesValue / U16T(handle->info.adcClock));

					return (true);
					break;
				}

				case DAVIS_CONFIG_APS_GLOBAL_SHUTTER:
					if (handle->info.apsHasGlobalShutter) {
						return (spiConfigReceive(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_START_COLUMN_1:
				case DAVIS_CONFIG_APS_END_COLUMN_1:
				case DAVIS_CONFIG_APS_START_COLUMN_2:
				case DAVIS_CONFIG_APS_END_COLUMN_2:
				case DAVIS_CONFIG_APS_START_COLUMN_3:
				case DAVIS_CONFIG_APS_END_COLUMN_3:
				case DAVIS_CONFIG_APS_START_ROW_1:
				case DAVIS_CONFIG_APS_END_ROW_1:
				case DAVIS_CONFIG_APS_START_ROW_2:
				case DAVIS_CONFIG_APS_END_ROW_2:
				case DAVIS_CONFIG_APS_START_ROW_3:
				case DAVIS_CONFIG_APS_END_ROW_3:
				case DAVIS_CONFIG_APS_ROI1_ENABLED:
				case DAVIS_CONFIG_APS_ROI2_ENABLED:
				case DAVIS_CONFIG_APS_ROI3_ENABLED:
					if (handle->info.apsHasQuadROI) {
						return (spiConfigReceive(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SAMPLE_ENABLE:
				case DAVIS_CONFIG_APS_SAMPLE_SETTLE:
				case DAVIS_CONFIG_APS_RAMP_RESET:
				case DAVIS_CONFIG_APS_RAMP_SHORT_RESET:
				case DAVIS_CONFIG_APS_ADC_TEST_MODE:
					if (handle->info.apsHasInternalADC) {
						return (spiConfigReceive(state, DAVIS_CONFIG_APS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_APS_SNAPSHOT:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DAVIS_CONFIG_APS_AUTOEXPOSURE:
					*param = atomic_load(&state->aps.autoExposure.enabled);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_IMU:
			switch (paramAddr) {
				case DAVIS_CONFIG_IMU_RUN:
				case DAVIS_CONFIG_IMU_TEMP_STANDBY:
				case DAVIS_CONFIG_IMU_ACCEL_STANDBY:
				case DAVIS_CONFIG_IMU_GYRO_STANDBY:
				case DAVIS_CONFIG_IMU_LP_CYCLE:
				case DAVIS_CONFIG_IMU_LP_WAKEUP:
				case DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER:
				case DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER:
				case DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE:
				case DAVIS_CONFIG_IMU_GYRO_FULL_SCALE:
					return (spiConfigReceive(state, DAVIS_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_HAS_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_HAS_EXTRA_DETECTORS:
					return (spiConfigReceive(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_USE_CUSTOM_SIGNAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DAVIS_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigReceive(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY1:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH1:
				case DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_RISING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_FALLING_EDGES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSES2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY2:
				case DAVIS_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH2:
					if (handle->info.extInputHasExtraDetectors) {
						return (spiConfigReceive(state, DAVIS_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_BIAS: // Also DAVIS_CONFIG_CHIP (starts at address 128).
			if (paramAddr < 128) {
				// All new DAVISes use the new bias generator with 37 branches.
				switch (paramAddr) {
					// Same and shared between all of the above chips.
					case DAVIS128_CONFIG_BIAS_APSOVERFLOWLEVEL:
					case DAVIS128_CONFIG_BIAS_APSCAS:
					case DAVIS128_CONFIG_BIAS_ADCREFHIGH:
					case DAVIS128_CONFIG_BIAS_ADCREFLOW:
					case DAVIS128_CONFIG_BIAS_LOCALBUFBN:
					case DAVIS128_CONFIG_BIAS_PADFOLLBN:
					case DAVIS128_CONFIG_BIAS_DIFFBN:
					case DAVIS128_CONFIG_BIAS_ONBN:
					case DAVIS128_CONFIG_BIAS_OFFBN:
					case DAVIS128_CONFIG_BIAS_PIXINVBN:
					case DAVIS128_CONFIG_BIAS_PRBP:
					case DAVIS128_CONFIG_BIAS_PRSFBP:
					case DAVIS128_CONFIG_BIAS_REFRBP:
					case DAVIS128_CONFIG_BIAS_READOUTBUFBP:
					case DAVIS128_CONFIG_BIAS_APSROSFBN:
					case DAVIS128_CONFIG_BIAS_ADCCOMPBP:
					case DAVIS128_CONFIG_BIAS_COLSELLOWBN:
					case DAVIS128_CONFIG_BIAS_DACBUFBP:
					case DAVIS128_CONFIG_BIAS_LCOLTIMEOUTBN:
					case DAVIS128_CONFIG_BIAS_AEPDBN:
					case DAVIS128_CONFIG_BIAS_AEPUXBP:
					case DAVIS128_CONFIG_BIAS_AEPUYBP:
					case DAVIS128_CONFIG_BIAS_IFREFRBN:
					case DAVIS128_CONFIG_BIAS_IFTHRBN:
					case DAVIS128_CONFIG_BIAS_BIASBUFFER:
					case DAVIS128_CONFIG_BIAS_SSP:
					case DAVIS128_CONFIG_BIAS_SSN:
						return (spiConfigReceive(state, DAVIS_CONFIG_BIAS, paramAddr, param));
						break;

					default:
						return (false);
						break;
				}
			}
			else {
				// CHIP CONFIGURATION (DAVIS_CONFIG_CHIP).
				switch (paramAddr) {
					// Chip configuration common to all chips.
					case DAVIS128_CONFIG_CHIP_DIGITALMUX0:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX1:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX2:
					case DAVIS128_CONFIG_CHIP_DIGITALMUX3:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX0:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX1:
					case DAVIS128_CONFIG_CHIP_ANALOGMUX2:
					case DAVIS128_CONFIG_CHIP_BIASMUX0:
					case DAVIS128_CONFIG_CHIP_RESETCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_TYPENCALIBNEURON:
					case DAVIS128_CONFIG_CHIP_RESETTESTPIXEL:
					case DAVIS128_CONFIG_CHIP_AERNAROW:
					case DAVIS128_CONFIG_CHIP_USEAOUT:
					case DAVIS128_CONFIG_CHIP_SELECTGRAYCOUNTER:
						return (spiConfigReceive(state, DAVIS_CONFIG_CHIP, paramAddr, param));
						break;

					case DAVIS128_CONFIG_CHIP_GLOBAL_SHUTTER:
						// Only supported by some chips.
						if (handle->info.apsHasGlobalShutter) {
							return (spiConfigReceive(state, DAVIS_CONFIG_CHIP, paramAddr, param));
						}
						break;

					default:
						return (false);
						break;
				}
			}

			return (false);
			break;

		case DAVIS_CONFIG_SYSINFO:
			switch (paramAddr) {
				case DAVIS_CONFIG_SYSINFO_LOGIC_VERSION:
				case DAVIS_CONFIG_SYSINFO_CHIP_IDENTIFIER:
				case DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER:
				case DAVIS_CONFIG_SYSINFO_LOGIC_CLOCK:
				case DAVIS_CONFIG_SYSINFO_ADC_CLOCK:
					return (spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DAVIS_CONFIG_DDRAER:
			switch (paramAddr) {
				case DAVIS_CONFIG_DDRAER_RUN:
				case DAVIS_CONFIG_DDRAER_REQ_DELAY:
				case DAVIS_CONFIG_DDRAER_ACK_DELAY:
					return (spiConfigReceive(state, DAVIS_CONFIG_DDRAER, paramAddr, param));
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

static bool gpioThreadStart(davisRPiHandle handle) {
	// Start GPIO communication thread.
	if ((errno = thrd_create(&handle->state.gpio.thread, &gpioThreadRun, handle)) != thrd_success) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to create GPIO thread. Error: %d.", errno);
		return (false);
	}

	while (atomic_load(&handle->state.gpio.threadState) == THR_IDLE) {
		thrd_yield();
	}

	return (true);
}

static void gpioThreadStop(davisRPiHandle handle) {
	// Shut down GPIO communication thread.
	atomic_store(&handle->state.gpio.threadState, THR_EXITED);

	// Wait for GPIO communication thread to terminate.
	if ((errno = thrd_join(handle->state.gpio.thread, NULL)) != thrd_success) {
		// This should never happen!
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to join GPIO thread. Error: %d.", errno);
	}
}

bool davisRPiDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

	// Store new data available/not available anymore call-backs.
	dataExchangeSetNotify(&state->dataExchange, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr);

	state->gpio.shutdownCallback = dataShutdownNotify;
	state->gpio.shutdownCallbackPtr = dataShutdownUserPtr;

	containerGenerationCommitTimestampReset(&state->container);

	if (!dataExchangeBufferInit(&state->dataExchange)) {
		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

#if DAVIS_RPI_BENCHMARK == 0
	// Allocate packets.
	if (!containerGenerationAllocate(&state->container, DAVIS_RPI_EVENT_TYPES)) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPackets.polarity = caerPolarityEventPacketAllocate(DAVIS_RPI_POLARITY_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPackets.polarity == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentPackets.special = caerSpecialEventPacketAllocate(DAVIS_RPI_SPECIAL_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0);
	if (state->currentPackets.special == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	state->currentPackets.frame = caerFrameEventPacketAllocate(DAVIS_RPI_FRAME_DEFAULT_SIZE,
		I16T(handle->info.deviceID), 0, handle->info.apsSizeX, handle->info.apsSizeY, APS_ADC_CHANNELS);
	if (state->currentPackets.frame == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
		return (false);
	}

	state->currentPackets.imu6 = caerIMU6EventPacketAllocate(DAVIS_RPI_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID),
		0);
	if (state->currentPackets.imu6 == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
		return (false);
	}

	state->aps.frame.pixels = calloc((size_t) (state->aps.sizeX * state->aps.sizeY * APS_ADC_CHANNELS),
		sizeof(uint16_t));
	if (state->aps.frame.pixels == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS pixels memory.");
		return (false);
	}

	state->aps.frame.resetPixels = calloc((size_t) (state->aps.sizeX * state->aps.sizeY * APS_ADC_CHANNELS),
		sizeof(uint16_t));
	if (state->aps.frame.resetPixels == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS reset pixels memory.");
		return (false);
	}

	state->aps.frame.pixelIndexes = calloc((size_t) (state->aps.sizeX * state->aps.sizeY * APS_ADC_CHANNELS),
		sizeof(size_t));
	if (state->aps.frame.pixelIndexes == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS pixel positions memory.");
		return (false);
	}

	state->aps.expectedCountY = calloc((size_t) state->aps.sizeX, sizeof(uint16_t));
	if (state->aps.expectedCountY == NULL) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate APS expected count Y memory.");
		return (false);
	}

	// Default IMU settings (for event parsing).
	uint32_t param32 = 0;

	spiConfigReceive(state, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, &param32);
	state->imu.accelScale = calculateIMUAccelScale(U8T(param32));
	spiConfigReceive(state, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, &param32);
	state->imu.gyroScale = calculateIMUGyroScale(U8T(param32));

	// Ignore multi-part events (APS and IMU) at startup, so that any initial
	// incomplete event is ignored. The START events reset this as soon as
	// the first one is observed.
	state->aps.ignoreEvents = true;
	state->imu.ignoreEvents = true;

	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, &param32);
	state->aps.globalShutter = param32;
	spiConfigReceive(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RESET_READ, &param32);
	state->aps.resetRead = param32;

	// Fully disable APS ROI by default. Device will send the correct values to enable.
	for (size_t i = 0; i < APS_ROI_REGIONS; i++) {
		state->aps.roi.startColumn[i] = state->aps.roi.endColumn[i] = U16T(state->aps.sizeX);
		state->aps.roi.startRow[i] = state->aps.roi.endRow[i] = U16T(state->aps.sizeY);

		state->aps.roi.positionX[i] = state->aps.roi.sizeX[i] = U16T(handle->info.apsSizeX);
		state->aps.roi.positionY[i] = state->aps.roi.sizeY[i] = U16T(handle->info.apsSizeY);
	}
#endif

	if (!gpioThreadStart(handle)) {
		freeAllDataMemory(state);

		davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to start GPIO data transfers.");
		return (false);
	}

#if DAVIS_RPI_BENCHMARK == 0
	if (dataExchangeStartProducers(&state->dataExchange)) {
		// Enable data transfer on USB end-point 2.
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, true);
		// Do NOT enable additional ExtInput detectors, those are always user controlled.

		// Enable data transfer only after enabling the data producers, so that the chip
		// has time to start up and we avoid the initial data flood.
		struct timespec noDataSleep = { .tv_sec = 0, .tv_nsec = 500000000 };
		thrd_sleep(&noDataSleep, NULL);

		davisRPiConfigSet(cdh, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, true);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, true);
	}
#endif

	return (true);
}

bool davisRPiDataStop(caerDeviceHandle cdh) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

#if DAVIS_RPI_BENCHMARK == 0
	if (dataExchangeStopProducers(&state->dataExchange)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR2, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR1, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_EXTINPUT, DAVIS_CONFIG_EXTINPUT_RUN_DETECTOR, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_FORCE_CHIP_BIAS_ENABLE, false); // Ensure chip turns off.
		davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, false); // Turn off timestamping too.
		davisRPiConfigSet(cdh, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, false);
		davisRPiConfigSet(cdh, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, false);
	}
#endif

	gpioThreadStop(handle);

	dataExchangeBufferEmpty(&state->dataExchange);

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPackets.polarityPosition = 0;
	state->currentPackets.specialPosition = 0;
	state->currentPackets.framePosition = 0;
	state->currentPackets.imu6Position = 0;

	// Reset private composite events. 'aps.currentEvent' is taken care of in freeAllDataMemory().
	memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

	return (true);
}

caerEventPacketContainer davisRPiDataGet(caerDeviceHandle cdh) {
	davisRPiHandle handle = (davisRPiHandle) cdh;
	davisRPiState state = &handle->state;

	return (dataExchangeGet(&state->dataExchange, &state->gpio.threadState));
}

#if DAVIS_RPI_BENCHMARK == 1

static void davisRPiDataTranslator(davisRPiHandle handle, const uint16_t *buffer, size_t bufferSize) {
	davisRPiState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down.
	if (atomic_load(&state->gpio.threadState) != THR_RUNNING) {
		return;
	}

	for (size_t eventIdx = 0; eventIdx < bufferSize; eventIdx++) {
		uint16_t value = le16toh(buffer[eventIdx]);

		if (value != state->benchmark.expectedValue) {
			davisRPiLog(CAER_LOG_ERROR, handle, "Failed benchmark test, unexpected value of %d, instead of %d.", value,
				state->benchmark.expectedValue);
			state->benchmark.errorCount++;

			state->benchmark.expectedValue = value;
		}

		switch (state->benchmark.testMode) {
			case ZEROS:
				state->benchmark.expectedValue = 0;

				break;

			case ONES:
				state->benchmark.expectedValue = 0xFFFF;

				break;

			case SWITCHING:
				if (state->benchmark.expectedValue == 0xFFFF) {
					state->benchmark.expectedValue = 0;
				}
				else {
					state->benchmark.expectedValue = 0xFFFF;
				}

				break;

			case ALTERNATING:
				if (state->benchmark.expectedValue == 0x5555) {
					state->benchmark.expectedValue = 0xAAAA;
				}
				else {
					state->benchmark.expectedValue = 0x5555;
				}

				break;

			case COUNTER:
			default:
				state->benchmark.expectedValue++;

				break;
		}
	}

	// Count events.
	state->benchmark.dataCount += U32T(bufferSize);
}

static void setupGPIOTest(davisRPiHandle handle, enum benchmarkMode mode) {
	davisRPiState state = &handle->state;

	// Reset global variables.
	state->benchmark.dataCount = 0;
	state->benchmark.errorCount = 0;

	if (mode == ONES) {
		state->benchmark.expectedValue = 0xFFFF;
	}
	else if (mode == ALTERNATING) {
		state->benchmark.expectedValue = 0x5555;
	}
	else {
		state->benchmark.expectedValue = 0;
	}

	// Set global test mode variable.
	state->benchmark.testMode = mode;

	// Set test mode.
	spiConfigSend(state, 0x00, 0x09, mode);

	// Enable test.
	spiConfigSend(state, 0x00, 0x08, true);

	// Remember test start time.
	portable_clock_gettime_monotonic(&state->benchmark.startTime);
}

static void shutdownGPIOTest(davisRPiHandle handle) {
	davisRPiState state = &handle->state;

	// Get test end time.
	struct timespec endTime;
	portable_clock_gettime_monotonic(&endTime);

	// Disable current tests.
	spiConfigSend(state, 0x00, 0x08, false);

	// Drain FIFOs by disabling.
	spiConfigSend(state, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, false);
	sleep(1);
	spiConfigSend(state, DAVIS_CONFIG_DDRAER, DAVIS_CONFIG_DDRAER_RUN, true);

	// Check if test was successful.
	if (state->benchmark.errorCount == 0) {
		davisRPiLog(CAER_LOG_ERROR, handle, "Test %d successful (%zu events). No errors encountered.",
			state->benchmark.testMode, state->benchmark.dataCount);
	}
	else {
		davisRPiLog(CAER_LOG_ERROR, handle,
			"Test %d failed (%zu events). %zu errors encountered. See the console for more details.",
			state->benchmark.testMode, state->benchmark.dataCount, state->benchmark.errorCount);
	}

	// Calculate bandwidth.
	uint64_t diffNanoTime = (uint64_t) (((int64_t) (endTime.tv_sec - state->benchmark.startTime.tv_sec) * 1000000000LL)
		+ (int64_t) (endTime.tv_nsec - state->benchmark.startTime.tv_nsec));

	double diffSecondTime = ((double) diffNanoTime) / ((double) 1000000000ULL);

	double eventsPerSecond = ((double) state->benchmark.dataCount) / diffSecondTime;

	davisRPiLog(CAER_LOG_ERROR, handle, "Test %d: bandwidth of %g events/second (%zu events in %g seconds).",
		state->benchmark.testMode, eventsPerSecond, state->benchmark.dataCount, diffSecondTime);
}

#else

#define TS_WRAP_ADD 0x8000

static void davisRPiDataTranslator(davisRPiHandle handle, const uint16_t *buffer, size_t bufferSize) {
	davisRPiState state = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (atomic_load(&state->gpio.threadState) != THR_RUNNING) {
		return;
	}

	for (size_t eventIdx = 0; eventIdx < bufferSize; eventIdx++) {
		// Allocate new packets for next iteration as needed.
		if (!containerGenerationAllocate(&state->container, DAVIS_RPI_EVENT_TYPES)) {
			davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
			return;
		}

		if (state->currentPackets.special == NULL) {
			state->currentPackets.special = caerSpecialEventPacketAllocate(
			DAVIS_RPI_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.special == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		} // +1 to ensure space for double frame info.
		else if ((state->currentPackets.specialPosition + 1)
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.special)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerSpecialEventPacket grownPacket = (caerSpecialEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.special,
				caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.special) * 2);
			if (grownPacket == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to grow special event packet.");
				return;
			}

			state->currentPackets.special = grownPacket;
		}

		if (state->currentPackets.polarity == NULL) {
			state->currentPackets.polarity = caerPolarityEventPacketAllocate(
			DAVIS_RPI_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.polarity == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}
		else if (state->currentPackets.polarityPosition
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.polarity)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerPolarityEventPacket grownPacket = (caerPolarityEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.polarity,
				caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.polarity) * 2);
			if (grownPacket == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to grow polarity event packet.");
				return;
			}

			state->currentPackets.polarity = grownPacket;
		}

		if (state->currentPackets.frame == NULL) {
			state->currentPackets.frame = caerFrameEventPacketAllocate(
			DAVIS_RPI_FRAME_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow,
				handle->info.apsSizeX, handle->info.apsSizeY, APS_ADC_CHANNELS);
			if (state->currentPackets.frame == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate frame event packet.");
				return;
			}
		} // +3 to ensure space for Quad-ROI (and +7 for debug Quad-ROI).
		else if ((state->currentPackets.framePosition + ((APS_DEBUG_FRAME == 0) ? (3) : (3 + APS_ROI_REGIONS)))
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.frame)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerFrameEventPacket grownPacket = (caerFrameEventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.frame,
				caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.frame) * 2);
			if (grownPacket == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to grow frame event packet.");
				return;
			}

			state->currentPackets.frame = grownPacket;
		}

		if (state->currentPackets.imu6 == NULL) {
			state->currentPackets.imu6 = caerIMU6EventPacketAllocate(
			DAVIS_RPI_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.imu6 == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
				return;
			}
		}
		else if (state->currentPackets.imu6Position
			>= caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.imu6)) {
			// If not committed, let's check if any of the packets has reached its maximum
			// capacity limit. If yes, we grow them to accomodate new events.
			caerIMU6EventPacket grownPacket = (caerIMU6EventPacket) caerEventPacketGrow(
				(caerEventPacketHeader) state->currentPackets.imu6,
				caerEventPacketHeaderGetEventCapacity((caerEventPacketHeader) state->currentPackets.imu6) * 2);
			if (grownPacket == NULL) {
				davisRPiLog(CAER_LOG_CRITICAL, handle, "Failed to grow IMU6 event packet.");
				return;
			}

			state->currentPackets.imu6 = grownPacket;
		}

		bool tsReset = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(buffer[eventIdx]);

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			handleTimestampUpdateNewLogic(&state->timestamps, event, handle->info.deviceString, &state->deviceLogLevel);

			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							davisRPiLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							handleTimestampResetNewLogic(&state->timestamps, handle->info.deviceString,
								&state->deviceLogLevel);

							containerGenerationCommitTimestampReset(&state->container);
							containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets.
							uint32_t param = 0;
							spiConfigReceive(state, DAVIS_CONFIG_SYSINFO, DAVIS_CONFIG_SYSINFO_DEVICE_IS_MASTER,
								&param);

							atomic_thread_fence(memory_order_seq_cst);
							handle->info.deviceIsMaster = param;
							atomic_thread_fence(memory_order_seq_cst);
							break;
						}

						case 2: { // External input (falling edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 3: { // External input (rising edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 4: { // External input (pulse)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 5: { // IMU Start (6 axes)
							davisRPiLog(CAER_LOG_DEBUG, handle, "IMU6 Start event received.");

							state->imu.ignoreEvents = false;
							state->imu.count = 0;

							memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

							break;
						}

						case 7: { // IMU End
							if (state->imu.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "IMU End event received.");

							if (state->imu.count == IMU6_COUNT) {
								// Timestamp at event-stream insertion point.
								caerIMU6EventSetTimestamp(&state->imu.currentEvent, state->timestamps.current);

								caerIMU6EventValidate(&state->imu.currentEvent, state->currentPackets.imu6);

								// IMU6 and APS operate on an internal event and copy that to the actual output
								// packet here, in the END state, for a reason: if a packetContainer, with all its
								// packets, is committed due to hitting any of the triggers that are not TS reset
								// or TS wrap-around related, like number of polarity events, the event in the packet
								// would be left incomplete, and the event in the new packet would be corrupted.
								// We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
								// just deleting that event, but these kinds of commits happen much more often and the
								// possible data loss would be too significant. So instead we keep a private event,
								// fill it, and then only copy it into the packet here in the END state, at which point
								// the whole event is ready and cannot be broken/corrupted in any way anymore.
								caerIMU6Event imuCurrentEvent = caerIMU6EventPacketGetEvent(state->currentPackets.imu6,
									state->currentPackets.imu6Position);
								memcpy(imuCurrentEvent, &state->imu.currentEvent, sizeof(struct caer_imu6_event));
								state->currentPackets.imu6Position++;
							}
							else {
								davisRPiLog(CAER_LOG_INFO, handle,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imu.count);
							}
							break;
						}

						case 8: { // APS Global Shutter Frame Start
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS GS Frame Start event received.");
							state->aps.ignoreEvents = false;
							state->aps.globalShutter = true;
							state->aps.resetRead = true;

							apsInitFrame(handle);

							break;
						}

						case 9: { // APS Rolling Shutter Frame Start
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS RS Frame Start event received.");
							state->aps.ignoreEvents = false;
							state->aps.globalShutter = false;
							state->aps.resetRead = true;

							apsInitFrame(handle);

							break;
						}

						case 10: { // APS Frame End
							if (state->aps.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Frame End event received.");

							// NOTE: IMU6 and APS operate on an internal event and copy that to the actual output
							// packet here, in the END state, for a reason: if a packetContainer, with all its
							// packets, is committed due to hitting any of the triggers that are not TS reset
							// or TS wrap-around related, like number of polarity events, the event in the packet
							// would be left incomplete, and the event in the new packet would be corrupted.
							// We could avoid this like for the TS reset/TS wrap-around case (see tsReset) by
							// just deleting that event, but these kinds of commits happen much more often and the
							// possible data loss would be too significant. So instead we keep a private event,
							// fill it, and then only copy it into the packet here in the END state, at which point
							// the whole event is ready and cannot be broken/corrupted in any way anymore.
							bool validFrame = apsEndFrame(handle);

							// Validate event and advance frame packet position.
							if (validFrame) {
								caerFrameEventConst newFrameEvents[APS_ROI_REGIONS] = { NULL };

								for (size_t i = 0; i < APS_ROI_REGIONS; i++) {
									// Skip disabled ROI regions.
									if (!state->aps.roi.enabled[i]) {
										continue;
									}

									// Get next frame.
									caerFrameEvent frameEvent = caerFrameEventPacketGetEvent(
										state->currentPackets.frame, state->currentPackets.framePosition);
									state->currentPackets.framePosition++;
									newFrameEvents[i] = frameEvent;

									// Setup new frame.
									caerFrameEventSetColorFilter(frameEvent, handle->info.apsColorFilter);
									caerFrameEventSetROIIdentifier(frameEvent, U8T(i));
									caerFrameEventSetTSStartOfFrame(frameEvent, state->aps.frame.tsStartFrame);
									caerFrameEventSetTSStartOfExposure(frameEvent, state->aps.frame.tsStartExposure);
									caerFrameEventSetTSEndOfExposure(frameEvent, state->aps.frame.tsEndExposure);
									caerFrameEventSetTSEndOfFrame(frameEvent, state->timestamps.current);
									caerFrameEventSetPositionX(frameEvent, state->aps.roi.positionX[i]);
									caerFrameEventSetPositionY(frameEvent, state->aps.roi.positionY[i]);
									caerFrameEventSetLengthXLengthYChannelNumber(frameEvent, state->aps.roi.sizeX[i],
										state->aps.roi.sizeY[i], APS_ADC_CHANNELS, state->currentPackets.frame);
									caerFrameEventValidate(frameEvent, state->currentPackets.frame);

									// Copy pixels over row-wise.
									uint16_t *roiPixels = caerFrameEventGetPixelArrayUnsafe(frameEvent);
									size_t roiOffset = 0;
									size_t frameOffset = (state->aps.roi.positionY[i] * (size_t) handle->info.apsSizeX)
										+ state->aps.roi.positionX[i];

									for (uint16_t y = 0; y < state->aps.roi.sizeY[i]; y++) {
										memcpy(roiPixels + roiOffset, state->aps.frame.pixels + frameOffset,
											state->aps.roi.sizeX[i] * sizeof(uint16_t));

										roiOffset += state->aps.roi.sizeX[i];
										frameOffset += (size_t) handle->info.apsSizeX;
									}

									// Separate debug support.
#if APS_DEBUG_FRAME == 1
									// Get debug frame.
									caerFrameEvent debugEvent = caerFrameEventPacketGetEvent(
										state->currentPackets.frame, state->currentPackets.framePosition);
									state->currentPackets.framePosition++;

									// Setup new frame.
									caerFrameEventSetColorFilter(debugEvent, handle->info.apsColorFilter);
									caerFrameEventSetROIIdentifier(debugEvent, U8T(i + APS_ROI_REGIONS));
									caerFrameEventSetTSStartOfFrame(debugEvent, state->aps.frame.tsStartFrame);
									caerFrameEventSetTSStartOfExposure(debugEvent, state->aps.frame.tsStartExposure);
									caerFrameEventSetTSEndOfExposure(debugEvent, state->aps.frame.tsEndExposure);
									caerFrameEventSetTSEndOfFrame(debugEvent, state->timestamps.current);
									caerFrameEventSetPositionX(debugEvent, state->aps.roi.positionX[i]);
									caerFrameEventSetPositionY(debugEvent, state->aps.roi.positionY[i]);
									caerFrameEventSetLengthXLengthYChannelNumber(debugEvent, state->aps.roi.sizeX[i],
										state->aps.roi.sizeY[i], APS_ADC_CHANNELS, state->currentPackets.frame);
									caerFrameEventValidate(debugEvent, state->currentPackets.frame);

									// Copy pixels over row-wise.
									roiPixels = caerFrameEventGetPixelArrayUnsafe(debugEvent);
									roiOffset = 0;
									frameOffset = (state->aps.roi.positionY[i] * (size_t) handle->info.apsSizeX)
									+ state->aps.roi.positionX[i];

									for (uint16_t y = 0; y < state->aps.roi.sizeY[i]; y++) {
										memcpy(roiPixels + roiOffset, state->aps.frame.resetPixels + frameOffset,
											state->aps.roi.sizeX[i] * sizeof(uint16_t));

										roiOffset += state->aps.roi.sizeX[i];
										frameOffset += (size_t) handle->info.apsSizeX;
									}
#endif
								}

								// Automatic exposure control support. Call once for all ROI regions.
								if (atomic_load_explicit(&state->aps.autoExposure.enabled, memory_order_relaxed)) {
									uint32_t exposureFrameCC = U32T(
										state->aps.autoExposure.currentFrameExposure / U16T(handle->info.adcClock));

									int32_t newExposureValue = autoExposureCalculate(&state->aps.autoExposure.state,
										newFrameEvents, exposureFrameCC, state->aps.autoExposure.lastSetExposure);

									if (newExposureValue >= 0) {
										// Update exposure value. Done in main thread to avoid deadlock inside callback.
										davisRPiLog(CAER_LOG_DEBUG, handle,
											"Automatic exposure control set exposure to %" PRIi32 " µs.",
											newExposureValue);

										state->aps.autoExposure.lastSetExposure = U32T(newExposureValue);

										uint32_t newExposureCC = U32T(newExposureValue * U16T(handle->info.adcClock));

										spiConfigSend(state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE,
											newExposureCC);
									}
								}
							}

							break;
						}

						case 11: { // APS Reset Column Start
							if (state->aps.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Reset Column Start event received.");

							state->aps.currentReadoutType = APS_READOUT_RESET;
							state->aps.countY[APS_READOUT_RESET] = 0;

							// The first Reset Column Read Start is also the start
							// of the exposure for the RS.
							if ((!state->aps.globalShutter) && (state->aps.countX[APS_READOUT_RESET] == 0)) {
								state->aps.frame.tsStartExposure = state->timestamps.current;

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 12: { // APS Signal Column Start
							if (state->aps.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Signal Column Start event received.");

							state->aps.currentReadoutType = APS_READOUT_SIGNAL;
							state->aps.countY[APS_READOUT_SIGNAL] = 0;

							// The first Signal Column Read Start is also always the end
							// of the exposure time, for both RS and GS.
							if (state->aps.countX[APS_READOUT_SIGNAL] == 0) {
								state->aps.frame.tsEndExposure = state->timestamps.current;

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_END);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 13: { // APS Column End
							if (state->aps.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Column End event received.");

							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Column End: CountX[%d] is %d.",
								state->aps.currentReadoutType, state->aps.countX[state->aps.currentReadoutType]);
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS Column End: CountY[%d] is %d.",
								state->aps.currentReadoutType, state->aps.countY[state->aps.currentReadoutType]);

							if (state->aps.countY[state->aps.currentReadoutType]
								!= state->aps.expectedCountY[state->aps.countX[state->aps.currentReadoutType]]) {
								davisRPiLog(CAER_LOG_ERROR, handle,
									"APS Column End - %d - %d: wrong row count %d detected, expected %d.",
									state->aps.currentReadoutType, state->aps.countX[state->aps.currentReadoutType],
									state->aps.countY[state->aps.currentReadoutType],
									state->aps.expectedCountY[state->aps.countX[state->aps.currentReadoutType]]);
							}

							state->aps.countX[state->aps.currentReadoutType]++;

							// The last Reset Column Read End is also the start
							// of the exposure for the GS.
							if ((state->aps.globalShutter) && (state->aps.currentReadoutType == APS_READOUT_RESET)
								&& (state->aps.countX[APS_READOUT_RESET] == state->aps.expectedCountX)) {
								state->aps.frame.tsStartExposure = state->timestamps.current;

								// Send APS info event out (as special event).
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 14: { // APS Global Shutter Frame Start with no Reset Read
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS GS NORST Frame Start event received.");
							state->aps.ignoreEvents = false;
							state->aps.globalShutter = true;
							state->aps.resetRead = false;

							apsInitFrame(handle);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							state->aps.frame.tsStartExposure = state->timestamps.current;

							// Send APS info event out (as special event).
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;

							break;
						}

						case 15: { // APS Rolling Shutter Frame Start with no Reset Read
							davisRPiLog(CAER_LOG_DEBUG, handle, "APS RS NORST Frame Start event received.");
							state->aps.ignoreEvents = false;
							state->aps.globalShutter = false;
							state->aps.resetRead = false;

							apsInitFrame(handle);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							state->aps.frame.tsStartExposure = state->timestamps.current;

							// Send APS info event out (as special event).
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, APS_EXPOSURE_START);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;

							break;
						}

						case 16:
						case 17:
						case 18:
						case 19:
						case 20:
						case 21:
						case 22:
						case 23:
						case 24:
						case 25:
						case 26:
						case 27:
						case 28:
						case 29:
						case 30:
						case 31: {
							if (state->imu.ignoreEvents) {
								break;
							}
							davisRPiLog(CAER_LOG_DEBUG, handle, "IMU Scale Config event (%" PRIu16 ") received.", data);

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imu.accelScale = calculateIMUAccelScale(U16T(data >> 2) & 0x03);
							state->imu.gyroScale = calculateIMUGyroScale(data & 0x03);

							// At this point the IMU event count should be zero (reset by start).
							if (state->imu.count != 0) {
								davisRPiLog(CAER_LOG_INFO, handle,
									"IMU Scale Config: previous IMU start event missed, attempting recovery.");
							}

							// Increase IMU count by one, to a total of one (0+1=1).
							// This way we can recover from the above error of missing start, and we can
							// later discover if the IMU Scale Config event actually arrived itself.
							state->imu.count = 1;

							break;
						}

						case 32: {
							// Next Misc8 APS ROI Size events will refer to ROI region 0.
							// 0/1 used to distinguish between X and Y sizes.
							state->aps.roi.update = (0x00U << 2);
							state->aps.roi.tmpData = 0;

							state->aps.roi.deviceEnabled[0] = true;
							state->aps.roi.startColumn[0] = state->aps.roi.endColumn[0] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[0] = state->aps.roi.endRow[0] = U16T(state->aps.sizeY);
							break;
						}

						case 33: {
							// Next Misc8 APS ROI Size events will refer to ROI region 1.
							// 2/3 used to distinguish between X and Y sizes.
							state->aps.roi.update = (0x01U << 2);
							state->aps.roi.tmpData = 0;

							state->aps.roi.deviceEnabled[1] = true;
							state->aps.roi.startColumn[1] = state->aps.roi.endColumn[1] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[1] = state->aps.roi.endRow[1] = U16T(state->aps.sizeY);
							break;
						}

						case 34: {
							// Next Misc8 APS ROI Size events will refer to ROI region 2.
							// 4/5 used to distinguish between X and Y sizes.
							state->aps.roi.update = (0x02U << 2);
							state->aps.roi.tmpData = 0;

							state->aps.roi.deviceEnabled[2] = true;
							state->aps.roi.startColumn[2] = state->aps.roi.endColumn[2] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[2] = state->aps.roi.endRow[2] = U16T(state->aps.sizeY);
							break;
						}

						case 35: {
							// Next Misc8 APS ROI Size events will refer to ROI region 3.
							// 6/7 used to distinguish between X and Y sizes.
							state->aps.roi.update = (0x03U << 2);
							state->aps.roi.tmpData = 0;

							state->aps.roi.deviceEnabled[3] = true;
							state->aps.roi.startColumn[3] = state->aps.roi.endColumn[3] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[3] = state->aps.roi.endRow[3] = U16T(state->aps.sizeY);
							break;
						}

						case 36: { // External input 1 (falling edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 1 (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 37: { // External input 1 (rising edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 1 (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 38: { // External input 1 (pulse)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 1 (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT1_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 39: { // External input 2 (falling edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 2 (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 40: { // External input 2 (rising edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 2 (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 41: { // External input 2 (pulse)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External input 2 (pulse) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT2_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 42: { // External generator (falling edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External generator (falling edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 43: { // External generator (rising edge)
							davisRPiLog(CAER_LOG_DEBUG, handle, "External generator (rising edge) event received.");

							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
							break;
						}

						case 48: { // Exposure information.
							// Reset counter and value.
							state->aps.autoExposure.tmpData = 0;
							state->aps.autoExposure.currentFrameExposure = 0;
							break;
						}

						case 49: {
							// ROI region 0 disabled. No follow-up Misc8 info events.
							state->aps.roi.deviceEnabled[0] = false;
							state->aps.roi.startColumn[0] = state->aps.roi.endColumn[0] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[0] = state->aps.roi.endRow[0] = U16T(state->aps.sizeY);
							break;
						}

						case 50: {
							// ROI region 1 disabled. No follow-up Misc8 info events.
							state->aps.roi.deviceEnabled[1] = false;
							state->aps.roi.startColumn[1] = state->aps.roi.endColumn[1] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[1] = state->aps.roi.endRow[1] = U16T(state->aps.sizeY);
							break;
						}

						case 51: {
							// ROI region 2 disabled. No follow-up Misc8 info events.
							state->aps.roi.deviceEnabled[2] = false;
							state->aps.roi.startColumn[2] = state->aps.roi.endColumn[2] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[2] = state->aps.roi.endRow[2] = U16T(state->aps.sizeY);
							break;
						}

						case 52: {
							// ROI region 3 disabled. No follow-up Misc8 info events.
							state->aps.roi.deviceEnabled[3] = false;
							state->aps.roi.startColumn[3] = state->aps.roi.endColumn[3] = U16T(state->aps.sizeX);
							state->aps.roi.startRow[3] = state->aps.roi.endRow[3] = U16T(state->aps.sizeY);
							break;
						}

						default:
							davisRPiLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.",
								data);
							break;
					}
					break;

				case 1: // Y address
					// Check range conformity.
					if (data >= state->dvs.sizeY) {
						davisRPiLog(CAER_LOG_ALERT, handle, "DVS: Y address out of range (0-%d): %" PRIu16 ".",
							state->dvs.sizeY - 1, data);
						break; // Skip invalid Y address (don't update lastY).
					}

					if (state->dvs.gotY) {
						caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
							state->currentPackets.special, state->currentPackets.specialPosition);

						// Timestamp at event-stream insertion point.
						caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
						caerSpecialEventSetType(currentSpecialEvent, DVS_ROW_ONLY);
						caerSpecialEventSetData(currentSpecialEvent, state->dvs.lastY);
						caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
						state->currentPackets.specialPosition++;

						davisRPiLog(CAER_LOG_DEBUG, handle, "DVS: row-only event received for address Y=%" PRIu16 ".",
							state->dvs.lastY);
					}

					state->dvs.lastY = data;
					state->dvs.gotY = true;

					break;

				case 2: // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= state->dvs.sizeX) {
						davisRPiLog(CAER_LOG_ALERT, handle, "DVS: X address out of range (0-%d): %" PRIu16 ".",
							state->dvs.sizeX - 1, data);
						break; // Skip invalid event.
					}

					caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
						state->currentPackets.polarity, state->currentPackets.polarityPosition);

					// Timestamp at event-stream insertion point.
					caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
					caerPolarityEventSetPolarity(currentPolarityEvent, (code & 0x01));
					if (state->dvs.invertXY) {
						caerPolarityEventSetY(currentPolarityEvent, data);
						caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastY);
					}
					else {
						caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastY);
						caerPolarityEventSetX(currentPolarityEvent, data);
					}
					caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
					state->currentPackets.polarityPosition++;

					state->dvs.gotY = false;

					break;
				}

				case 4: {
					if (state->aps.ignoreEvents) {
						break;
					}

					// Ignore too big X/Y counts, can happen if column start/end events are lost.
					if ((state->aps.countX[state->aps.currentReadoutType] >= state->aps.expectedCountX)
						|| (state->aps.countY[state->aps.currentReadoutType]
							>= state->aps.expectedCountY[state->aps.countX[state->aps.currentReadoutType]])) {
						break;
					}

					apsUpdateFrame(handle, data);

					state->aps.countY[state->aps.currentReadoutType]++;

					break;
				}

				case 5: {
					// Misc 8bit data.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imu.ignoreEvents) {
								break;
							}

							// Detect missing IMU end events.
							if (state->imu.count >= IMU6_COUNT) {
								davisRPiLog(CAER_LOG_INFO, handle,
									"IMU data: IMU samples count is at maximum, discarding further samples.");
								break;
							}

							// IMU data event.
							switch (state->imu.count) {
								case 0:
									davisRPiLog(CAER_LOG_ERROR, handle,
										"IMU data: missing IMU Scale Config event. Parsing of IMU events will still be attempted, but be aware that Accel/Gyro scale conversions may be inaccurate.");
									state->imu.count = 1;
									// Fall through to next case, as if imu.count was equal to 1.

								case 1:
								case 3:
								case 5:
								case 7:
								case 9:
								case 11:
								case 13:
									state->imu.tmpData = misc8Data;
									break;

								case 2: {
									int16_t accelX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										accelX = I16T(-accelX);
									}
									caerIMU6EventSetAccelX(&state->imu.currentEvent, accelX / state->imu.accelScale);
									break;
								}

								case 4: {
									int16_t accelY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										accelY = I16T(-accelY);
									}
									caerIMU6EventSetAccelY(&state->imu.currentEvent, accelY / state->imu.accelScale);
									break;
								}

								case 6: {
									int16_t accelZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										accelZ = I16T(-accelZ);
									}
									caerIMU6EventSetAccelZ(&state->imu.currentEvent, accelZ / state->imu.accelScale);
									break;
								}

									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 340) + 36.53
								case 8: {
									int16_t temp = I16T((state->imu.tmpData << 8) | misc8Data);
									caerIMU6EventSetTemp(&state->imu.currentEvent, (temp / 340.0F) + 36.53F);
									break;
								}

								case 10: {
									int16_t gyroX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										gyroX = I16T(-gyroX);
									}
									caerIMU6EventSetGyroX(&state->imu.currentEvent, gyroX / state->imu.gyroScale);
									break;
								}

								case 12: {
									int16_t gyroY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										gyroY = I16T(-gyroY);
									}
									caerIMU6EventSetGyroY(&state->imu.currentEvent, gyroY / state->imu.gyroScale);
									break;
								}

								case 14: {
									int16_t gyroZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										gyroZ = I16T(-gyroZ);
									}
									caerIMU6EventSetGyroZ(&state->imu.currentEvent, gyroZ / state->imu.gyroScale);
									break;
								}

								default:
									davisRPiLog(CAER_LOG_ERROR, handle, "Got invalid IMU update sequence.");
									break;
							}

							state->imu.count++;

							break;

						case 1:
							// APS ROI Size Part 1 (bits 15-8).
							// Here we just store the temporary value, and use it again
							// in the next case statement.
							state->aps.roi.tmpData = U16T(misc8Data << 8);

							break;

						case 2: {
							// APS ROI Size Part 2 (bits 7-0).
							// Here we just store the values and re-use the four fields
							// sizeX/Y and positionX/Y to store endCol/Row and startCol/Row.
							// We then recalculate all the right values and set everything
							// up in START_FRAME.
							size_t apsROIRegion = state->aps.roi.update >> 2;

							if ((apsROIRegion >= APS_ROI_REGIONS) || (!state->aps.roi.deviceEnabled[apsROIRegion])) {
								break;
							}

							switch (state->aps.roi.update & 0x03) {
								case 0:
									// START COLUMN
									state->aps.roi.startColumn[apsROIRegion] = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 1:
									// START ROW
									state->aps.roi.startRow[apsROIRegion] = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 2:
									// END COLUMN
									state->aps.roi.endColumn[apsROIRegion] = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								case 3:
									// END ROW
									state->aps.roi.endRow[apsROIRegion] = U16T(state->aps.roi.tmpData | misc8Data);
									break;

								default:
									davisRPiLog(CAER_LOG_ERROR, handle, "Got invalid ROI update sequence.");
									break;
							}

							// Jump to next type of APS info (col->row, start->end).
							state->aps.roi.update++;

							break;
						}

						default:
							davisRPiLog(CAER_LOG_ERROR, handle, "Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 6: {
					// Misc 10bit data.
					uint8_t misc10Code = U8T((data & 0x0C00) >> 10);
					uint16_t misc10Data = U16T(data & 0x03FF);

					switch (misc10Code) {
						case 0:
							state->aps.autoExposure.currentFrameExposure |= (U32T(misc10Data)
								<< U32T(10 * state->aps.autoExposure.tmpData));
							state->aps.autoExposure.tmpData++;
							break;

						default:
							davisRPiLog(CAER_LOG_ERROR, handle, "Caught Misc10 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: { // Timestamp wrap
					tsBigWrap = handleTimestampWrapNewLogic(&state->timestamps, data, TS_WRAP_ADD,
						handle->info.deviceString, &state->deviceLogLevel);

					if (tsBigWrap) {
						caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
							state->currentPackets.special, state->currentPackets.specialPosition);
						caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
						caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
						caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
						state->currentPackets.specialPosition++;
					}
					else {
						containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
					}

					break;
				}

				default:
					davisRPiLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// tsReset and tsBigWrap are already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = containerGenerationGetMaxPacketSize(&state->container);
		bool containerSizeCommit = (currentPacketContainerCommitSize > 0)
			&& ((state->currentPackets.polarityPosition >= currentPacketContainerCommitSize)
				|| (state->currentPackets.specialPosition >= currentPacketContainerCommitSize)
				|| (state->currentPackets.framePosition >= currentPacketContainerCommitSize)
				|| (state->currentPackets.imu6Position >= currentPacketContainerCommitSize));

		bool containerTimeCommit = containerGenerationIsCommitTimestampElapsed(&state->container,
			state->timestamps.wrapOverflow, state->timestamps.current);

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPackets.polarityPosition > 0) {
				containerGenerationSetPacket(&state->container, POLARITY_EVENT,
					(caerEventPacketHeader) state->currentPackets.polarity);

				state->currentPackets.polarity = NULL;
				state->currentPackets.polarityPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentPackets.specialPosition > 0) {
				containerGenerationSetPacket(&state->container, SPECIAL_EVENT,
					(caerEventPacketHeader) state->currentPackets.special);

				state->currentPackets.special = NULL;
				state->currentPackets.specialPosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentPackets.framePosition > 0) {
				containerGenerationSetPacket(&state->container, FRAME_EVENT,
					(caerEventPacketHeader) state->currentPackets.frame);

				state->currentPackets.frame = NULL;
				state->currentPackets.framePosition = 0;
				emptyContainerCommit = false;
			}

			if (state->currentPackets.imu6Position > 0) {
				containerGenerationSetPacket(&state->container, IMU6_EVENT,
					(caerEventPacketHeader) state->currentPackets.imu6);

				state->currentPackets.imu6 = NULL;
				state->currentPackets.imu6Position = 0;
				emptyContainerCommit = false;
			}

			if (tsReset || tsBigWrap) {
				// Ignore all APS and IMU6 (composite) events, until a new APS or IMU6
				// Start event comes in, for the next packet.
				// This is to correctly support the forced packet commits that a TS reset,
				// or a TS big wrap, impose. Continuing to parse events would result
				// in a corrupted state of the first event in the new packet, as it would
				// be incomplete, incorrect and miss vital initialization data.
				// See APS and IMU6 END states for more details on a related issue.
				state->aps.ignoreEvents = true;
				state->imu.ignoreEvents = true;
			}

			containerGenerationExecute(&state->container, emptyContainerCommit, tsReset, state->timestamps.wrapOverflow,
				state->timestamps.current, &state->dataExchange, &state->gpio.threadState, handle->info.deviceID,
				handle->info.deviceString, &state->deviceLogLevel);
		}
	}
}

#endif

bool davisRPiROIConfigure(caerDeviceHandle cdh, uint8_t roiRegion, bool enable, uint16_t startX, uint16_t startY,
	uint16_t endX, uint16_t endY) {
	davisRPiHandle handle = (davisRPiHandle) cdh;

	// First disable, then set all four coordinates, then enable again IF requested.
	spiConfigSend(&handle->state, DAVIS_CONFIG_APS, U8T(DAVIS_CONFIG_APS_ROI0_ENABLED + roiRegion), 0x00);

	switch (roiRegion) {
		case 0:
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, startX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, startY);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, endX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, endY);
			break;

		case 1:
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_1, startX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_1, startY);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_1, endX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_1, endY);
			break;

		case 2:
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_2, startX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_2, startY);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_2, endX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_2, endY);
			break;

		case 3:
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_3, startX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_3, startY);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_3, endX);
			spiConfigSend(&handle->state, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_3, endY);
			break;
	}

	if (enable) {
		spiConfigSend(&handle->state, DAVIS_CONFIG_APS, U8T(DAVIS_CONFIG_APS_ROI0_ENABLED + roiRegion), 0x01);
	}

	return (true);
}
