#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>
#include <cstdio>
#include <csignal>
#include <atomic>
#include <SFML/Audio.hpp>

using namespace std;

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
	// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
	if (signal == SIGTERM || signal == SIGINT) {
		globalShutdown.store(true);
	}
}

int main(void) {
	// Install signal handler for global shutdown.
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#else
	struct sigaction shutdownAction;

	shutdownAction.sa_handler = &globalShutdownSignalHandler;
	shutdownAction.sa_flags = 0;
	sigemptyset(&shutdownAction.sa_mask);
	sigaddset(&shutdownAction.sa_mask, SIGTERM);
	sigaddset(&shutdownAction.sa_mask, SIGINT);

	if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		caerLog(CAER_LOG_CRITICAL, "ShutdownAction", "Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
	caerDeviceHandle davis_handle = caerDeviceOpen(1, CAER_DEVICE_DAVIS_FX3, 0, 0, NULL);
	if (davis_handle == NULL) {
		return (EXIT_FAILURE);
	}

	// Let's take a look at the information we have on the device.
	struct caer_davis_info davis_info = caerDavisInfoGet(davis_handle);

	printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
		davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
		davis_info.logicVersion);

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	caerDeviceSendDefaultConfig(davis_handle);

	// Don't start all producers automatically, we only want to start microphones.
	caerDeviceConfigSet(davis_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS,
		false);

	// Start microphones and USB data transfer.
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY, 32);
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_RUN, true);

	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, true);
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, true);
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, true);

	// Now let's get start getting some data from the device. We just loop, no notification needed.
	caerDeviceDataStart(davis_handle, NULL, NULL, NULL, NULL, NULL);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	caerDeviceConfigSet(davis_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	std::vector<sf::Int16> samples;

	while (!globalShutdown.load(memory_order_relaxed)) {
		caerEventPacketContainer packetContainer = caerDeviceDataGet(davis_handle);
		if (packetContainer == NULL) {
			continue; // Skip if nothing there.
		}

		caerSampleEventPacket samplePacket = (caerSampleEventPacket) caerEventPacketContainerFindEventPacketByType(
			packetContainer, SAMPLE_EVENT);
		if (samplePacket == NULL) {
			continue; // Skip if nothing there.
		}

		// Convert to 16 bit samples.
		int64_t meanValue = 0;
		int32_t samplesNumber = caerEventPacketHeaderGetEventValid(&samplePacket->packetHeader);

		CAER_SAMPLE_ITERATOR_VALID_START(samplePacket)
			int16_t value = caerSampleEventGetSample(caerSampleIteratorElement) >> 8;
			samples.push_back(value);
			meanValue += value;
		CAER_SAMPLE_ITERATOR_VALID_END

		meanValue /= samplesNumber;

		printf("\nGot %d sound samples (mean value is %ld).\n", samplesNumber, meanValue);

		caerEventPacketContainerFree(packetContainer);
	}

	caerDeviceDataStop(davis_handle);

	caerDeviceClose(&davis_handle);

	sf::SoundBuffer buffer;
	buffer.loadFromSamples(&samples[0], samples.size(), 2, 48000);

	// Playback current samples.
	sf::Sound sound;
	sound.setBuffer(buffer);
	sound.play();
	while (sound.getStatus() != sf::Sound::Status::Stopped) {
		;
	}

	printf("Shutdown successful.\n");

	return (EXIT_SUCCESS);
}
