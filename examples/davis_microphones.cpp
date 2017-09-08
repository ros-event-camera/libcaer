#include <libcaercpp/devices/davis.hpp>
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

static void usbShutdownHandler(void *ptr) {
	(void)(ptr); // UNUSED.

	globalShutdown.store(true);
}

int main(void) {
	// Install signal handler for global shutdown.
#if defined(_WIN32)
	if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
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
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGTERM. Error: %d.", errno);
		return (EXIT_FAILURE);
	}

	if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
		libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
			"Failed to set signal handler for SIGINT. Error: %d.", errno);
		return (EXIT_FAILURE);
	}
#endif

	// Open a DAVIS FX3 (only ones to have microphones on some boards), give it
	// a device ID of 1, and don't care about USB bus or SN restrictions.
	libcaer::devices::davisfx3 davisHandle = libcaer::devices::davisfx3(1);

	// Let's take a look at the information we have on the device.
	struct caer_davis_info davis_info = davisHandle.infoGet();

	printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
		davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
		davis_info.logicVersion);

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	davisHandle.sendDefaultConfig();

	// Don't start all producers automatically, we only want to start microphones.
	davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS, false);

	// Start microphones and USB data transfer. Set sampling frequency to 48 KHz.
	davisHandle.configSet(DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_SAMPLE_FREQUENCY, 32);
	davisHandle.configSet(DAVIS_CONFIG_MICROPHONE, DAVIS_CONFIG_MICROPHONE_RUN, true);

	davisHandle.configSet(DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_RUN, true);
	davisHandle.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_RUN, true);
	davisHandle.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RUN, true);

	// Now let's get start getting some data from the device. We just loop in blocking mode,
	// no notification needed regarding new events. The shutdown notification, for example if
	// the device is disconnected, should be listened to.
	davisHandle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	std::vector<sf::Int16> samples;

	while (!globalShutdown.load(memory_order_relaxed)) {
		std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davisHandle.dataGet();
		if (packetContainer == nullptr) {
			continue; // Skip if nothing there.
		}

		std::shared_ptr<const libcaer::events::SampleEventPacket> samplePacket = static_pointer_cast<
			libcaer::events::SampleEventPacket>(packetContainer->findEventPacketByType(SAMPLE_EVENT));
		if (samplePacket == nullptr) {
			continue; // Skip if nothing there.
		}

		// Convert to 16 bit samples.
		int64_t meanValue = 0;
		int32_t samplesNumber = samplePacket->getEventValid();

		for (auto &sample : *samplePacket) {
			if (sample.isValid()) {
				int16_t value = static_cast<int16_t>(sample.getSample() >> 8);
				samples.push_back(value);
				meanValue += value;
			}
		}

		meanValue /= samplesNumber;

		printf("\nGot %d sound samples (mean value is %ld).\n", samplesNumber, meanValue);
	}

	davisHandle.dataStop();

	// Close automatically done by destructor.

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
