#include "devices/device_discover.h"

#include "dvs128.h"
#include "davis.h"
#include "davis_rpi.h"
#include "dynapse.h"

#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
#include "edvs.h"
#else
#include "devices/edvs.h"
#endif

// Supported devices and their functions.
static ssize_t (*deviceFinders[CAER_SUPPORTED_DEVICES_NUMBER])(caerDeviceDiscoveryResult *discoveredDevices) = {
	[CAER_DEVICE_DVS128] = &dvs128Find,
	[CAER_DEVICE_DAVIS_FX2] = &davisFindFX2,
	[CAER_DEVICE_DAVIS_FX3] = &davisFindFX3,
	[CAER_DEVICE_DYNAPSE] = &dynapseFind,
	[CAER_DEVICE_DAVIS] = &davisFindAll,
#if defined(LIBCAER_HAVE_SERIALDEV) && LIBCAER_HAVE_SERIALDEV == 1
	[CAER_DEVICE_EDVS] = &edvsFind,
#else
	[CAER_DEVICE_EDVS] = NULL,
#endif
#if defined(OS_LINUX)
	[CAER_DEVICE_DAVIS_RPI] = &davisRPiFind,
#else
	[CAER_DEVICE_DAVIS_RPI] = NULL,
#endif
};

ssize_t caerDeviceDiscover(int16_t deviceType, caerDeviceDiscoveryResult *discoveredDevices) {
	if (discoveredDevices == NULL) {
		// Usage error, don't pass a NULL pointer!
		return (-1);
	}

	// Set to NULL initially (for error return).
	*discoveredDevices = NULL;

	// Check if device type is supported.
	if ((deviceType < CAER_DEVICE_DISCOVER_ALL) || (deviceType >= CAER_SUPPORTED_DEVICES_NUMBER)) {
		return (-1);
	}

	if (deviceType == CAER_DEVICE_DISCOVER_ALL) {
		// Go through all device finder functions that are defined.
		size_t foundDevices = 0;

		for (size_t i = 0; i < CAER_SUPPORTED_DEVICES_NUMBER; i++) {
			// Skip CAER_DEVICE_DAVIS: already considered by the specific
			// FX2 and FX3 DAVIS device searche cases.
			if (i == CAER_DEVICE_DAVIS) {
				continue;
			}

			if (deviceFinders[i] == NULL) {
				// Device type not supported on this system, skip it.
				continue;
			}

			caerDeviceDiscoveryResult discovered;
			ssize_t result = deviceFinders[i](&discovered);

			// Search error!
			if (result < 0) {
				caerLog(CAER_LOG_CRITICAL, "DeviceDiscovery", "All-device-discovery failed for device type %zu.", i);

				continue;
			}

			// No devices of this type found.
			if (result == 0) {
				continue;
			}

			// Found some devices.
			void *biggerDiscoveredDevices = realloc(*discoveredDevices,
				(foundDevices + (size_t) result) * sizeof(struct caer_device_discovery_result));
			if (biggerDiscoveredDevices == NULL) {
				// Memory allocation failure!
				free(*discoveredDevices);
				*discoveredDevices = NULL;

				return (-1);
			}

			// Memory allocation successful, get info.
			*discoveredDevices = biggerDiscoveredDevices;

			memcpy(&(*discoveredDevices)[foundDevices], discovered,
				(size_t) result * sizeof(struct caer_device_discovery_result));

			free(discovered);

			foundDevices += (size_t) result;
		}

		return ((ssize_t) foundDevices);
	}
	else {
		// Execute finder function for a specific device type.
		if (deviceFinders[deviceType] == NULL) {
			return (-1);
		}

		return (deviceFinders[deviceType](discoveredDevices));
	}
}

caerDeviceHandle caerDeviceDiscoverOpen(uint16_t deviceID, caerDeviceDiscoveryResult discoveredDevice) {
	return (NULL );
}
