/**
 * @file usb.h
 *
 * Common functions to access, configure and exchange data with
 * supported USB devices. Also contains defines for USB specific
 * configuration options.
 */

#ifndef LIBCAER_DEVICES_USB_H_
#define LIBCAER_DEVICES_USB_H_

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Module address: host-side USB configuration.
 */
#define CAER_HOST_CONFIG_USB -1

/**
 * Parameter address for module CAER_HOST_CONFIG_USB:
 * set number of buffers used by libusb for asynchronous data transfers
 * with the USB device. The default values are usually fine, only change
 * them if you're running into I/O limits.
 */
#define CAER_HOST_CONFIG_USB_BUFFER_NUMBER 0
/**
 * Parameter address for module CAER_HOST_CONFIG_USB:
 * set size of each buffer used by libusb for asynchronous data transfers
 * with the USB device. The default values are usually fine, only change
 * them if you're running into I/O limits.
 */
#define CAER_HOST_CONFIG_USB_BUFFER_SIZE   1

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_USB_H_ */
