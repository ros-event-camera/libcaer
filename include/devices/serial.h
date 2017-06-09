/**
 * @file serial.h
 *
 * Common functions to access, configure and exchange data with
 * supported serial port devices. Also contains defines for serial
 * port specific configuration options.
 */

#ifndef LIBCAER_DEVICES_SERIAL_H_
#define LIBCAER_DEVICES_SERIAL_H_

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Module address: host-side serial port configuration.
 */
#define CAER_HOST_CONFIG_SERIAL -1

/**
 * Parameter address for module CAER_HOST_CONFIG_SERIAL:
 * baud-rate for serial port communication.
 */
#define CAER_HOST_CONFIG_SERIAL_BAUD_RATE 0
/**
 * Parameter address for module CAER_HOST_CONFIG_SERIAL:
 * read size for serial port communication.
 */
#define CAER_HOST_CONFIG_SERIAL_READ_SIZE 1

#ifdef __cplusplus
}
#endif

#endif /* LIBCAER_DEVICES_SERIAL_H_ */
