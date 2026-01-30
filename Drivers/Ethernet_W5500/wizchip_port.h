/**************************************************************************
 * wizchip_port.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * Low-level hardware port for the WIZnet W5500 Ethernet controller.
 *
 * This module binds the WIZnet ioLibrary to STM32 HAL SPI and GPIO.
 * It provides chip select control, reset control, and SPI read/write
 * primitives required by the WIZnet stack.
 *************************************************************************/

#ifndef WIZCHIP_PORT_H
#define WIZCHIP_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>






/**
 * wizchip_log_fn
 *
 * Optional debug logging callback.
 * If registered, the driver emits high-level status messages.
 */
typedef void (*wizchip_log_fn)(const char *msg);


/**
 * wizchip_port_init
 *
 * Initialize low-level W5500 hardware access.
 *
 * Registers SPI and chip-select callbacks with the WIZnet ioLibrary,
 * performs a hardware reset, and verifies basic communication.
 *
 * Notes:
 *   Must be called after GPIO and SPI peripherals are initialized.
 *
 * Outputs:
 *   true  if the W5500 responds correctly
 *   false if communication or version check fails
 */
bool wizchip_port_init(void);


/**
 * wizchip_port_reset
 *
 * Perform a hardware reset of the W5500 using the reset pin.
 */
void wizchip_port_reset(void);


/**
 * wizchip_port_set_logger
 *
 * Register a debug logging callback.
 *
 * Notes:
 *   Passing NULL disables logging.
 */
void wizchip_port_set_logger(wizchip_log_fn fn);


#ifdef __cplusplus
}
#endif

#endif /* WIZCHIP_PORT_H */
