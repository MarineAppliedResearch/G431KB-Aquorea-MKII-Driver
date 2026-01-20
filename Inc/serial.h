/******************************************************************************
 * serial.h
 * Author: Isaac Travers | MARE | MIT License | 2026-01-14
 *
 * Public interface for an Arduino-style Serial library built on top of
 * STM32 HAL UART. Provides simple TX functions and non-blocking,
 * interrupt-driven RX with Arduino-compatible semantics.
 *****************************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

/* main.h is included to access UART_HandleTypeDef and CubeMX-generated
 * HAL configuration used by this project. */
#include "main.h"

/* Initialize the Serial library with a UART instance and optional
 * baud rate override (0 keeps CubeMX configuration). */
void Serial_begin(UART_HandleTypeDef *huart, uint32_t baud);


#endif /* SERIAL_H */
