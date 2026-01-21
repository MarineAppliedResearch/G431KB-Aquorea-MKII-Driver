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

/* Transmit a null-terminated string over the serial interface. */
void Serial_print(char *s);

/* Transmit a string followed by CRLF, like Arduino Serial.println(). */
void Serial_println(char *s);

/* Return the number of bytes currently available in the RX buffer. */
int Serial_available(void);

/* Read one byte from the RX buffer, or return -1 if no data is available. */
int Serial_read(void);


#endif /* SERIAL_H */
