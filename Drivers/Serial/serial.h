/******************************************************************************
 * serial.h
 * Author: Isaac Travers | MARE | MIT License | 2026-01-14
 *
 * Public interface for an Arduino-style Serial library built on top of
 * STM32 HAL UART. Provides simple TX functions and non-blocking,
 * interrupt-driven RX with Arduino-compatible semantics.
 *
 * Additional functionality:
 * -------------------------
 * This module also maintains a secondary "monitor" buffer for each serial
 * port. The monitor buffer holds a mirrored copy of recently received UART
 * bytes so higher-level systems can inspect or forward serial output without
 * consuming the primary RX stream.
 *
 * This enables features such as:
 * - Remote serial consoles over UDP or other transports
 * - Debug logging of device communication
 * - Observing serial traffic while still allowing normal parsing logic
 *
 * Important:
 * - The monitor buffer is independent of the main RX buffer
 * - Reading from the monitor buffer does not affect Serial_read()
 * - Reading from Serial_read() does not affect the monitor buffer
 *
 *****************************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

/* main.h is included to access UART_HandleTypeDef and CubeMX-generated
 * HAL configuration used by this project. */
#include "main.h"

// The size of each serial buffer
#define SERIAL_RX_BUF_SIZE 64

/**
 * SerialPort
 *
 * Purpose:
 *   Holds all per-UART state required by the Serial library.
 *   Each UART gets its own SerialPort instance, allowing
 *   multiple independent serial connections.
 */
typedef struct
{
	/* Pointer to the UART instance used by this Serial Instance.
	 * This is set during Serial_begin() and reused by all Serial functions. */
    UART_HandleTypeDef *huart;

    /* RX ring buffer */
    uint8_t  rx_buf[SERIAL_RX_BUF_SIZE];

    /* Head index (write position) for the RX ring buffer.
     * Advanced by the interrupt handler. */
    uint16_t rx_head;

    /* Tail index (read position) for the RX ring buffer.
     * Advanced by Serial_read(). */
    uint16_t rx_tail;

    /* Temporary single-byte storage used by HAL_UART_Receive_IT().
     * HAL writes the received byte here before the RX-complete callback fires. */
    uint8_t  rx_byte;

    /* Monitor ring buffer used to expose recent serial output
     * without consuming the normal RX stream. */
    uint8_t  monitor_buf[SERIAL_RX_BUF_SIZE];

    /* Head index for the monitor ring buffer.
     * Advanced by the interrupt handler. */
    uint16_t monitor_head;

    /* Tail index for the monitor ring buffer.
     * Advanced by Serial_monitor_read(). */
    uint16_t monitor_tail;

} SerialPort;


/* Initialize the Serial library with a UART instance and optional
 * baud rate override (0 keeps CubeMX configuration). */
void Serial_begin(SerialPort *sp, UART_HandleTypeDef *huart, uint32_t baud);

/* Transmit a null-terminated string over the serial interface. */
void Serial_print(SerialPort *sp, char *s);

/* Transmit a string followed by CRLF, like Arduino Serial.println(). */
void Serial_println(SerialPort *sp, char *s);

/* Transmit a raw byte buffer over the serial interface. */
void Serial_write(SerialPort *sp, const uint8_t *data, uint16_t len);

/* Return the number of bytes currently available in the RX buffer. */
int  Serial_available(SerialPort *sp);

/* Read one byte from the RX buffer, or return -1 if no data is available. */
int  Serial_read(SerialPort *sp);

/* Return the number of bytes currently available in the monitor buffer. */
int  Serial_monitor_available(SerialPort *sp);

/* Read one byte from the monitor buffer, or return -1 if no data is available. */
int  Serial_monitor_read(SerialPort *sp);


#endif /* SERIAL_H */
