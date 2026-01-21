/**************************************************************************
 * serial.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 * Date: 2026-01-14
 *
 * Purpose:
 * --------
 * Arduino-style Serial library implemented on top of STM32 HAL UART.
 * Provides interrupt-driven RX, ring-buffered input, and simple
 * Serial_begin / Serial_print / Serial_available style APIs.
 *************************************************************************/

/* serial.h provides the public API for this module as well as
 * access to the UART_HandleTypeDef type used by HAL. */
#include "serial.h"

/* string.h is required for strlen(), which is used when transmitting
 * null-terminated C strings over UART. */
#include <string.h>

/* Size of the receive ring buffer.
 * Incoming characters are stored here by the UART RX interrupt. */
#define SERIAL_RX_BUF_SIZE 64

/* ---------------------------------------------------------------------
 * State Variables/Objects Here
 * ------------------------------------------------------------------- */

/* Pointer to the UART instance used by this Serial library.
 * This is set during Serial_begin() and reused by all Serial functions. */
static UART_HandleTypeDef *serialUart;

/* Temporary single-byte storage used by HAL_UART_Receive_IT().
 * HAL writes the received byte here before the RX-complete callback fires. */
static uint8_t rx_byte;

/* Head index (write position) for the RX ring buffer.
 * Advanced by the interrupt handler. */
static volatile uint16_t rx_head = 0;

/* Tail index (read position) for the RX ring buffer.
 * Advanced by Serial_read(). */
static volatile uint16_t rx_tail = 0;

/* Ring buffer storage for received characters. */
static uint8_t rx_buf[SERIAL_RX_BUF_SIZE];

/* -------------------------------------------------------------------
 * Public Functions Here
 * ----------------------------------------------------------------- */

/**
 * Serial_begin
 *
 * Purpose:
 *   Initialize the Serial library with a HAL UART instance and optional
 *   baud rate override.
 *
 * Inputs:
 *   huart - Pointer to an already-configured UART_HandleTypeDef
 *   baud  - Desired baud rate, or 0 to keep CubeMX-configured value
 *
 * Outputs:
 *   None
 */
void Serial_begin(UART_HandleTypeDef *huart, uint32_t baud)
{
    /* Store the UART instance for later use by the library */
    serialUart = huart;

    /* Check if a baud rate has been provided */
    if (baud > 0)
    {
        /* Update the UART configuration with the requested baud rate */
        serialUart->Init.BaudRate = baud;

        /* Reinitialize the UART peripheral so the new baud takes effect */
        HAL_UART_Init(serialUart);
    }

    /* Start interrupt-driven RX, receiving one byte at a time.
     * Each completed receive will trigger HAL_UART_RxCpltCallback(). */
    HAL_UART_Receive_IT(serialUart, &rx_byte, 1);
}


/**
 * Serial_print
 *
 * Purpose:
 *   Transmit a null-terminated string over the serial port.
 *
 * Inputs:
 *   s - Pointer to a null-terminated C string
 *
 * Outputs:
 *   None
 */
void Serial_print(char *s)
{
    /* Transmit the string using a blocking HAL call.
     * This mirrors Arduino Serial.print() behavior. */
    HAL_UART_Transmit(
        serialUart,
        (uint8_t *)s,
        strlen(s),
        HAL_MAX_DELAY
    );
}


/**
 * Serial_println
 *
 * Purpose:
 *   Transmit a string followed by CRLF, similar to Arduino Serial.println().
 *
 * Inputs:
 *   s - Pointer to a null-terminated C string
 *
 * Outputs:
 *   None
 */
void Serial_println(char *s)
{
    /* Print the string itself */
    Serial_print(s);

    /* Append carriage return and newline for terminal-friendly output */
    Serial_print("\r\n");
}


/* --------------------------------------------------------------------------
 * RX buffer accessors
 * -------------------------------------------------------------------------- */

/**
 * Serial_available
 *
 * Purpose:
 *   Return the number of unread bytes currently available in the RX buffer.
 *
 * Inputs:
 *   None
 *
 * Outputs:
 *   Number of bytes available to read
 */
int Serial_available(void)
{
    /* If head has wrapped past tail, compute normally */
    if (rx_head >= rx_tail)
        return rx_head - rx_tail;
    else
        /* Handle wrap-around case */
        return SERIAL_RX_BUF_SIZE - rx_tail + rx_head;
}


/**
 * Serial_read
 *
 * Purpose:
 *   Read a single byte from the RX buffer.
 *
 * Inputs:
 *   None
 *
 * Outputs:
 *   The next byte as an int (0-255), or -1 if no data is available
 */
int Serial_read(void)
{

}


/* -------------------------------------------------------------------
 * Private Functions Here
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * HAL callback
 * -------------------------------------------------------------------------- */

/**
 * HAL_UART_RxCpltCallback
 *
 * Purpose:
 *   HAL callback invoked when a UART receive interrupt completes.
 *   This function stores the received byte into the RX ring buffer
 *   and immediately re-arms the UART for the next byte.
 *
 * Inputs:
 *   huart - Pointer to the UART that triggered the interrupt
 *
 * Outputs:
 *   None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Ignore RX events from UARTs not owned by this Serial instance */
    if (huart != serialUart)
        return;

    /* Compute the next head position in the ring buffer, wrap around if too big */
    uint16_t next = (rx_head + 1) % SERIAL_RX_BUF_SIZE;

    /* Only store the byte if the buffer is not full.
     * If next == rx_tail, the buffer is full and the byte is dropped. */
    if (next != rx_tail)
    {
        /* Store the received byte */
        rx_buf[rx_head] = rx_byte;

        /* Advance the head index */
        rx_head = next;
    }

    /* Re-arm the UART receive interrupt for the next byte.
     * This is critical: without this, only one byte would ever be received. */
    HAL_UART_Receive_IT(serialUart, &rx_byte, 1);
}

/* --------------------------------------------------------------------
 * Callback Functions Here
 * ------------------------------------------------------------------ */

