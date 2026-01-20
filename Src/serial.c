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

/* ---------------------------------------------------------------------
 * State Variables/Objects Here
 * ------------------------------------------------------------------- */

/* Pointer to the UART instance used by this Serial library.
 * This is set during Serial_begin() and reused by all Serial functions. */
static UART_HandleTypeDef *serialUart;

/* Temporary single-byte storage used by HAL_UART_Receive_IT().
 * HAL writes the received byte here before the RX-complete callback fires. */
static uint8_t rx_byte;

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

/* -------------------------------------------------------------------
 * Private Functions Here
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * Callback Functions Here
 * ------------------------------------------------------------------ */

