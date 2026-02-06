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

/* ----------------------------------------------------------------------
 * Serial instance registry
 *
 * HAL provides only the UART handle in RX callbacks.
 * This table allows us to map a UART back to its SerialPort.
 * --------------------------------------------------------------------- */

#define MAX_SERIAL_PORTS 4

static SerialPort *serial_ports[MAX_SERIAL_PORTS];
static uint8_t serial_port_count = 0;

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
 *   sp    - An instance to a serial port struct
 *   huart - Pointer to an already-configured UART_HandleTypeDef
 *   baud  - Desired baud rate, or 0 to keep CubeMX-configured value
 *
 * Outputs:
 *   None
 */
void Serial_begin(SerialPort *sp, UART_HandleTypeDef *huart, uint32_t baud)
{
	/* Initialize instance state */
	sp->huart   = huart;
	sp->rx_head = 0;
	sp->rx_tail = 0;

    /* Check if a baud rate has been provided */
	if (baud > 0)
	{
		/* Update the UART configuration with the requested baud rate */
		sp->huart->Init.BaudRate = baud;

		/* Reinitialize the UART peripheral so the new baud takes effect */
		HAL_UART_Init(sp->huart);
	}

	 /* Register this SerialPort instance */
	 if (serial_port_count < MAX_SERIAL_PORTS)
	 {
		 serial_ports[serial_port_count++] = sp;
	 }

     /* Start interrupt-driven RX, receiving one byte at a time.
     * Each completed receive will trigger HAL_UART_RxCpltCallback(). */

    HAL_UART_Receive_IT(sp->huart, &sp->rx_byte, 1);
}


/**
 * Serial_print
 *
 * Purpose:
 *   Transmit a null-terminated string over the serial port.
 *
 * Inputs:
 *   SerialPort *sp - An instance of a serial port struct, that has already been initialized with _begin
 *   s - Pointer to a null-terminated C string
 *
 * Outputs:
 *   None
 */
void Serial_print(SerialPort *sp, char *s)
{
    /* Transmit the string using a blocking HAL call.
     * This mirrors Arduino Serial.print() behavior. */
    HAL_UART_Transmit(
        sp->huart,
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
 *   SerialPort *sp - An instance of a serial port struct, that has already been initialized with _begin
 *   s - Pointer to a null-terminated C string
 *
 * Outputs:
 *   None
 */
void Serial_println(SerialPort *sp, char *s)
{
    /* Print the string itself */
    Serial_print(sp, s);

    /* Append carriage return and newline for terminal-friendly output */
    Serial_print(sp, "\r\n");
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
 *   SerialPort *sp - An instance of a serial port struct, that has already been initialized with _begin
 *
 * Outputs:
 *   Number of bytes available to read
 */
int Serial_available(SerialPort *sp)
{
    /* If head has wrapped past tail, compute normally */
	if (sp->rx_head >= sp->rx_tail)
		return sp->rx_head - sp->rx_tail;
	else
        /* Handle wrap-around case */
	    return SERIAL_RX_BUF_SIZE - sp->rx_tail + sp->rx_head;
}


/**
 * Serial_read
 *
 * Purpose:
 *   Read a single byte from the RX buffer.
 *
 * Inputs:
 *   SerialPort *sp - An instance of a serial port struct, that has already been initialized with _begin
 *
 * Outputs:
 *   The next byte as an int (0-255), or -1 if no data is available
 */
int Serial_read(SerialPort *sp)
{
    /* If head equals tail, the buffer is empty */
    if (sp->rx_head == sp->rx_tail)
        return -1;

    /* Read the next byte from the buffer */
    uint8_t c = sp->rx_buf[sp->rx_tail];

    /* Advance the tail index, wrapping if necessary */
    sp->rx_tail = (sp->rx_tail + 1) % SERIAL_RX_BUF_SIZE;

    return c;
}



/* -------------------------------------------------------------------
 * Private Functions Here
 * ----------------------------------------------------------------- */

/* --------------------------------------------------------------------
 * Callback Functions Here
 * ------------------------------------------------------------------ */



/**
 * HAL_UART_RxCpltCallback
 *
 * Purpose:
 *   HAL callback invoked when a UART receive interrupt completes.
 *   Routes the received byte to the correct SerialPort instance.
 *
 * Inputs:
 *   huart - Pointer to the UART that triggered the interrupt
 *
 * Outputs:
 *   None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Loop through every Serial Port that has been registered
    for (uint8_t i = 0; i < serial_port_count; i++)
    {

        SerialPort *sp = serial_ports[i];

        // Check if this serial port is the one that contains the UART that has been triggered.
        if (sp->huart == huart)
        {
        	/* Compute the next head position in the ring buffer, wrap around */
            uint16_t next = (sp->rx_head + 1) % SERIAL_RX_BUF_SIZE;

            /* Only store the byte if the buffer is not full.
             * If next == rx_tail, the buffer is full and the byte is dropped. */
            if (next != sp->rx_tail)
            {
            	/* Store the received byte */
                sp->rx_buf[sp->rx_head] = sp->rx_byte;

                /* Advance the head index */
                sp->rx_head = next;
            }

            /* Re-arm the UART receive interrupt for the next byte.
             * This is critical: without this, only one byte would ever be received. */
            HAL_UART_Receive_IT(sp->huart, &sp->rx_byte, 1);
            return;
        }
    }
}

