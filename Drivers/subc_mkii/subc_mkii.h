/**************************************************************************
 * subc_mkii.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 * Date: 2026-01-22
 *
 * Purpose:
 * --------
 * Thin driver wrapper for the SubC Aquorea MkII light.
 *
 * This library sits between:
 *   - A host interface (currently USB serial)
 *   - The SubC Aquorea MkII light (RS-232 serial)
 *
 * Responsibilities:
 *   - Accept high-level light commands from the host
 *   - Translate them into the SubC MkII serial command format
 *   - Forward commands to the light
 *   - Collect any response bytes from the light
 *   - Package and forward those responses back to the host
 *
 * All progress is expected to occur via periodic polling from the main loop.
 *************************************************************************/

#ifndef SUBC_MKII_H
#define SUBC_MKII_H

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Includes
 * -------------------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include "serial.h"



/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */

/* Maximum number of bytes we will store for a single light response */
#define SUBC_MKII_RESPONSE_BUF_SIZE 256

/* Quiet time (in milliseconds) that indicates the end of a response.
 * This value will be tuned with real hardware testing. */
#define SUBC_MKII_RESPONSE_TIMEOUT_MS 10

/* --------------------------------------------------------------------------
 * Driver context
 * -------------------------------------------------------------------------- */

/**
 * SubcMkII
 *
 * Driver context for a single SubC Aquorea MkII light.
 * One instance corresponds to one physical device.
 */
typedef struct
{
    /* Serial interface connected to the light */
    SerialPort *light_serial;

    /* True while a command is in progress */
    bool command_active;

    /* True when a complete response has been received */
    bool response_ready;

    /* Raw response buffer */
    uint8_t response_buf[SUBC_MKII_RESPONSE_BUF_SIZE];
    uint16_t response_len;

    /* Timestamp of the most recent received byte (ms) */
    uint32_t last_rx_time_ms;

} SubcMkII;





/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * subc_mkii_init
 *
 * Initialize a SubcMkII driver instance.
 */
void subc_mkii_init(SubcMkII *driver, SerialPort *light_serial);


/**
 * subc_mkii_set_brightness
 * Send a brightness command to the SubC Aquorea MkII light.
 * percent: Desired brightness level (0–100).
 * Returns:
 *   true  - Command accepted and sent to the light
 *   false - Driver is busy processing another command
 */
bool subc_mkii_set_brightness(SubcMkII *driver, uint8_t percent);


/**
 * subc_mkii_poll
 *
 * Advance the driver by collecting any response data from the light
 * and determining when a response has completed.
 */
void subc_mkii_poll(SubcMkII *driver, uint32_t now_ms);


/**
 * subc_mkii_response_available
 *
 * Return true if a complete response from the light is available.
 */
bool subc_mkii_response_available(SubcMkII *driver);


/**
 * subc_mkii_read_response
 *
 * Retrieve the completed response from the light.
 * The response buffer is cleared after this call.
 */
bool subc_mkii_read_response(SubcMkII *driver,
                             uint8_t *out_buf,
                             uint16_t *out_len);


#ifdef __cplusplus
}
#endif

#endif /* SUBC_MKII_H */
