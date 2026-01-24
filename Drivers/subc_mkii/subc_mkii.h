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

/* How many milliseconds the driver waits between querying the temp. */
#define SUBC_MKII_TEMP_REQUEST_INTERVAL_MS 5000


/* --------------------------------------------------------------------------
 * Driver context
 * -------------------------------------------------------------------------- */

/**
 * SubcCommandState
 * Tracks whether an exclusive command is currently active.
 * Streaming commands do not affect this state and may be issued at any time.
 * Exclusive commands are rejected while another exclusive command is active.
 */
typedef enum
{
    /* No exclusive command is in progress */
    SUBC_CMD_IDLE = 0,

    /* An exclusive command is active and collecting a response */
    SUBC_CMD_EXCLUSIVE_ACTIVE

} SubcCommandState;


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

    /* Current exclusive command state */
    SubcCommandState cmd_state;

    /* True when a complete response has been received */
    bool response_ready;

    /* Raw response buffer */
    uint8_t response_buf[SUBC_MKII_RESPONSE_BUF_SIZE];
    uint16_t response_len;

    /* Timestamp of the most recent received byte (ms) */
    uint32_t last_rx_time_ms;

    /* ------------------------------------------------------------------
	 * Temperature tracking
	 * ------------------------------------------------------------------ */

	/* Most recently received temperature (degrees C) */
	float temp_current;

	/* Lowest temperature observed since init */
	float temp_min;

	/* Highest temperature observed since init */
	float temp_max;

	/* Running sum of temperatures for averaging */
	float temp_sum;

	/* Number of temperature samples collected */
	uint32_t temp_samples;

	/* Timestamp of last temperature request */
	uint32_t last_temp_request_ms;

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


/**
 * subc_mkii_get_temperature
 *
 * Return the most recently recorded temperature sample.
 *
 * Notes:
 *   This does not talk to the light. It only returns the cached value that
 *   was last parsed from a temperature response.
 *
 * Inputs:
 *   driver   - Driver instance
 *   out_temp - Output pointer to receive the temperature (degrees C)
 *
 * Outputs:
 *   true  if a temperature sample is available and was written to out_temp
 *   false if driver/out_temp is invalid or no samples have been recorded yet
 */
bool subc_mkii_get_temperature(SubcMkII *driver, float *out_temp);


/**
 * subc_mkii_get_temperature_stats
 *
 * Return temperature statistics collected since driver initialization.
 *
 * Notes:
 *   Any output pointer may be NULL if the caller does not need that value.
 *   Average is computed from the running sum and sample count.
 *
 * Inputs:
 *   driver  - Driver instance
 *   out_min - Output pointer for minimum observed temperature (optional)
 *   out_max - Output pointer for maximum observed temperature (optional)
 *   out_avg - Output pointer for average temperature (optional)
 *
 * Outputs:
 *   true  if at least one sample exists and requested outputs were written
 *   false if driver is invalid or no samples have been recorded yet
 */
bool subc_mkii_get_temperature_stats(SubcMkII *driver,
                                     float *out_min,
                                     float *out_max,
                                     float *out_avg);


#ifdef __cplusplus
}
#endif

#endif /* SUBC_MKII_H */
