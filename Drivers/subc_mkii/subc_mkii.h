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
#define SUBC_MKII_RESPONSE_BUF_SIZE 512

/* How many milliseconds the driver waits between querying the temp. */
#define SUBC_MKII_TEMP_REQUEST_INTERVAL_MS 10000

// Preferred quiet gap before we attempt parsing.
#define SUBC_MKII_SILENCE_TIMEOUT_MS   5

// Absolute maximum time we allow a buffer to accumulate before we force
// one parse attempt and clear it, even if the light never goes silent.
#define SUBC_MKII_MAX_WINDOW_MS       50


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

    // Optional debug output
    SerialPort *debug_serial;

    /* Current exclusive command state */
    SubcCommandState cmd_state;

    /* True when a complete response has been received */
    bool response_ready;

    /* Raw response buffer */
    uint8_t response_buf[SUBC_MKII_RESPONSE_BUF_SIZE];
    uint16_t response_len;

    /* Timestamp of the most recent received byte (ms) */
    uint32_t first_rx_time_ms;   // when current accumulation started
    uint32_t last_rx_time_ms;    // when last byte was received

    /* ------------------------------------------------------------------
	 * Environment tracking
	 * ------------------------------------------------------------------ */

	/* Most recently received temperature (degrees C) */
	int32_t temp_current;

	/* Lowest temperature observed since init */
	int32_t temp_min;

	/* Highest temperature observed since init */
	int32_t temp_max;

	/* Running sum of temperatures for averaging, int64_t allows running for years without reset */
	int64_t temp_sum;

	/* Number of temperature samples collected */
	uint32_t temp_samples;

	/* Timestamp of last temperature request */
	uint32_t last_temp_request_ms;

	/* Most recently received temperature (degrees C) */
	int32_t humidity_current;

	/* Lowest temperature observed since init */
	int32_t humidity_min;

	/* Highest temperature observed since init */
	int32_t humidity_max;

	/* Running sum of temperatures for averaging, int64_t allows running for years without reset */
	int64_t humidity_sum;

	/* Number of temperature samples collected */
	uint32_t humidity_samples;

	/* Most recently reported LED current (centi-percent) */
	int32_t led_current_centi;

	/* Most recently reported LED power PU (centi-percent) */
	int32_t led_power_centi;

	/* Most recently reported single-signal-enabled state (0 or 1) */
	uint8_t signal_enabled;

	/* ------------------------------------------------------------------
	 * Uptime tracking
	 * ------------------------------------------------------------------ */

	/* Timestamp of the first poll call (ms) */
	uint32_t start_time_ms;

	/* Most recent uptime value (ms) */
	uint32_t uptime_ms;

    // ------------------------------------------------------------------
    // Device lifetime (reported by light firmware)
    // ------------------------------------------------------------------

    // Total runtime reported by the light, in seconds
    uint32_t device_runtime_seconds;

    // Flag indicating whether runtime has ever been parsed successfully
    bool device_runtime_valid;


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
bool subc_mkii_get_temperature(SubcMkII *driver, uint32_t *out_temp);


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
									 uint32_t *out_min,
									 uint32_t *out_max,
									 uint32_t *out_avg,
									 uint32_t *out_current);


/**
 * subc_mkii_get_uptime
 *
 * Return the total uptime of the driver since initialization.
 *
 * Notes:
 *   Uptime is tracked internally using timestamps provided to
 *   subc_mkii_poll() and represents elapsed milliseconds since
 *   the driver became active.
 *
 * Inputs:
 *   driver        - Driver instance
 *   out_uptime_ms - Output pointer to receive uptime in milliseconds
 *
 * Outputs:
 *   true  if the uptime value was written successfully
 *   false if driver or output pointer is invalid
 */
bool subc_mkii_get_uptime(SubcMkII *driver, uint32_t *out_uptime_ms);


/**
 * subc_mkii_get_device_runtime
 *
 * Retrieve the device lifetime reported by the light firmware.
 *
 * Inputs:
 *   driver       - Initialized SubcMkII driver instance
 *   out_seconds  - Pointer to receive total runtime in seconds
 *
 * Outputs:
 *   true  if a runtime value has been parsed
 *   false if no runtime has been received yet
 */
bool subc_mkii_get_device_runtime(SubcMkII *driver,
                                  uint32_t *out_seconds);



/**
 * subc_mkii_get_environment_stats_formatted
 *
 * Purpose:
 *   Format all available environmental statistics (temperature and humidity)
 *   into a human-readable string suitable for serial output.
 *
 * Inputs:
 *   driver  - Initialized SubcMkII driver instance
 *   buf     - Output buffer to write formatted text into
 *   buf_len - Size of the output buffer in bytes
 *
 * Outputs:
 *   Returns true if any environmental data was available and formatted.
 *   Returns false if no environmental data has been collected yet.
 *
 * Preconditions:
 *   - driver must be initialized
 *   - buf must point to valid writable memory
 *
 * Postconditions:
 *   - buf contains a null-terminated string on success
 */
bool subc_mkii_get_stats_formatted(SubcMkII *driver,
                                               char *buf,
                                               size_t buf_len);



#ifdef __cplusplus
}
#endif

#endif /* SUBC_MKII_H */
