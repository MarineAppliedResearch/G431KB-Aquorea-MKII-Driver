/**************************************************************************
 * subc_mkii.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 * Date: 2026-01-22
 *
 * Purpose:
 * --------
 * Implementation of the SubC Aquorea MkII light driver.
 *
 * This module provides the concrete behavior behind the public
 * interface defined in subc_mkii.h.
 *************************************************************************/

/* --------------------------------------------------------------------------
 * Includes
 * -------------------------------------------------------------------------- */

#include "subc_mkii.h"
#include <stdio.h>
#include <string.h>



extern SerialPort SerialUSB;

/* --------------------------------------------------------------------------
 * Public Functions
 * -------------------------------------------------------------------------- */


/**
 * subc_mkii_init
 *
 * Initialize a SubcMkII driver instance.
 */
void subc_mkii_init(SubcMkII *driver, SerialPort *light_serial)
{
    if (!driver)
        return;

    // Init the debug_serial to null, user will manually set it after init if needed
    driver->debug_serial = NULL;

    // Uptime tracking starts on first poll
    driver->start_time_ms = 0;
    driver->uptime_ms     = 0;
    driver->last_temp_request_ms = 0;


    /* Associate this driver instance with the light's serial interface */
    driver->light_serial = light_serial;

    /* No exclusive command active at startup */
    driver->cmd_state = SUBC_CMD_IDLE;

    /* Clear response tracking state */
    driver->response_len   = 0;
    driver->response_ready = false;
    driver->last_rx_time_ms = 0;

    // Initialize LED-related state to known defaults
    // These values represent "unknown / not yet reported" at startup
    driver->led_current_centi = 0;
    driver->led_power_centi  = 0;

    // Initialize signal-enabled state explicitly
    driver->signal_enabled = 0xFF; // unknown until reported

    // Initialize device lifetime tracking
    driver->device_runtime_seconds = 0;
    driver->device_runtime_valid   = false;


}


/**
 * subc_mkii_set_brightness
 *
 * Set the light brightness as a percentage.
 *
 * This is a streaming command:
 * - It does not wait for a response
 * - It does not block other commands
 * - It may be sent repeatedly at high rate
 */
bool subc_mkii_set_brightness(SubcMkII *driver, uint8_t percent)
{
    char cmd[8];  /* "$Lb" + up to 3 digits + null */

    if (!driver || !driver->light_serial)
        return false;

    /* Clamp percent to valid range */
    if (percent > 100)
        percent = 100;

    /*
     * Format light command.
     * Protocol requires:
     *   $     start character
     *   Lb    brightness opcode
     *   ###   decimal parameter (no terminator)
     */
    snprintf(cmd, sizeof(cmd), "$Lb%u", percent);

    /* Send command to the light immediately */
    Serial_print(driver->light_serial, cmd);

    return true;
}



/**
 * subc_mkii_poll
 *
 * Advance the internal state of the SubC MkII driver.
 *
 * This function accumulates serial output from the light, waits a bounded
 * amount of time for the light to produce meaningful output, then attempts
 * to parse the accumulated data exactly once before clearing the buffer.
 */
void subc_mkii_poll(SubcMkII *driver, uint32_t now_ms)
{
    // Ensure the driver and serial interface are valid before proceeding.
    if (!driver || !driver->light_serial)
        return;

    // Initialize uptime reference on first poll
        if (driver->start_time_ms == 0)
        {
            driver->start_time_ms = now_ms;
            driver->uptime_ms     = 0;
        }
        else
        {
            driver->uptime_ms = now_ms - driver->start_time_ms;
        }

    // Check whether enough time has elapsed since the last temperature request.
    // This enforces the background telemetry polling interval.
    if ((now_ms - driver->last_temp_request_ms) >=
        SUBC_MKII_TEMP_REQUEST_INTERVAL_MS)
    {
        // Send a new temperature request to the light.
        // The response will be received asynchronously and handled
        // by the RX and parsing logic later.
        subc_mkii_request_temperature_internal(driver);

        // Record when this temperature request was sent so we
        // do not send another one too soon.
        driver->last_temp_request_ms = now_ms;
    }

    // Always drain all available serial data into the response buffer.
    // This must run on every call to poll so data is not artificially delayed.
    while (Serial_available(driver->light_serial) > 0)
    {
        // Read a single byte from the serial interface.
        int c = Serial_read(driver->light_serial);

        // Abort the drain loop if the serial driver reports an error.
        if (c < 0)
            break;

        // If this is the first byte of a new accumulation window,
        // record the start time so we can enforce a maximum window length.
        if (driver->response_len == 0)
            driver->first_rx_time_ms = now_ms;

        // Append the received byte to the response buffer if space remains.
        // We always leave room for a null terminator at the end.
        if (driver->response_len < (SUBC_MKII_RESPONSE_BUF_SIZE - 1)){
        	driver->response_buf[driver->response_len++] = (uint8_t)c;
        }


        // Update the timestamp of the most recently received byte.
        // This is used to detect short periods of silence.
        driver->last_rx_time_ms = now_ms;
    }

    // If there is no accumulated data, there is nothing to parse yet.
    if (driver->response_len == 0)
        return;

    // Check whether the light has been quiet long enough to consider the
    // current output complete.
    bool silence_elapsed =
        (now_ms - driver->last_rx_time_ms) >= SUBC_MKII_SILENCE_TIMEOUT_MS;

    // Check whether we have been accumulating data for too long overall.
    // This prevents the buffer from growing indefinitely if the light
    // never goes fully silent.
    bool window_expired =
        (now_ms - driver->first_rx_time_ms) >= SUBC_MKII_MAX_WINDOW_MS;

    // If neither the silence condition nor the max window condition
    // has been met, continue accumulating data.
    if (!silence_elapsed && !window_expired)
        return;

    // At this point, we have waited long enough to make a decision.
    // Attempt to parse whatever data we have exactly once.
    driver->response_buf[driver->response_len] = '\0';

    // Try parsing any data that might be available in the response buffer
    // This function is expected to fail gracefully if parsing is not possible.
    subc_mkii_parse_response(driver);

    // Clear the response buffer so the next output from the light
    // starts from a clean state.
    driver->response_len = 0;
    driver->response_buf[0] = '\0';
}


/**
 * subc_mkii_poll
 *
 * Advance the internal state of the SubC MkII driver.
 */
/*
void subc_mkii_poll(SubcMkII *driver, uint32_t now_ms)
{
	// Make sure the driver, and serial connection are properly initialized.
    if (!driver || !driver->light_serial)
        return;

    static uint32_t last_poll_ms = UINT32_MAX;

    if (now_ms == last_poll_ms)
        return;

    last_poll_ms = now_ms;

    // Initialize uptime reference on first poll
    if (driver->start_time_ms == 0)
    {
        driver->start_time_ms = now_ms;
        driver->uptime_ms     = 0;
    }
    else
    {
        driver->uptime_ms = now_ms - driver->start_time_ms;
    }

    // Background telemetry
    ////subc_mkii_request_temperature_internal(driver, now_ms); // update the temp

    // Drain all available bytes from the light serial port
    while (Serial_available(driver->light_serial) > 0)
    {
    	// Read a byte from serial
        int c = Serial_read(driver->light_serial);

        // If we didn't read a byte, exit the loop
        if (c < 0)
            break;

        ////Serial_println(&SerialUSB, (uint8_t)c);

        // If we have not filled up our driver buffer, copy that byte into it, drop otherwise.
        if (driver->response_len < SUBC_MKII_RESPONSE_BUF_SIZE)
        {
            driver->response_buf[driver->response_len++] = (uint8_t)c;
        }

        // Update timestamp whenever we receive data
        driver->last_rx_time_ms = now_ms;
    }



    // If we have collected any bytes, and we have been quiet long enough,
     //  treat the current buffer as a complete burst we can scan
    if (driver->response_len > 0 &&
        (now_ms - driver->last_rx_time_ms) >= SUBC_MKII_RESPONSE_TIMEOUT_MS)
    {
        // Null terminate so we can safely treat it as a string
        if (driver->response_len < SUBC_MKII_RESPONSE_BUF_SIZE)
        {
            driver->response_buf[driver->response_len] = '\0';
        }
        else
        {
            driver->response_buf[SUBC_MKII_RESPONSE_BUF_SIZE - 1] = '\0';
        }

        // Opportunistic parse. If the burst contains Temp, we will update stats
        subc_mkii_parse_temperature(driver);

        // Mark that a burst is ready for the host to read if desired
        // consume burst
            driver->response_len = 0;
            driver->response_buf[0] = '\0';
            driver->response_ready = false;
    }
}
*/


/* Tells us if a response is available in the reponse buffer. */
bool subc_mkii_response_available(SubcMkII *driver)
{
    if (!driver)
        return false;

    return driver->response_ready;
}


/* Returns the response in the response buffer, and clears the buffer. */
bool subc_mkii_read_response(SubcMkII *driver,
                             uint8_t *out_buf,
                             uint16_t *out_len)
{
    // Check to make sure the driver has been initialized.
    // and make sure a response is ready.
    if (!driver || !driver->response_ready)
        return false;


    // Copy  the completed response out of the driver
    // and into memory provided by the caller.
    if (out_buf && out_len)
    {
        for (uint16_t i = 0; i < driver->response_len; i++)
        {
            out_buf[i] = driver->response_buf[i];
        }

        *out_len = driver->response_len;
    }

    /* Clear response state */
    driver->response_len   = 0;
    driver->response_ready = false;

    return true;
}


/**
 * subc_mkii_get_temperature
 *
 * Get the most recently recorded temperature.
 */

bool subc_mkii_get_temperature(SubcMkII *driver, uint32_t *out_temp)
{
    if (!driver || !out_temp || driver->temp_samples == 0)
        return false;

    *out_temp = driver->temp_current;
    return true;
}



/**
 * subc_mkii_get_temperature_stats
 *
 * Get temperature statistics collected since driver initialization.
 */

bool subc_mkii_get_temperature_stats(SubcMkII *driver,
                                     uint32_t *out_min,
									 uint32_t *out_max,
									 uint32_t *out_avg,
									 uint32_t *out_current)
{
    if (!driver || driver->temp_samples == 0)
        return false;

    if (out_min)
        *out_min = driver->temp_min;

    if (out_max)
        *out_max = driver->temp_max;

    if (out_avg)
        *out_avg = (int32_t)(driver->temp_sum / driver->temp_samples);

    if(out_current)
    	*out_current = driver->temp_current;

    return true;
}


/**
 * subc_mkii_get_temperature_stats
 *
 * Get temperature statistics collected since driver initialization.
 */

bool subc_mkii_get_humidity_stats(SubcMkII *driver,
                                     uint32_t *out_min,
									 uint32_t *out_max,
									 uint32_t *out_avg,
									 uint32_t *out_current)
{
    if (!driver || driver->humidity_samples == 0)
        return false;

    if (out_min)
        *out_min = driver->humidity_min;

    if (out_max)
        *out_max = driver->humidity_max;

    if (out_avg)
        *out_avg = (int32_t)(driver->humidity_sum / driver->humidity_samples);

    if(out_current)
    	*out_current = driver->humidity_current;

    return true;
}



/* --------------------------------------------------------------------------
 * Private Functions
 * -------------------------------------------------------------------------- */

/**
 * subc_mkii_request_temperature_internal
 *
 * Internal helper used by the driver to request temperature
 * as part of background telemetry collection.
 */
void subc_mkii_request_temperature_internal(SubcMkII *driver)
{
    if (!driver || !driver->light_serial)
        return;
    /* Send temperature request command */
    Serial_print(driver->light_serial, "$St");
}


/**
 * subc_mkii_parse_response
 *
 * Calls each possible individual parser for the data
 */
void subc_mkii_parse_response(SubcMkII *driver)
{

	// Try to parse any data that might be in the buffer
	subc_mkii_parse_temperature(driver);		// First try to parse the temp
	subc_mkii_parse_humidity(driver);			// Then try to parse for humidity
	subc_mkii_parse_led_current(driver);		// Now try for LED current
	subc_mkii_parse_led_power(driver);			// Now try for LED power
	subc_mkii_parse_signal_enabled(driver);		// Now try for "signal enabled" setting
	subc_mkii_parse_device_runtime(driver);     // see if we can get the total runtime of device
}


/**
 * subc_mkii_parse_temperature
 *
 * Parse the current temperature value from a completed light response
 * and update cached temperature statistics.
 */
void subc_mkii_parse_temperature(SubcMkII *driver)
{

	Serial_print(&SerialUSB, "PARSE TRY: ");
	Serial_println(&SerialUSB, (char *)driver->response_buf);

    // Validate driver pointer before accessing internal state
    if (!driver)
        return;


    // Search the response buffer for the "Temp:" field
    char *p = strstr((char *)driver->response_buf, "Temp:");
    if (!p)
        return;

    // Advance pointer past the "Temp:" label
    p += 5;

    // Track sign for negative temperatures
    int32_t sign = 1;
    if (*p == '-')
    {
        sign = -1;
        p++;
    }

    // Parse the integer portion of the temperature
    int32_t int_part = 0;
    bool has_digits = false;

    while (*p >= '0' && *p <= '9')
    {
        has_digits = true;
        int_part = (int_part * 10) + (*p - '0');
        p++;
    }

    // Abort if no numeric digits were found
    if (!has_digits)
        return;

    // Parse the fractional portion, if present
    int32_t frac_part  = 0;
    int32_t frac_scale = 1;

    if (*p == '.')
    {
        p++;

        // Collect up to two fractional digits for centi-degree resolution
        while (*p >= '0' && *p <= '9' && frac_scale < 100)
        {
            frac_part  = (frac_part * 10) + (*p - '0');
            frac_scale *= 10;
            p++;
        }
    }

    // Convert parsed value into centi-degrees Celsius
    int32_t temp_centi =
        sign * ((int_part * 100) + ((frac_part * 100) / frac_scale));

    // Store most recent temperature
    driver->temp_current = temp_centi;

    // Initialize min/max on first sample
    if (driver->temp_samples == 0)
    {
        driver->temp_min = temp_centi;
        driver->temp_max = temp_centi;
    }
    else
    {
        // Update min/max bounds
        if (temp_centi < driver->temp_min) driver->temp_min = temp_centi;
        if (temp_centi > driver->temp_max) driver->temp_max = temp_centi;
    }

    // Accumulate sum for average calculation
    driver->temp_sum += temp_centi;
    driver->temp_samples++;
}


/**
 * subc_mkii_parse_humidity
 *
 * Parse the current humidity value from a completed light response
 * and update cached humidity statistics.
 */
void subc_mkii_parse_humidity(SubcMkII *driver)
{
    // Validate driver pointer before accessing internal state
    if (!driver)
        return;

    // Search the response buffer for the "Humidity:" field
    char *p = strstr((char *)driver->response_buf, "Humidity:");
    if (!p)
        return;

    // Advance pointer past the "Humidity:" label
    p += 9;

    // Parse the integer portion of the humidity value
    int32_t int_part = 0;
    bool has_digits = false;

    while (*p >= '0' && *p <= '9')
    {
        has_digits = true;
        int_part = (int_part * 10) + (*p - '0');
        p++;
    }

    // Abort if no numeric digits were found
    if (!has_digits)
        return;

    // Parse the fractional portion, if present
    int32_t frac_part  = 0;
    int32_t frac_scale = 1;

    if (*p == '.')
    {
        p++;

        // Collect up to two fractional digits for centi-percent resolution
        while (*p >= '0' && *p <= '9' && frac_scale < 100)
        {
            frac_part  = (frac_part * 10) + (*p - '0');
            frac_scale *= 10;
            p++;
        }
    }

    // Convert parsed value into centi-percent relative humidity
    int32_t humidity_centi =
        (int_part * 100) + ((frac_part * 100) / frac_scale);

    // Store most recent humidity value
    driver->humidity_current = humidity_centi;

    // Initialize min/max on first sample
    if (driver->humidity_samples == 0)
    {
        driver->humidity_min = humidity_centi;
        driver->humidity_max = humidity_centi;
    }
    else
    {
        // Update min/max bounds
        if (humidity_centi < driver->humidity_min)
            driver->humidity_min = humidity_centi;

        if (humidity_centi > driver->humidity_max)
            driver->humidity_max = humidity_centi;
    }

    // Accumulate sum for average calculation
    driver->humidity_sum += humidity_centi;
    driver->humidity_samples++;
}


/**
 * subc_mkii_parse_led_current
 *
 * Parse the current LED current percentage from the response buffer
 * and update the cached LED current state.
 */
void subc_mkii_parse_led_current(SubcMkII *driver)
{
    // Validate driver pointer before accessing internal state
    if (!driver)
        return;

    // Search the response buffer for the LED current field
    char *p = strstr((char *)driver->response_buf, "LED current:");
    if (!p)
        return;

    // Advance pointer past the "LED current:" label
    p += 12;

    // Parse integer portion of the percentage value
    int32_t int_part = 0;
    bool has_digits = false;

    while (*p >= '0' && *p <= '9')
    {
        has_digits = true;
        int_part = (int_part * 10) + (*p - '0');
        p++;
    }

    // Abort if no numeric digits were found
    if (!has_digits)
        return;

    // Parse fractional portion if present
    int32_t frac_part  = 0;
    int32_t frac_scale = 1;

    if (*p == '.')
    {
        p++;

        // Collect up to two fractional digits for centi-percent resolution
        while (*p >= '0' && *p <= '9' && frac_scale < 100)
        {
            frac_part  = (frac_part * 10) + (*p - '0');
            frac_scale *= 10;
            p++;
        }
    }

    // Convert parsed value into centi-percent
    driver->led_current_centi =
        (int_part * 100) + ((frac_part * 100) / frac_scale);
}


/**
 * subc_mkii_parse_led_power
 *
 * Parse the current LED power PU percentage from the response buffer
 * and update the cached LED power state.
 */
void subc_mkii_parse_led_power(SubcMkII *driver)
{
    // Validate driver pointer before accessing internal state
    if (!driver)
        return;

    // Search the response buffer for the LED power field
    char *p = strstr((char *)driver->response_buf, "LED Power PU:");
    if (!p)
        return;

    // Advance pointer past the "LED Power PU:" label
    p += 13;

    // Parse integer portion of the percentage value
    int32_t int_part = 0;
    bool has_digits = false;

    while (*p >= '0' && *p <= '9')
    {
        has_digits = true;
        int_part = (int_part * 10) + (*p - '0');
        p++;
    }

    // Abort if no numeric digits were found
    if (!has_digits)
        return;

    // Parse fractional portion if present
    int32_t frac_part  = 0;
    int32_t frac_scale = 1;

    if (*p == '.')
    {
        p++;

        // Collect up to two fractional digits for centi-percent resolution
        while (*p >= '0' && *p <= '9' && frac_scale < 100)
        {
            frac_part  = (frac_part * 10) + (*p - '0');
            frac_scale *= 10;
            p++;
        }
    }

    // Convert parsed value into centi-percent
    driver->led_power_centi =
        (int_part * 100) + ((frac_part * 100) / frac_scale);
}


/**
 * subc_mkii_parse_signal_enabled
 *
 * Parse the single-signal-enabled flag from the response buffer
 * and update the cached signal enabled state.
 */
void subc_mkii_parse_signal_enabled(SubcMkII *driver)
{
    // Validate driver pointer before accessing internal state
    if (!driver)
        return;

    // Search the response buffer for the signal enabled field
    char *p = strstr((char *)driver->response_buf,
                     "Single signal enabled:");
    if (!p)
        return;

    // Advance pointer past the label
    p += 22;

    // Only valid values are '0' or '1'
    if (*p == '0')
    {
        driver->signal_enabled = 0;
    }
    else if (*p == '1')
    {
        driver->signal_enabled = 1;
    }
}


/**
 * subc_mkii_parse_device_runtime
 *
 * Parse the device lifetime string reported by the light firmware.
 *
 * Expected format:
 *   "@Update at Runtime:HHH MM SS"
 *
 * Where:
 *   HHH = hours
 *   MM  = minutes
 *   SS  = seconds
 *
 * On success, stores total runtime in seconds and marks it valid.
 */
void subc_mkii_parse_device_runtime(SubcMkII *driver)
{
    // Validate driver pointer before accessing internal state
    if (!driver)
        return;

    // Look for the runtime marker string in the response buffer
    char *p = strstr((char *)driver->response_buf, "Update at Runtime:");
    if (!p)
        return;

    // Advance pointer past the label
    p += strlen("Update at Runtime:");

    // Skip any spaces before the numbers
    while (*p == ' ')
        p++;

    // Parse hours
    uint32_t hours = 0;
    if (*p < '0' || *p > '9')
        return;

    while (*p >= '0' && *p <= '9')
    {
        hours = (hours * 10) + (*p - '0');
        p++;
    }

    // Skip space between fields
    if (*p != ' ')
        return;
    p++;

    // Parse minutes
    uint32_t minutes = 0;
    if (*p < '0' || *p > '9')
        return;

    while (*p >= '0' && *p <= '9')
    {
        minutes = (minutes * 10) + (*p - '0');
        p++;
    }

    // Skip space between fields
    if (*p != ' ')
        return;
    p++;

    // Parse seconds
    uint32_t seconds = 0;
    if (*p < '0' || *p > '9')
        return;

    while (*p >= '0' && *p <= '9')
    {
        seconds = (seconds * 10) + (*p - '0');
        p++;
    }

    // Convert everything to total seconds
    driver->device_runtime_seconds =
        (hours * 3600UL) + (minutes * 60UL) + seconds;

    // Mark runtime as valid
    driver->device_runtime_valid = true;
}



/**
 * subc_mkii_get_uptime
 *
 * Return the driver uptime in milliseconds.
 */
bool subc_mkii_get_uptime(SubcMkII *driver, uint32_t *out_uptime_ms)
{
    if (!driver || !out_uptime_ms)
        return false;

    *out_uptime_ms = driver->uptime_ms;
    return true;
}


/**
 * subc_mkii_get_uptime
 *
 * Return the driver runtime in seconds.
 */
bool subc_mkii_get_device_runtime(SubcMkII *driver,
                                  uint32_t *out_seconds)
{
    if (!driver || !out_seconds)
        return false;

    if (!driver->device_runtime_valid)
        return false;

    *out_seconds = driver->device_runtime_seconds;
    return true;
}


/**
 * subc_mkii_get_stats_formatted
 *
 * Format all collected environmental and operational statistics
 * into a single human-readable string for external display.
 */
bool subc_mkii_get_stats_formatted(SubcMkII *driver,
                                   char *buf,
                                   size_t buf_len)
{
    // Validate inputs before accessing driver state
    if (!driver || !buf || buf_len == 0)
        return false;

    uint32_t t_cur, t_min, t_max, t_avg;
    uint32_t h_cur, h_min, h_max, h_avg;

    // Query temperature statistics from the driver
    bool have_temp =
        subc_mkii_get_temperature_stats(driver,
                                        &t_min,
                                        &t_max,
                                        &t_avg,
                                        &t_cur);

    // Query humidity statistics from the driver
    bool have_humidity =
        subc_mkii_get_humidity_stats(driver,
                                     &h_min,
                                     &h_max,
                                     &h_avg,
                                     &h_cur);

    // Get the device current uptime
    uint32_t tuptime_ms;
    bool have_uptime = subc_mkii_get_uptime(driver, &tuptime_ms);

    // get the device lifetime runtime
    uint32_t runtime_seconds;
       bool have_runtime =
           subc_mkii_get_device_runtime(driver, &runtime_seconds);



    size_t offset = 0;

    if (have_uptime)
	{
    	uint32_t total_seconds= tuptime_ms / 1000;
		uint32_t hours   = total_seconds / 3600;
		uint32_t minutes = (total_seconds % 3600) / 60;
		uint32_t seconds = total_seconds % 60;

		offset += snprintf(buf + offset,
						   buf_len - offset,
						   "Device Uptime: %lu:%02lu:%02lu (H:M:S)\r\n",
						   hours,
						   minutes,
						   seconds);
	}

    if (have_runtime)
	{
		uint32_t hours   = runtime_seconds / 3600;
		uint32_t minutes = (runtime_seconds % 3600) / 60;
		uint32_t seconds = runtime_seconds % 60;

		offset += snprintf(buf + offset,
						   buf_len - offset,
						   "Device Runtime: %lu:%02lu:%02lu (H:M:S)\r\n",
						   hours,
						   minutes,
						   seconds);
	}

    // Format temperature statistics if available
    if (have_temp)
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Temp C:%ld.%02ld  Min:%ld.%02ld  Max:%ld.%02ld  Avg:%ld.%02ld\r",
                           t_cur / 100, abs(t_cur % 100),
                           t_min / 100, abs(t_min % 100),
                           t_max / 100, abs(t_max % 100),
                           t_avg / 100, abs(t_avg % 100));
    }
    else
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Temp: no data\r");
    }

    // Format humidity statistics if available
    if (have_humidity)
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Humidity %%RH:%ld.%02ld  Min:%ld.%02ld  Max:%ld.%02ld  Avg:%ld.%02ld\r",
                           h_cur / 100, abs(h_cur % 100),
                           h_min / 100, abs(h_min % 100),
                           h_max / 100, abs(h_max % 100),
                           h_avg / 100, abs(h_avg % 100));
    }
    else
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Humidity: no data\r");
    }

    // Format current LED current percentage
    offset += snprintf(buf + offset,
                       buf_len - offset,
                       "LED current: %ld.%02ld %%\r",
                       driver->led_current_centi / 100,
                       abs(driver->led_current_centi % 100));

    // Format current LED power PU percentage
    offset += snprintf(buf + offset,
                       buf_len - offset,
                       "LED power PU: %ld.%02ld %%\r",
                       driver->led_power_centi / 100,
                       abs(driver->led_power_centi % 100));

    // Format single signal enabled state
    if (driver->signal_enabled == 0xFF)
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Single signal enabled: unknown\r");
    }
    else
    {
        offset += snprintf(buf + offset,
                           buf_len - offset,
                           "Single signal enabled: %u\r",
                           driver->signal_enabled);
    }

    return true;
}

