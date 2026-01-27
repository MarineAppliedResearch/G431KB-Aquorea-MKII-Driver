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

    // Uptime tracking starts on first poll
    driver->start_time_ms = 0;
    driver->uptime_ms     = 0;

    /* Associate this driver instance with the light's serial interface */
    driver->light_serial = light_serial;

    /* No exclusive command active at startup */
    driver->cmd_state = SUBC_CMD_IDLE;

    /* Clear response tracking state */
    driver->response_len   = 0;
    driver->response_ready = false;
    driver->last_rx_time_ms = 0;
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
 */
void subc_mkii_poll(SubcMkII *driver, uint32_t now_ms)
{
	// Make sure the driver, and serial connection are properly initialized.
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

    /* Background telemetry */
    subc_mkii_request_temperature_internal(driver, now_ms); // update the temp

    /* Drain all available bytes from the light serial port */
    while (Serial_available(driver->light_serial) > 0)
    {
    	// Read a byte from serial
        int c = Serial_read(driver->light_serial);

        // If we didn't read a byte, exit the loop
        if (c < 0)
            break;

        // If we have not filled up our driver buffer, copy that byte into it, drop otherwise.
        if (driver->response_len < SUBC_MKII_RESPONSE_BUF_SIZE)
        {
            driver->response_buf[driver->response_len++] = (uint8_t)c;
        }

        /* Update timestamp whenever we receive data */
        driver->last_rx_time_ms = now_ms;
    }

    /* If an exclusive command is active, check for response completion */
    if (driver->cmd_state == SUBC_CMD_EXCLUSIVE_ACTIVE &&
        driver->response_len > 0 &&
        (now_ms - driver->last_rx_time_ms) >= SUBC_MKII_RESPONSE_TIMEOUT_MS)
    {
        /* Null-terminate so we can safely treat it as a string */
        if (driver->response_len < SUBC_MKII_RESPONSE_BUF_SIZE)
        {
            driver->response_buf[driver->response_len] = '\0';
        }

        /* If this was a temperature request, parse it */
        subc_mkii_parse_temperature(driver);

        driver->cmd_state      = SUBC_CMD_IDLE;
        driver->response_ready = true;
    }
}


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



/* --------------------------------------------------------------------------
 * Private Functions
 * -------------------------------------------------------------------------- */

/**
 * subc_mkii_request_temperature_internal
 *
 * Internal helper used by the driver to request temperature
 * as part of background telemetry collection.
 */
void subc_mkii_request_temperature_internal(SubcMkII *driver,
                                                    uint32_t now_ms)
{
    if (!driver || !driver->light_serial)
        return;

    /* Do not interrupt another exclusive command */
    if (driver->cmd_state != SUBC_CMD_IDLE)
        return;

    /* Enforce internal rate limit */
    if ((now_ms - driver->last_temp_request_ms) <
        SUBC_MKII_TEMP_REQUEST_INTERVAL_MS)
        return;

    /* Prepare to collect response */
    driver->response_len   = 0;
    driver->response_ready = false;
    driver->cmd_state      = SUBC_CMD_EXCLUSIVE_ACTIVE;
    driver->last_temp_request_ms = now_ms;

    /* Send temperature request command */
    Serial_print(driver->light_serial, "$St");
}


/**
 * subc_mkii_parse_temperature
 *
 * Parse the current temperature value from a completed light response
 * and update cached temperature statistics.
 */
void subc_mkii_parse_temperature(SubcMkII *driver)
{
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
