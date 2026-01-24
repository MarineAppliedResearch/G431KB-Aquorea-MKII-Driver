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

    /* Background telemetry */
    subc_mkii_request_temperature_internal(driver, now_ms);

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

bool subc_mkii_get_temperature(SubcMkII *driver, float *out_temp)
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
                                     float *out_min,
                                     float *out_max,
                                     float *out_avg)
{
    if (!driver || driver->temp_samples == 0)
        return false;

    if (out_min)
        *out_min = driver->temp_min;

    if (out_max)
        *out_max = driver->temp_max;

    if (out_avg)
        *out_avg = driver->temp_sum / driver->temp_samples;

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


void subc_mkii_parse_temperature(SubcMkII *driver)
{
    char *p = strstr((char *)driver->response_buf, "Temp:");
    if (!p)
        return;

    float temp = strtof(p + 5, NULL);

    driver->temp_current = temp;

    if (driver->temp_samples == 0)
    {
        driver->temp_min = temp;
        driver->temp_max = temp;
        driver->temp_sum = temp;
    }
    else
    {
        if (temp < driver->temp_min) driver->temp_min = temp;
        if (temp > driver->temp_max) driver->temp_max = temp;
        driver->temp_sum += temp;
    }

    driver->temp_samples++;
}
