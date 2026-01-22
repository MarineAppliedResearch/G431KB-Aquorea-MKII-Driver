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
    /* Associate this driver instance with the light's serial interface */
    driver->light_serial = light_serial;

    /* No command is active at initialization */
    driver->command_active = false;
}
