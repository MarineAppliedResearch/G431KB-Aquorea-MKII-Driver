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
#include "serial.h"


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

#ifdef __cplusplus
}
#endif

#endif /* SUBC_MKII_H */
