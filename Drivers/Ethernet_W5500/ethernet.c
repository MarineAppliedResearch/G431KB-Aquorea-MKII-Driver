/**************************************************************************
 * ethernet.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * Implementation of the high-level Ethernet interface.
 *
 * This module provides a simple, Arduino-style Ethernet abstraction
 * on top of the wizchip_driver layer. It owns the single global
 * Ethernet driver instance and mediates access to basic network
 * status information.
 *
 * This module does not manage sockets or perform UDP or TCP I/O.
 *************************************************************************/

#include "ethernet.h"

// Single global Ethernet driver instance.
// Arduino assumes exactly one Ethernet interface.
static WizchipDriver eth_driver;

// Tracks whether Ethernet_begin has completed successfully.
// Used to guard all public API calls.
static bool eth_initialized = false;


/**
 * Initialize the Ethernet interface.
 *
 * This function wraps wizchip_driver_init and records whether
 * initialization succeeded so that later calls can be validated.
 */
bool Ethernet_begin(const WizchipNetConfig *cfg)
{
    // Reject invalid configuration pointer early
    if (!cfg)
        return false;

    // Attempt to initialize the underlying W5500 driver
    if (!wizchip_driver_init(&eth_driver, cfg))
        return false;

    // Mark Ethernet as initialized for subsequent API calls
    eth_initialized = true;
    return true;
}


/**
 * Query the physical Ethernet link state.
 *
 * This is a thin wrapper around the driver link check.
 */
bool Ethernet_linkUp(void)
{
    // Ethernet must be initialized before querying link state
    if (!eth_initialized)
        return false;

    // Delegate link status query to the driver layer
    return wizchip_driver_link_up(&eth_driver);
}


/**
 * Retrieve the currently assigned local IP address.
 *
 * This function delegates to the driver layer and validates
 * inputs and initialization state.
 */
bool Ethernet_localIP(uint8_t out_ip[4])
{
    // Validate that Ethernet is initialized and output buffer is valid
    if (!eth_initialized || !out_ip)
        return false;

    // Query the driver for the active IP address
    return wizchip_driver_get_ip(&eth_driver, out_ip);
}
