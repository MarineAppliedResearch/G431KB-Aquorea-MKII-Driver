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
#include <string.h>  // required for memcpy if used elsewhere

// Single global Ethernet driver instance.
// Arduino assumes exactly one Ethernet interface.
static WizchipDriver eth_driver;

// Tracks whether Ethernet_begin has completed successfully.
// Used to guard all public API calls.
static bool eth_initialized = false;

// Global Ethernet interface statistics
static Ethernet_Stats eth_stats;

// Last observed physical link state
static bool cached_link_up;
static bool link_state_valid;


/**
 * Initialize the Ethernet interface.
 *
 * This function wraps wizchip_driver_init and records whether
 * initialization succeeded so that later calls can be validated.
 */
bool Ethernet_begin(const WizchipNetConfig *cfg)
{
    if (!cfg)
        return false;

    if (!wizchip_driver_init(&eth_driver, cfg))
    {
        // Track failed initialization attempts
        eth_stats.init_failures++;
        return false;
    }

    // Successful initialization
    eth_initialized = true;
    eth_stats.init_count++;



    ethernet_observe_link();

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

    ethernet_observe_link();
    return cached_link_up;
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


/**
 * Retrieve the configured IPv4 gateway address.
 *
 * This function queries the underlying WIZnet driver for the current
 * network configuration and extracts only the gateway address. It is
 * intentionally read-only and has no side effects.
 */
bool Ethernet_gatewayIP(uint8_t out_gw[4])
{
	// Ethernet must be initialized and output buffer must be valid
	if (!eth_initialized || !out_gw)
		return false;

	WizchipNetConfig info;

	// Query the driver for the active network configuration
	// Failure indicates the network stack is not ready
	if (!wizchip_driver_get_netinfo(&eth_driver, &info))
		return false;

	// Copy only the gateway address requested by the caller
	memcpy(out_gw, info.gateway, 4);

	return true;
}


/**
 * Retrieve the configured IPv4 subnet mask.
 *
 * This function queries the underlying WIZnet driver for the current
 * network configuration and extracts only the subnet mask. It is
 * read-only and does not modify driver or hardware state.
 */
bool Ethernet_subnetMask(uint8_t out_mask[4])
{
	// Ethernet must be initialized and output buffer must be valid
	if (!eth_initialized || !out_mask)
		return false;

	WizchipNetConfig info;

	// Query the driver for the active network configuration
	// Failure indicates the network stack is not ready
	if (!wizchip_driver_get_netinfo(&eth_driver, &info))
		return false;

	// Copy only the subnet mask requested by the caller
	memcpy(out_mask, info.subnet, 4);

	return true;
}


/**
 * Retrieve the configured MAC address.
 *
 * This function queries the underlying WIZnet driver for the current
 * network configuration and extracts only the MAC address. It is
 * read-only and does not modify driver or hardware state.
 */
bool Ethernet_macAddress(uint8_t out_mac[6])
{
    // Ethernet must be initialized and output buffer must be valid
    if (!eth_initialized || !out_mac)
        return false;

    WizchipNetConfig info;

    // Query the driver for the active network configuration
    // Failure indicates the network stack is not ready
    if (!wizchip_driver_get_netinfo(&eth_driver, &info))
        return false;

    // Copy only the MAC address requested by the caller
    memcpy(out_mac, info.mac, 6);

    return true;
}


/**
 * Query the current Ethernet PHY link state.
 *
 * This function provides a stable, Arduino-style link status
 * abstraction without exposing driver internals.
 */
EthernetLinkStatus Ethernet_linkStatus(void)
{
    // Cannot determine link state before initialization
    if (!eth_initialized)
        return ETHERNET_LINK_UNKNOWN;

    // Delegate PHY query to driver
    if (wizchip_driver_link_up(&eth_driver))
        return ETHERNET_LINK_UP;

    return ETHERNET_LINK_DOWN;
}


/**
 * Retrieve a human-readable Ethernet status string.
 *
 * This function is intended for diagnostics and CLI output.
 * It delegates formatting to the driver layer and enforces
 * initialization and buffer safety.
 */
bool Ethernet_status(char *buf, size_t len)
{
    // Validate inputs and initialization state
    if (!eth_initialized || !buf || len == 0)
        return false;

    // Driver emits status text into caller-provided buffer
    wizchip_driver_get_status(&eth_driver, buf, len);

    // Driver does not report failure; reaching here means success
    return true;
}


// Observe current link state and update stats if a transition is seen.
// Called opportunistically from existing API entry points.
void ethernet_observe_link(void)
{
    if (!eth_initialized)
        return;

    bool link_up = wizchip_driver_link_up(&eth_driver);

    if (!link_state_valid)
    {
        cached_link_up = link_up;
        link_state_valid = true;
        return;
    }

    if (link_up && !cached_link_up)
    {
        eth_stats.link_up_events++;
    }
    else if (!link_up && cached_link_up)
    {
        eth_stats.link_down_events++;
    }

    cached_link_up = link_up;
}

/**
 * Retrieve global Ethernet statistics.
 */
const Ethernet_Stats *Ethernet_getStats(void)
{
    return &eth_stats;
}


/**
 * Record transmitted packet and byte counts.
 *
 * Called internally by transport layers (UDP, TCP).
 */
void ethernet_record_tx(uint32_t bytes)
{
    eth_stats.tx_packets_total++;
    eth_stats.tx_bytes_total += bytes;
}


/**
 * Record transmitted packet and byte counts.
 *
 * Called internally by transport layers (UDP, TCP).
 */
void ethernet_record_rx(uint32_t bytes)
{
    eth_stats.rx_packets_total++;
    eth_stats.rx_bytes_total += bytes;
}
