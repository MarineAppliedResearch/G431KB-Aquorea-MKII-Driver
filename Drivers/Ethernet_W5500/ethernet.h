/**************************************************************************
 * ethernet.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * High-level Ethernet interface providing an Arduino-style API for
 * initializing and querying a single WIZnet-based Ethernet controller.
 *
 * This module represents the global Ethernet interface (singleton)
 * responsible for network bring-up and basic network status queries.
 * It wraps the wizchip_driver layer and intentionally hides low-level
 * hardware and driver details from application code.
 *
 * Socket-level functionality is provided separately by ethernet_udp.h.
 *************************************************************************/

#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>
#include <stdbool.h>
#include "wizchip_driver.h"

/**
 * EthernetLinkStatus
 *
 * Represents the current physical Ethernet link state.
 */
typedef enum
{
    ETHERNET_LINK_UNKNOWN = 0,
    ETHERNET_LINK_DOWN,
    ETHERNET_LINK_UP
} EthernetLinkStatus;


/**
 * Ethernet_Stats
 *
 * Global runtime statistics for the Ethernet interface.
 *
 * These counters track interface-level events and aggregate
 * traffic across all sockets.
 *
 * All counters are monotonic and observational only.
 */
typedef struct
{
    uint32_t init_count;        // Number of successful Ethernet_begin calls
    uint32_t init_failures;     // Failed initialization attempts

    uint32_t link_up_events;    // Link DOWN → UP transitions
    uint32_t link_down_events;  // Link UP → DOWN transitions

    uint32_t rx_packets_total;  // Total packets received across all sockets
    uint32_t tx_packets_total;  // Total packets transmitted across all sockets
} Ethernet_Stats;


/**
 * Ethernet_begin
 *
 * Initialize the Ethernet interface using the provided network
 * configuration.
 *
 * Inputs:
 *   cfg - Pointer to a WizchipNetConfig structure describing MAC
 *         address and static or DHCP network configuration
 *
 * Returns:
 *   true if the Ethernet controller was successfully initialized
 *   false if initialization failed or inputs are invalid
 *
 * Preconditions:
 *   Must be called once during system startup
 *   wizchip_port must be correctly configured
 *
 * Postconditions:
 *   Ethernet interface is initialized and ready for socket operations
 */
bool Ethernet_begin(const WizchipNetConfig *cfg);


/**
 * Ethernet_linkUp
 *
 * Query the physical Ethernet link status.
 *
 * Inputs:
 *   none
 *
 * Returns:
 *   true if the Ethernet PHY link is up
 *   false if the link is down or Ethernet has not been initialized
 *
 * Preconditions:
 *   Ethernet_begin must have been called successfully
 *
 * Postconditions:
 *   none
 */
bool Ethernet_linkUp(void);


/**
 * Ethernet_localIP
 *
 * Retrieve the currently assigned local IPv4 address.
 *
 * Inputs:
 *   out_ip - Pointer to a 4-byte buffer to receive the IP address
 *
 * Returns:
 *   true if a valid IP address was written to out_ip
 *   false if Ethernet is not initialized or out_ip is NULL
 *
 * Preconditions:
 *   Ethernet_begin must have been called successfully
 *
 * Postconditions:
 *   out_ip contains the current IPv4 address on success
 */
bool Ethernet_localIP(uint8_t out_ip[4]);


/**
 * Ethernet_gatewayIP
 *
 * Retrieve the configured IPv4 gateway address.
 *
 * Inputs:
 *   out_gw - Pointer to a 4-byte buffer to receive the gateway address
 *
 * Returns:
 *   true if a valid gateway address was written
 *   false if Ethernet is not initialized or out_gw is NULL
 *
 * Preconditions:
 *   Ethernet_begin must have been called successfully
 *
 * Postconditions:
 *   out_gw contains the configured gateway address on success
 */
bool Ethernet_gatewayIP(uint8_t out_gw[4]);


/**
 * Ethernet_subnetMask
 *
 * Retrieve the configured IPv4 subnet mask.
 *
 * Inputs:
 *   out_mask - Pointer to a 4-byte buffer to receive the subnet mask
 *
 * Returns:
 *   true if a valid subnet mask was written
 *   false if Ethernet is not initialized or out_mask is NULL
 *
 * Preconditions:
 *   Ethernet_begin must have been called successfully
 *
 * Postconditions:
 *   out_mask contains the configured subnet mask on success
 */
bool Ethernet_subnetMask(uint8_t out_mask[4]);


/**
 * Ethernet_macAddress
 *
 * Retrieve the configured MAC address.
 *
 * Inputs:
 *   out_mac - Pointer to a 6-byte buffer to receive the MAC address
 *
 * Returns:
 *   true if a valid MAC address was written
 *   false if Ethernet is not initialized or out_mac is NULL
 *
 * Preconditions:
 *   Ethernet_begin must have been called successfully
 *
 * Postconditions:
 *   out_mac contains the configured MAC address on success
 */
bool Ethernet_macAddress(uint8_t out_mac[6]);


/**
 * Ethernet_linkStatus
 *
 * Query the current Ethernet PHY link state.
 *
 * Inputs:
 *   none
 *
 * Returns:
 *   ETHERNET_LINK_UP if link is active
 *   ETHERNET_LINK_DOWN if link is down
 *   ETHERNET_LINK_UNKNOWN if Ethernet is not initialized
 *
 * Preconditions:
 *   Ethernet_begin must have been called
 */
EthernetLinkStatus Ethernet_linkStatus(void);


/**
 * Ethernet_status
 *
 * Retrieve a human-readable Ethernet status string.
 *
 * Inputs:
 *   buf - Destination buffer for status text
 *   len - Size of the destination buffer
 *
 * Returns:
 *   true if status text was written
 *   false if Ethernet is not initialized or inputs are invalid
 *
 * Preconditions:
 *   Ethernet_begin must have been called
 *
 * Postconditions:
 *   buf contains a null-terminated status string on success
 */
bool Ethernet_status(char *buf, size_t len);


/**
 * Ethernet_getStats
 *
 * Retrieve a pointer to the global Ethernet statistics structure.
 *
 * Returns:
 *   Pointer to a read-only Ethernet_Stats structure
 *
 * Notes:
 *   The returned pointer remains valid for the lifetime of the program.
 *   The caller must not modify the contents.
 */
const Ethernet_Stats *Ethernet_getStats(void);


#endif /* ETHERNET_H */
