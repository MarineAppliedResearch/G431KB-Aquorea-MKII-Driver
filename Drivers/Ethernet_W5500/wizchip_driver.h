/**************************************************************************
 * wizchip_driver.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * High-level driver for the WIZnet W5500 Ethernet controller.
 *
 * This driver sits above the wizchip_port layer and is responsible for:
 *   - Initializing the W5500 device
 *   - Configuring network parameters (static or DHCP)
 *   - Managing link and DHCP state
 *   - Preparing the device for TCP/UDP socket use
 *
 * All progress occurs via periodic polling from the main loop.
 *************************************************************************/

#ifndef WIZCHIP_DRIVER_H
#define WIZCHIP_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Includes
 * -------------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // for size_t

#ifdef IR
#undef IR
#endif
#include "wizchip_conf.h"

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */

/* Socket reserved for DHCP operations */
#define WIZCHIP_DHCP_SOCKET  7

/* Socket reserved for DNS operations */
#define WIZCHIP_DNS_SOCKET   6

/* --------------------------------------------------------------------------
 * Types
 * -------------------------------------------------------------------------- */


/**
 * WizchipNetMode
 *
 * Selects how the network interface is configured.
 */
typedef enum
{
    WIZCHIP_NET_STATIC = 0,
    WIZCHIP_NET_DHCP
} WizchipNetMode;


/**
 * WizchipNetConfig
 *
 * Network configuration parameters for the W5500.
 *
 * When using DHCP, only the MAC address is required.
 * Static fields are ignored until a lease is obtained.
 */
typedef struct
{
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
    uint8_t dns[4];
    WizchipNetMode mode; // DHCP or Static
} WizchipNetConfig;


/**
 * WizchipState
 *
 * Tracks the high-level state of the Ethernet interface.
 */
typedef enum
{
    WIZCHIP_STATE_UNINITIALIZED = 0,
    WIZCHIP_STATE_INIT,
    WIZCHIP_STATE_WAIT_LINK,
    WIZCHIP_STATE_WAIT_DHCP,
    WIZCHIP_STATE_READY,
    WIZCHIP_STATE_ERROR

} WizchipState;


/**
 * WizchipDriver
 *
 * Driver context for a single W5500 device.
 * One instance corresponds to one physical Ethernet controller.
 */
typedef struct
{
    /* Current driver state */
    WizchipState state;

    /* Network configuration */
    WizchipNetConfig net_cfg;

    /* True once a valid IP configuration is active */
    bool network_up;

    /* Timestamp of last state transition (ms) */
    uint32_t last_state_change_ms;

} WizchipDriver;


/**
 * wizchip_log_fn
 *
 * Optional debug logging callback.
 */
typedef void (*wizchip_log_fn)(const char *msg);

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
 * wizchip_driver_init
 *
 * Initialize a W5500 driver instance.
 *
 * Inputs:
 *   driver - Driver instance
 *   cfg    - Network configuration
 *
 * Outputs:
 *   true  if initialization begins successfully
 *   false if inputs are invalid or hardware is not detected
 */
bool wizchip_driver_init(WizchipDriver *driver,
                         const WizchipNetConfig *cfg);


/**
 * wizchip_driver_poll
 *
 * Advance the internal driver state machine.
 *
 * Notes:
 *   Must be called periodically from the main loop.
 *   Required for link monitoring and DHCP progression.
 */
void wizchip_driver_poll(WizchipDriver *driver,
                         uint32_t now_ms);


/**
 * wizchip_driver_is_ready
 *
 * Check whether the network interface is fully configured and ready.
 *
 * Outputs:
 *   true  if link is up and IP configuration is valid
 *   false otherwise
 */
bool wizchip_driver_is_ready(WizchipDriver *driver);


/**
 * wizchip_driver_get_ip
 *
 * Retrieve the currently active IP address.
 *
 * Notes:
 *   Only valid when wizchip_driver_is_ready() returns true.
 */
bool wizchip_driver_get_ip(WizchipDriver *driver,
                           uint8_t out_ip[4]);


/**
 * wizchip_driver_set_logger
 *
 * Register a debug logging callback for the driver.
 *
 * Notes:
 *   Passing NULL disables logging.
 */
void wizchip_driver_set_logger(wizchip_log_fn fn);


/**
 * wizchip_driver_link_up
 *
 * Check the current Ethernet PHY link status.
 *
 * Inputs:
 *   driver - Initialized driver instance
 *
 * Outputs:
 *   true  if the physical link is up
 *   false if link is down or driver is invalid
 */
bool wizchip_driver_link_up(WizchipDriver *driver);


/**
 * wizchip_driver_get_netinfo
 *
 * Retrieve the active network configuration from the W5500.
 *
 * Inputs:
 *   driver   - Initialized driver instance
 *   out_info - Caller-provided structure to receive network info
 *
 * Outputs:
 *   true  if network info was read successfully
 *   false if inputs are invalid
 */
bool wizchip_driver_get_netinfo(WizchipDriver *driver,
                                wiz_NetInfo *out_info);


/**
 * wizchip_driver_get_status
 *
 * Generate a human-readable summary of the driver state.
 *
 * Inputs:
 *   driver   - Driver instance
 *   buf      - Output string buffer
 *   buf_len  - Size of output buffer in bytes
 *
 * Notes:
 *   Intended for diagnostics and CLI inspection.
 */
void wizchip_driver_get_status(WizchipDriver *driver,
                               char *buf,
                               size_t buf_len);


/**
 * wizchip_driver_request_dhcp
 *
 * Initiate a DHCP lease request.
 *
 * Inputs:
 *   driver - Initialized driver instance
 *
 * Notes:
 *   Transitions the driver into DHCP wait state.
 *   Does not block and requires polling to complete.
 */
void wizchip_driver_request_dhcp(WizchipDriver *driver);

#ifdef __cplusplus
}
#endif

#endif /* WIZCHIP_DRIVER_H */
