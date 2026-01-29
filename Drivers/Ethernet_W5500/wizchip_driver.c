/**************************************************************************
 * wizchip_driver.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * High-level driver for the WIZnet W5500 Ethernet controller.
 *
 * This module builds on the wizchip_port layer to bring up the W5500,
 * configure the network interface, and manage link and DHCP state.
 *
 * Initialization performs bounded blocking checks. Ongoing progress
 * is handled via polling.
 *************************************************************************/

#include "wizchip_driver.h"
#include "wizchip_port.h"

// IR must come before wizchip_conf.h
#ifdef IR
#undef IR
#endif

#include "wizchip_conf.h"
//#include "ctlnetwork.h"
#include "socket.h"
#include "DHCP/dhcp.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

// Optional debug logger provided by the application
static wizchip_log_fn logger = NULL;

// DHCP working buffer required by WIZnet DHCP module
static uint8_t dhcp_buffer[548];

// Tracks whether a DHCP lease has been successfully obtained
static bool dhcp_assigned = false;

/* --------------------------------------------------------------------------
 * DHCP callbacks
 * -------------------------------------------------------------------------- */

/**
 * dhcp_ip_assigned
 *
 * Called by the DHCP module when a lease is successfully acquired.
 */
static void dhcp_ip_assigned(void)
{
    // Mark network configuration as valid
    dhcp_assigned = true;

    // Report successful lease acquisition
    if (logger)
        logger("DHCP lease acquired");
}

/**
 * dhcp_ip_conflict
 *
 * Called by the DHCP module when an IP conflict is detected.
 */
static void dhcp_ip_conflict(void)
{
    // Invalidate current network configuration
    dhcp_assigned = false;

    // Report address conflict
    if (logger)
        logger("DHCP IP conflict detected");
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */


/**
 * wizchip_driver_set_logger
 *
 * Register an optional logging callback for driver diagnostics.
 * Passing NULL disables logging.
 */
void wizchip_driver_set_logger(wizchip_log_fn fn)
{
    // Store application-provided logging callback
    logger = fn;
}


/**
 * wizchip_driver_init
 *
 * Perform W5500 bring-up and initial network configuration.
 * This function blocks briefly while verifying hardware, link,
 * and starting DHCP if enabled.
 */
bool wizchip_driver_init(WizchipDriver *driver,
                         const WizchipNetConfig *cfg)
{
    uint8_t version;
    uint8_t link;
    wiz_NetInfo netinfo;

    // Validate input pointers
    if (!driver || !cfg)
        return false;

    // Initialize low-level hardware access and verify communication
      if (!wizchip_port_init())
      {
          // Enter error state if hardware bring-up fails
          driver->state = WIZCHIP_STATE_ERROR;

          // Report failure to initialize SPI/CS/RESET layer
          if (logger)
              logger("W5500 port init failed");

          return false;
      }

    // Clear driver state to a known baseline
    memset(driver, 0, sizeof(*driver));

    // Record initial configuration
    driver->state = WIZCHIP_STATE_INIT;
    driver->net_cfg = *cfg;
    driver->network_up = false;

    // Report start of Ethernet bring-up
    if (logger)
        logger("Initializing W5500");

    // Allocate internal TX/RX memory for sockets
    // This must occur before any register or socket access
    uint8_t memsize[2][8] =
    {
        {2,2,2,2,2,2,2,2},
        {2,2,2,2,2,2,2,2}
    };

    // Initialize W5500 internal memory configuration
    if (ctlwizchip(CW_INIT_WIZCHIP, memsize) != 0)
    {
        // Enter error state if memory setup fails
        driver->state = WIZCHIP_STATE_ERROR;

        // Memory init failure usually indicates SPI or CS issues
        if (logger)
            logger("W5500 memory init failed");

        return false;
    }

    // Read the VERSIONR register to confirm SPI communication
    version = getVERSIONR();

    // W5500 reports version 0x04; any other value indicates SPI failure
    if (version != 0x04)
    {
        // Enter error state
        driver->state = WIZCHIP_STATE_ERROR;

        // Report version mismatch
        if (logger)
            logger("W5500 version mismatch");

        return false;
    }

    // Confirm device presence
    if (logger)
        logger("W5500 detected");

    // Transition to link wait state
    driver->state = WIZCHIP_STATE_WAIT_LINK;
    link = PHY_LINK_OFF;

    // Wait for Ethernet PHY link to come up
    // Bounded to avoid deadlock when cable is unplugged
    for (int i = 0; i < 20; i++)
    {
        ctlwizchip(CW_GET_PHYLINK, &link);

        // Exit early once link is established
        if (link == PHY_LINK_ON)
            break;
    }

    // Abort if link never came up
    if (link != PHY_LINK_ON)
    {
        // Enter error state on link failure
        driver->state = WIZCHIP_STATE_ERROR;

        // Report link failure
        if (logger)
            logger("Ethernet link down");

        return false;
    }

    // Confirm physical network connection
    if (logger)
        logger("Ethernet link up");

    // Program MAC address into the W5500
    setSHAR(cfg->mac);

    // Configure network interface based on selected mode
    if (cfg->mode == WIZCHIP_NET_STATIC)
    {
    	wiz_NetInfo netinfo;
    	memset(&netinfo, 0, sizeof(netinfo));

    	memcpy(netinfo.mac, cfg->mac, 6);
    	memcpy(netinfo.ip, cfg->ip, 4);
    	memcpy(netinfo.sn, cfg->subnet, 4);
    	memcpy(netinfo.gw, cfg->gateway, 4);
    	memcpy(netinfo.dns, cfg->dns, 4);
    	netinfo.dhcp = NETINFO_STATIC;

    	ctlnetwork(CN_SET_NETINFO, &netinfo);

        // Mark network as usable
        driver->network_up = true;
        driver->state = WIZCHIP_STATE_READY;

        // Report successful configuration
        if (logger)
            logger("Static IP configured");
    }
    else
    {
        // Reset DHCP state before starting lease negotiation
        dhcp_assigned = false;

        // Initialize DHCP state machine
        DHCP_init(WIZCHIP_DHCP_SOCKET, dhcp_buffer);

        // Register DHCP event callbacks
        reg_dhcp_cbfunc(dhcp_ip_assigned,
                        dhcp_ip_assigned,
                        dhcp_ip_conflict);

        // Enter DHCP wait state
        driver->state = WIZCHIP_STATE_WAIT_DHCP;

        // Report DHCP startup
        if (logger)
            logger("DHCP started");
    }

    return true;
}


/**
 * wizchip_driver_poll
 *
 * Advance non-blocking network state such as DHCP processing.
 * Must be called periodically from the main loop.
 */
void wizchip_driver_poll(WizchipDriver *driver,
                         uint32_t now_ms)
{
    // Suppress unused parameter warning
    (void)now_ms;

    // Ignore calls before initialization
    if (!driver)
        return;

    // Advance DHCP state machine when waiting for a lease
    if (driver->state == WIZCHIP_STATE_WAIT_DHCP)
    {
        // Allow DHCP module to process timers and packets
        DHCP_run();

        // Transition to ready state once lease is obtained
        if (dhcp_assigned)
        {
            driver->network_up = true;
            driver->state = WIZCHIP_STATE_READY;

            // Report network readiness
            if (logger)
                logger("Network interface ready");
        }
    }
}


/**
 * wizchip_driver_is_ready
 *
 * Query whether the network interface is fully configured
 * and ready for socket operations.
 */
bool wizchip_driver_is_ready(WizchipDriver *driver)
{
    // Validate driver pointer
    if (!driver)
        return false;

    // Network is ready only in READY state
    return driver->state == WIZCHIP_STATE_READY;
}


/**
 * wizchip_driver_get_ip
 *
 * Retrieve the currently assigned IPv4 address.
 * Valid only after the network interface is ready.
 */
bool wizchip_driver_get_ip(WizchipDriver *driver,
                           uint8_t out_ip[4])
{
    wiz_NetInfo info;

    // Reject request if network is not active
    if (!driver || !out_ip || !driver->network_up)
        return false;

    // Query current network configuration from the chip
    ctlnetwork(CN_GET_NETINFO, &info);

    // Copy IP address to caller-provided buffer
    memcpy(out_ip, info.ip, 4);

    return true;
}


/**
 * wizchip_driver_link_up
 *
 * Query the W5500 PHY to determine Ethernet link state.
 *
 * Inputs:
 *   driver - Initialized driver instance
 *
 * Outputs:
 *   true  if link is established
 *   false if link is down or driver is invalid
 */
bool wizchip_driver_link_up(WizchipDriver *driver)
{
    uint8_t link;

    // Reject invalid driver context
    if (!driver)
        return false;

    // Read PHY link status directly from the chip
    ctlwizchip(CW_GET_PHYLINK, &link);

    return link == PHY_LINK_ON;
}


/**
 * wizchip_driver_get_netinfo
 *
 * Read the active network configuration from the W5500.
 *
 * Inputs:
 *   driver   - Initialized driver instance
 *   out_info - Destination structure for network parameters
 *
 * Outputs:
 *   true  if network info was retrieved
 *   false if inputs are invalid
 */
bool wizchip_driver_get_netinfo(WizchipDriver *driver,
                                wiz_NetInfo *out_info)
{
    // Validate inputs before accessing hardware
    if (!driver || !out_info)
        return false;

    // Query the W5500 network configuration registers
    ctlnetwork(CN_GET_NETINFO, out_info);

    return true;
}


/**
 * wizchip_driver_get_status
 *
 * Format a concise, human-readable driver status string.
 *
 * Inputs:
 *   driver  - Driver instance
 *   buf     - Output buffer for status text
 *   buf_len - Size of output buffer
 *
 * Notes:
 *   Intended for debugging and serial CLI output.
 */
void wizchip_driver_get_status(WizchipDriver *driver,
                               char *buf,
                               size_t buf_len)
{
    const char *state = "UNKNOWN";

    // Validate output buffer before writing
    if (!driver || !buf || buf_len == 0)
        return;

    // Translate internal state enum into readable text
    switch (driver->state)
    {
        case WIZCHIP_STATE_INIT:        state = "INIT"; break;
        case WIZCHIP_STATE_WAIT_LINK:   state = "WAIT_LINK"; break;
        case WIZCHIP_STATE_WAIT_DHCP:   state = "WAIT_DHCP"; break;
        case WIZCHIP_STATE_READY:       state = "READY"; break;
        case WIZCHIP_STATE_ERROR:       state = "ERROR"; break;
        default: break;
    }

    // Emit status string for diagnostics
    snprintf(buf, buf_len,
             "Ethernet state: %s\r\n",
             state);
}


/**
 * wizchip_driver_request_dhcp
 *
 * Start a DHCP lease request using the configured socket.
 *
 * Inputs:
 *   driver - Initialized driver instance
 *
 * Notes:
 *   Does not block.
 *   Requires wizchip_driver_poll() to progress.
 */
void wizchip_driver_request_dhcp(WizchipDriver *driver)
{
    // Ignore invalid driver context
    if (!driver)
        return;

    // Only allow DHCP to be started from valid states
    if (driver->state != WIZCHIP_STATE_READY &&
        driver->state != WIZCHIP_STATE_WAIT_DHCP)
        return;

    // Clear any previous lease state
    dhcp_assigned = false;

    // Initialize DHCP state machine on reserved socket
    DHCP_init(WIZCHIP_DHCP_SOCKET, dhcp_buffer);

    // Register DHCP event callbacks
    reg_dhcp_cbfunc(dhcp_ip_assigned,
                    dhcp_ip_assigned,
                    dhcp_ip_conflict);

    // Transition driver into DHCP wait state
    driver->state = WIZCHIP_STATE_WAIT_DHCP;

    // Report DHCP initiation if logging is enabled
    if (logger)
        logger("DHCP request issued");
}
