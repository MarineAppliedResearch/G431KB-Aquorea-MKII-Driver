/**************************************************************************
 * ethernet_udp.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * Implementation of the EthernetUDP interface.
 *
 * This module provides a minimal UDP socket wrapper built on top of the
 * WIZnet ioLibrary socket API. It mirrors the core behavior of Arduino's
 * EthernetUDP class while remaining compatible with a C-only STM32
 * codebase.
 *
 * Network initialization and maintenance are handled by the Ethernet
 * module. This file is concerned only with socket-level UDP I/O.
 *************************************************************************/

#include "ethernet_udp.h"
#include "socket.h"
#include <string.h>

static ethernet_udp_log_fn udp_logger = NULL;


/**
 * Open and bind a UDP socket.
 *
 * This function initializes the EthernetUDP structure and opens
 * a hardware UDP socket bound to the specified local port.
 */
bool EthernetUDP_begin(EthernetUDP *udp, uint16_t port)
{
    // Validate input pointer
    if (!udp)
        return false;

    // Clear all state so the structure starts in a known condition
    memset(udp, 0, sizeof(*udp));

    // For the skeleton implementation we use a fixed socket index
    // This will be replaced later with proper socket allocation
    udp->socket = 0;
    udp->local_port = port;

    // Open a UDP socket using the WIZnet ioLibrary
    // socket() returns the socket number on success
    int ret = socket(udp->socket, Sn_MR_UDP, port, 0);
    if (ret != udp->socket)
        return false;

    // Mark this instance as active and ready for use
    udp->active = true;
    udp->has_remote = false; // No packet latched yet
    return true;
}


/**
 * Check for an incoming UDP packet and extract header information.
 *
 * This function mirrors Arduino's parsePacket behavior by reading
 * the UDP header to determine payload length and source address.
 */
int EthernetUDP_parsePacket(EthernetUDP *udp)
{
    // Validate instance before touching hardware
    if (!udp || !udp->active)
        return 0;

    // If the previous packet was not fully read, discard remaining payload bytes.
    // This keeps packet boundaries well defined for the next packet.
    while (udp->remaining > 0)
    {
        uint8_t dump[32];

        int n = recv(udp->socket,
                     dump,
                     (udp->remaining > sizeof(dump)) ?
                         sizeof(dump) : udp->remaining);

        // Stop if the socket reports no more data
        if (n <= 0)
            break;

        udp->remaining -= (uint16_t)n;
    }

    // Detect presence of a new UDP packet payload in RX buffer.
    // On W5500, reading sender metadata requires a real recvfrom call with len > 0.
    uint16_t rx_size = getSn_RX_RSR(udp->socket);
    if (rx_size == 0)
        return 0;

    // New packet is available.
    // Mark sender metadata as not yet latched so the first read uses recvfrom.
    udp->remaining = rx_size;
    udp->has_remote = false;

    return (int)udp->remaining;
}


/**
 * Read data from the current UDP packet payload.
 *
 * This function drains bytes from the socket RX buffer and
 * tracks how many bytes remain in the current packet.
 */
int EthernetUDP_read(EthernetUDP *udp, uint8_t *buf, size_t len)
{
    // Validate instance and ensure a packet is currently being read
    if (!udp || !udp->active || udp->remaining == 0 || !buf)
        return 0;

    // Clamp read length to remaining payload bytes
    if (len > udp->remaining)
        len = udp->remaining;

    int ret;

    if (!udp->has_remote)
    {
        // W5500 rule: recvfrom must be the first read of a UDP packet.
        // It consumes the UDP header and latches sender IP and port.
        ret = recvfrom(udp->socket,
                       buf,
                       (uint16_t)len,
                       udp->remote_ip,
                       &udp->remote_port);

        if (ret <= 0)
            return ret;

        udp->has_remote = true;
    }
    else
    {
        // After header is consumed, recv reads payload bytes only.
        ret = recv(udp->socket, buf, (uint16_t)len);
        if (ret <= 0)
            return ret;
    }

    udp->remaining -= (uint16_t)ret;

    if (udp_logger && ret > 0)
        udp_logger("EthernetUDP_read: payload read\r\n");

    return ret;
}


/**
 * Send a UDP packet to a remote destination.
 *
 * This function transmits the provided buffer as a single UDP packet.
 */
int EthernetUDP_sendTo(EthernetUDP *udp,
                       const uint8_t *buf,
                       size_t len,
                       const uint8_t ip[4],
                       uint16_t port)
{
	if (udp_logger)
	    udp_logger("EthernetUDP_sendTo called\r\n");

    // Validate instance and ensure socket is active
    if (!udp || !udp->active || !buf || !ip)
        return -1;

    // Delegate transmission to the WIZnet ioLibrary
    return sendto(udp->socket, buf, len, ip, port);
}


/**
 * Set a callback function that allows the ethernet device to debug log
 */
void EthernetUDP_set_logger(ethernet_udp_log_fn fn)
{
    udp_logger = fn;
}



/**
 * Begin construction of a UDP packet.
 *
 * This function initializes internal TX state for building a single
 * UDP datagram using Arduino-style semantics. No data is transmitted
 * until EthernetUDP_endPacket() is called.
 *
 * Only one packet may be active at a time. Nested packets are rejected.
 */
bool EthernetUDP_beginPacket(EthernetUDP *udp,
                             const uint8_t ip[4],
                             uint16_t port)
{
    if (!udp || !udp->active || !ip)
        return false;

    // Reject nested packets
    if (udp->tx_active)
        return false;

    memcpy(udp->tx_ip, ip, 4);
    udp->tx_port   = port;
    udp->tx_len    = 0;
    udp->tx_active = true;

    return true;
}


/**
 * Append data to the current outgoing UDP packet.
 *
 * Data is copied into an internal TX buffer and accumulated until
 * EthernetUDP_endPacket() is called. Transmission does not occur here.
 *
 * If the buffer fills, additional data is silently discarded to match
 * Arduino EthernetUDP behavior.
 */
size_t EthernetUDP_write(EthernetUDP *udp,
                          const uint8_t *buf,
                          size_t len)
{
    if (!udp || !udp->tx_active || !buf || len == 0)
        return 0;

    // Compute remaining buffer space
    size_t space = ETHERNETUDP_TX_BUFFER_SIZE - udp->tx_len;
    if (space == 0)
        return 0;

    // Arduino semantics: silently truncate
    if (len > space)
        len = space;

    memcpy(&udp->tx_buf[udp->tx_len], buf, len);
    udp->tx_len += (uint16_t)len;

    return len;
}


/**
 * Transmit the buffered UDP packet.
 *
 * Sends the accumulated TX buffer as a single UDP datagram using the
 * existing EthernetUDP_sendTo() primitive. Packet boundaries are
 * preserved exactly.
 *
 * TX state is reset regardless of success or failure.
 */
bool EthernetUDP_endPacket(EthernetUDP *udp)
{
    if (!udp || !udp->tx_active)
        return false;

    bool ok = false;

    // Send exactly one UDP datagram
    int ret = EthernetUDP_sendTo(udp,
                                 udp->tx_buf,
                                 udp->tx_len,
                                 udp->tx_ip,
                                 udp->tx_port);

    if (ret == (int)udp->tx_len)
        ok = true;

    // Reset TX state regardless of success
    udp->tx_active = false;
    udp->tx_len    = 0;

    return ok;
}
