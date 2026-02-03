/**************************************************************************
 * ethernet_udp.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 *
 * Purpose:
 * --------
 * High-level UDP socket interface providing an Arduino-style API for
 * sending and receiving UDP packets over Ethernet.
 *
 * This module defines the EthernetUDP data structure and associated
 * functions that encapsulate a single UDP socket. Each EthernetUDP
 * instance corresponds to exactly one WIZnet hardware socket and
 * delegates all low-level socket operations to the WIZnet ioLibrary.
 *
 * Network initialization and link management are handled separately
 * by the Ethernet module.
 *************************************************************************/

#ifndef ETHERNET_UDP_H
#define ETHERNET_UDP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define ETHERNETUDP_TX_BUFFER_SIZE 1472



/**
 * EthernetUDP_Stats
 *
 * Runtime statistics for a single UDP socket instance.
 *
 * These counters are monotonically increasing and are intended
 * for diagnostics, debugging, and performance analysis.
 *
 * All fields use uint32_t to ensure atomic access on Cortex-M
 * and predictable wraparound behavior.
 *
 * Statistics are observational only and MUST NOT affect
 * socket behavior or control flow.
 */
typedef struct
{
    // Packet-level counters
    uint32_t rx_packets;     // Number of UDP packets successfully detected
    uint32_t tx_packets;     // Number of UDP packets successfully transmitted

    // Byte-level counters
    uint32_t rx_bytes;       // Payload bytes delivered to the application
    uint32_t tx_bytes;       // Payload bytes queued for transmission

    // Error and anomaly tracking
    uint32_t rx_truncated;   // Packets where payload exceeded user reads
    uint32_t rx_errors;      // recv / recvfrom failures
    uint32_t tx_errors;      // sendto failures

    // Behavioral diagnostics (useful during bring-up)
    uint32_t parse_calls;    // Number of calls to EthernetUDP_parsePacket()
    uint32_t read_calls;     // Number of calls to EthernetUDP_read()
    uint32_t send_calls;     // Number of send attempts
} EthernetUDP_Stats;



/*
 * EthernetUDP
 *
 * Represents a single UDP socket bound to a local port.
 * One instance corresponds to one WIZnet hardware socket.
 */
typedef struct
{
    uint8_t  socket;        // Hardware socket index used by this instance
    uint16_t local_port;    // Local UDP port this socket is bound to

    uint8_t  remote_ip[4];  // Source IP address of the last received packet
    uint16_t remote_port;   // Source port of the last received packet

    bool     has_remote;      // True after first recvfrom latches sender metadata

    uint16_t remaining;     // Bytes remaining to be read from current packet
    bool     active;        // True once the socket has been successfully opened

    /* ---------- TX state (Arduino-style) ---------- */

   uint8_t  tx_ip[4];     // Destination IP for packet being built
   uint16_t tx_port;      // Destination port
   uint8_t  tx_buf[ETHERNETUDP_TX_BUFFER_SIZE];
   uint16_t tx_len;       // Bytes currently buffered
   bool     tx_active;    // True between beginPacket() and endPacket()
   EthernetUDP_Stats stats; // Per-socket runtime statistics

} EthernetUDP;





/* Debugger callback function */
typedef void (*ethernet_udp_log_fn)(const char *msg);

// Register an optional debug logger for EthernetUDP
void EthernetUDP_set_logger(ethernet_udp_log_fn fn);


/**
 * EthernetUDP_begin
 *
 * Open a UDP socket bound to the specified local port.
 *
 * Inputs:
 *   udp  - Pointer to an EthernetUDP instance
 *   port - Local UDP port number to bind
 *
 * Returns:
 *   true if the socket was successfully opened
 *   false if inputs are invalid or the socket could not be opened
 *
 * Preconditions:
 *   Ethernet_begin must have completed successfully
 *   udp must point to valid storage
 *
 * Postconditions:
 *   udp is associated with an open UDP socket and ready for use
 */
bool EthernetUDP_begin(EthernetUDP *udp, uint16_t port);


/**
 * EthernetUDP_parsePacket
 *
 * Check for an incoming UDP packet and prepare it for reading.
 *
 * Inputs:
 *   udp - Pointer to an initialized EthernetUDP instance
 *
 * Returns:
 *   Size of the received packet payload in bytes
 *   0 if no packet is available
 *
 * Preconditions:
 *   EthernetUDP_begin must have been called successfully
 *
 * Postconditions:
 *   Remote IP, remote port, and remaining byte count are updated
 *   if a packet is available
 */
int EthernetUDP_parsePacket(EthernetUDP *udp);


/**
 * EthernetUDP_read
 *
 * Read data from the current UDP packet payload.
 *
 * Inputs:
 *   udp - Pointer to an EthernetUDP instance
 *   buf - Destination buffer for received data
 *   len - Maximum number of bytes to read
 *
 * Returns:
 *   Number of bytes actually read
 *   0 if no data is available
 *
 * Preconditions:
 *   EthernetUDP_parsePacket must have returned a non-zero value
 *
 * Postconditions:
 *   Remaining byte count is reduced by the number of bytes read
 */
int EthernetUDP_read(EthernetUDP *udp, uint8_t *buf, size_t len);


/**
 * EthernetUDP_sendTo
 *
 * Send a UDP packet to a specified remote host.
 *
 * Inputs:
 *   udp  - Pointer to an initialized EthernetUDP instance
 *   buf  - Pointer to data to transmit
 *   len  - Number of bytes to transmit
 *   ip   - Destination IPv4 address
 *   port - Destination UDP port
 *
 * Returns:
 *   Number of bytes transmitted on success
 *   Negative value on failure
 *
 * Preconditions:
 *   EthernetUDP_begin must have been called successfully
 *
 * Postconditions:
 *   Packet data has been queued for transmission
 */
int EthernetUDP_sendTo(EthernetUDP *udp,
                       const uint8_t *buf,
                       size_t len,
                       const uint8_t ip[4],
                       uint16_t port);

/**
 * EthernetUDP_beginPacket
 *
 * Begin building a UDP packet for transmission.
 *
 * Inputs:
 *   udp  - Pointer to EthernetUDP instance
 *   ip   - Destination IPv4 address
 *   port - Destination UDP port
 *
 * Returns:
 *   true on success
 *   false if udp is invalid or a packet is already active
 */
bool EthernetUDP_beginPacket(EthernetUDP *udp,
                             const uint8_t ip[4],
                             uint16_t port);


/**
 * EthernetUDP_write
 *
 * Append data to the current outgoing UDP packet.
 *
 * Inputs:
 *   udp - Pointer to EthernetUDP instance
 *   buf - Data to append
 *   len - Number of bytes to write
 *
 * Returns:
 *   Number of bytes written (may be truncated if buffer fills)
 *
 * Notes:
 *   Data beyond internal buffer capacity is silently discarded
 *   to match Arduino EthernetUDP behavior.
 */
size_t EthernetUDP_write(EthernetUDP *udp,
                          const uint8_t *buf,
                          size_t len);


/**
 * EthernetUDP_endPacket
 *
 * Transmit the buffered UDP packet.
 *
 * Inputs:
 *   udp - Pointer to EthernetUDP instance
 *
 * Returns:
 *   true on success
 *   false on failure
 *
 * Postconditions:
 *   TX buffer is cleared and packet state reset
 */
bool EthernetUDP_endPacket(EthernetUDP *udp);


/**
 * EthernetUDP_remoteIP
 *
 * Retrieve the source IPv4 address of the most recently received packet.
 *
 * Inputs:
 *   udp     - Pointer to an EthernetUDP instance
 *   out_ip  - Pointer to a 4-byte buffer to receive the IP address
 *
 * Returns:
 *   true if a valid remote IP address was written
 *   false if no packet has been received or inputs are invalid
 *
 * Notes:
 *   The remote address becomes valid after the first successful
 *   EthernetUDP_read() call for a packet.
 */
bool EthernetUDP_remoteIP(const EthernetUDP *udp, uint8_t out_ip[4]);


/**
 * EthernetUDP_remotePort
 *
 * Retrieve the source UDP port of the most recently received packet.
 *
 * Inputs:
 *   udp       - Pointer to an EthernetUDP instance
 *   out_port  - Pointer to a variable to receive the port number
 *
 * Returns:
 *   true if a valid remote port was written
 *   false if no packet has been received or inputs are invalid
 *
 * Notes:
 *   The remote port becomes valid after the first successful
 *   EthernetUDP_read() call for a packet.
 */
bool EthernetUDP_remotePort(const EthernetUDP *udp, uint16_t *out_port);


/**
 * EthernetUDP_available
 *
 * Query how many payload bytes remain unread in the current packet.
 *
 * Inputs:
 *   udp - Pointer to an EthernetUDP instance
 *
 * Returns:
 *   Number of unread payload bytes
 *   0 if no packet is active or inputs are invalid
 */
int EthernetUDP_available(const EthernetUDP *udp);



/**
 * EthernetUDP_getStats
 *
 * Retrieve a snapshot of the UDP socket statistics.
 *
 * Inputs:
 *   udp - Pointer to an initialized EthernetUDP instance
 *   out - Destination buffer for statistics
 *
 * Returns:
 *   true if statistics were copied successfully
 *   false if inputs are invalid
 *
 * Notes:
 *   This function copies statistics by value to avoid exposing
 *   internal driver state or requiring synchronization.
 */
bool EthernetUDP_getStats(const EthernetUDP *udp,
                          EthernetUDP_Stats *out);
#endif /* ETHERNET_UDP_H */
