# STM32 W5500 Ethernet + UDP Driver

Arduino style Ethernet and UDP API for STM32, written in C, built for real hardware.

This repository implements a C only Ethernet and UDP driver for STM32 microcontrollers using the WIZnet W5500 Ethernet controller. It is built on top of the WIZnet ioLibrary, but exposes a clean, Arduino style API (Ethernet and EthernetUDP) without copying Arduino internals or hiding behavior.

The design prioritizes predictability, observability, and debuggability over abstraction magic.

---

## Key Goals

- Arduino style usage semantics in C
- No hidden background tasks
- No user managed servicing loops
- Clear packet boundaries
- Strong runtime observability
- Survives real hardware problems such as link flaps, SPI issues, and partial failures

This is not a demo or minimal example. It is intended to run on real vehicles and lab hardware.

---

## Hardware and Software Requirements

### Hardware

- WIZnet W5500 Ethernet controller
- STM32 MCU with SPI peripheral
- GPIO lines for CS and RESET

### Software

- STM32 HAL
- WIZnet ioLibrary (socket and DHCP modules)
- No RTOS required

Assumptions:

- Exactly one Ethernet controller exists
- SPI and GPIO are configured externally (for example using CubeMX)
- Application is main loop driven

---

## Quick Start

### Network Configuration

Define a network configuration using WizchipNetConfig.

```c
static const WizchipNetConfig eth_cfg =
{
    .mac     = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 },
    .ip      = { 192, 168, 50, 3 },
    .subnet  = { 255, 255, 255, 0 },
    .gateway = { 192, 168, 50, 2 },
    .dns     = { 8, 8, 8, 8 },
    .mode    = WIZCHIP_NET_STATIC
};
```

Notes:

- MAC address must be unique
- For DHCP mode, only mac and mode set to WIZCHIP_NET_DHCP are required
- Static fields are ignored until a DHCP lease is obtained

---

### Ethernet Initialization

```c
if (!Ethernet_begin(&eth_cfg))
{
    Serial_print(&SerialUSB, "Ethernet init FAILED\r\n");
}
else
{
    Serial_print(&SerialUSB, "Ethernet init OK\r\n");
}
```

Behavior:

- Performs hardware reset
- Registers SPI and CS callbacks
- Verifies W5500 presence
- Brings up PHY link
- Configures static IP or starts DHCP
- Fails fast if hardware is not responding

No background servicing is required.

---

### UDP Socket Initialization

```c
EthernetUDP udp;

if (EthernetUDP_begin(&udp, 5000))
{
    Serial_print(&SerialUSB, "UDP listening on port 5000\r\n");
}
```

Each EthernetUDP instance owns exactly one W5500 hardware socket.

---

## Typical Main Loop Usage

### Receiving Packets

```c
int packet_size = EthernetUDP_parsePacket(&udp);
if (packet_size > 0)
{
    int len = EthernetUDP_read(&udp, rx_buf, sizeof(rx_buf));
}
```

Semantics:

- parsePacket detects packet boundaries
- read drains payload bytes only
- Packet metadata is latched on first read
- No hidden buffering or callbacks

---

### Accessing Sender Metadata

```c
uint8_t remote_ip[4];
uint16_t remote_port;

EthernetUDP_remoteIP(&udp, remote_ip);
EthernetUDP_remotePort(&udp, &remote_port);
```

Notes:

- Metadata becomes valid after the first successful read
- These accessors have no side effects

---

### Sending Packets

```c
EthernetUDP_beginPacket(&udp, remote_ip, remote_port);
EthernetUDP_write(&udp, data, len);
EthernetUDP_endPacket(&udp);
```

Behavior:

- Buffered TX
- Exact datagram boundaries
- Silent truncation if buffer fills
- Transmission occurs only at endPacket

---

## Runtime Statistics and Observability

### Global Ethernet Statistics

```c
const Ethernet_Stats *stats = Ethernet_getStats();
```

Tracked metrics:

- Initialization attempts and failures
- Link up and down transitions
- Total RX and TX packets
- Total RX and TX bytes

Design notes:

- Counters are monotonic
- Observational only
- Safe to read at any time
- Useful during failure analysis

---

### Per UDP Socket Statistics

Each EthernetUDP instance tracks:

- RX and TX packets
- RX and TX bytes
- RX truncations
- RX and TX errors
- parse, read, and send call counts

```c
EthernetUDP_Stats udp_stats;
EthernetUDP_getStats(&udp, &udp_stats);
```

Statistics are copied by value to preserve encapsulation.

---

## Internal Architecture

### Layered Design

```
Application Code
    |
Ethernet / EthernetUDP
    |
wizchip_driver
    |
wizchip_port
    |
STM32 HAL (SPI, GPIO)
```

---

### wizchip_port

Responsibilities:

- SPI byte and burst primitives
- Chip select control
- Hardware reset timing
- ioLibrary callback registration

Design intent:

- All board specific code lives here
- Porting to another board or SPI bus is isolated

---

### wizchip_driver

Responsibilities:

- W5500 bring up and verification
- Internal memory configuration
- PHY link monitoring
- Static IP or DHCP configuration
- High level driver state machine

Design intent:

- No socket exposure
- No application logic
- Bounded blocking during init, polling afterward

---

### Ethernet (Global Singleton)

Responsibilities:

- Arduino style global Ethernet interface
- Initialization guarding
- Link transition observation
- Global traffic statistics

Design intent:

- Exactly one Ethernet interface
- No socket management
- Clean separation from transport logic

---

### EthernetUDP

Responsibilities:

- Own exactly one hardware socket
- UDP RX and TX
- Arduino compatible semantics
- Per socket statistics

Design intent:

- No background servicing
- Strict packet boundaries
- Minimal hidden behavior

---

## Known Limitations

- Fixed socket index (currently socket 0)
- No dynamic socket allocation yet
- UDP only (no TCP support)
- No RTOS specific integration
- No power management hooks

---

## Future Directions

- Dynamic socket allocation
- TCP support
- Multiple simultaneous UDP sockets
- Improved DHCP robustness
- Optional RTOS integration hooks

---

## License

MIT License

Developed at Marine Applied Research & Exploration.
