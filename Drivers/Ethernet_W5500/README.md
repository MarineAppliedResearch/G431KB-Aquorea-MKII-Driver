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
