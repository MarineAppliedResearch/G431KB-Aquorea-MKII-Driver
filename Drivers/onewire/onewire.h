/**************************************************************************
 * onewire.h
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 * Date: 2026-04-17
 *
 * Purpose:
 * --------
 * Generic 1 Wire bus master library for STM32 HAL projects.
 * Provides bus reset, bit and byte read/write, ROM search, ROM select,
 * and CRC8 support for 1 Wire devices such as DS18S20 and DS18B20.
 *
 * Notes:
 * ------
 * - This module is the generic bus layer only.
 * - Device specific drivers such as ds18s20.c or ds18b20.c should build
 *   on top of this interface.
 * - The timer passed to OneWire_Init() must already be configured for
 *   1 microsecond per timer tick.
 * - The application is responsible for starting the timer before calling
 *   OneWire_Init().
 *************************************************************************/

#ifndef ONEWIRE_H
#define ONEWIRE_H

/* Fixed width integer types used by the public API. */
#include <stdint.h>

/* STM32 HAL types required by this module interface. */
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"

/* ---------------------------------------------------------------------
 * 1 Wire ROM and function commands
 * ------------------------------------------------------------------- */

/* Read scratchpad contents from the selected device. */
#define ONEWIRE_CMD_RSCRATCHPAD         0xBE

/* Write scratchpad contents to the selected device. */
#define ONEWIRE_CMD_WSCRATCHPAD         0x4E

/* Copy scratchpad contents into device EEPROM if supported. */
#define ONEWIRE_CMD_CPYSCRATCHPAD       0x48

/* Recall EEPROM contents back into scratchpad if supported. */
#define ONEWIRE_CMD_RECEEPROM           0xB8

/* Read the device power supply mode. */
#define ONEWIRE_CMD_RPWRSUPPLY          0xB4

/* Search for all ROM addresses on the bus. */
#define ONEWIRE_CMD_SEARCHROM           0xF0

/* Read ROM address from a single device bus. */
#define ONEWIRE_CMD_READROM             0x33

/* Select a specific device by its 64 bit ROM code. */
#define ONEWIRE_CMD_MATCHROM            0x55

/* Address all devices on the bus at once. */
#define ONEWIRE_CMD_SKIPROM             0xCC

/* ---------------------------------------------------------------------
 * Public Types
 * ------------------------------------------------------------------- */

/**
 * OneWire_t
 *
 * Purpose:
 *   Store the hardware configuration and search state for one 1 Wire bus.
 *
 * Fields:
 *   GPIOx                 - GPIO port used for the 1 Wire data pin
 *   GPIO_Pin              - GPIO pin mask used for the 1 Wire data pin
 *   htim                  - Timer used for microsecond delays
 *   LastDiscrepancy       - Last ROM search discrepancy position
 *   LastFamilyDiscrepancy - Last family code discrepancy position
 *   LastDeviceFlag        - Nonzero when the last device has been found
 *   ROM_NO                - Last ROM code found during bus search
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    TIM_HandleTypeDef *htim;

    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    uint8_t LastDeviceFlag;
    uint8_t ROM_NO[8];
} OneWire_t;

/* ---------------------------------------------------------------------
 * Public Functions
 * ------------------------------------------------------------------- */

/**
 * OneWire_Init
 *
 * Purpose:
 *   Initialize a 1 Wire bus instance with its GPIO pin and timing source.
 *
 * Inputs:
 *   ow       - Pointer to the 1 Wire bus object to initialize
 *   GPIOx    - GPIO port used by the 1 Wire bus
 *   GPIO_Pin - GPIO pin mask used by the 1 Wire bus
 *   htim     - Pointer to a timer configured for 1 microsecond per tick
 *
 * Outputs:
 *   None
 *
 * Preconditions:
 *   - ow is a valid pointer
 *   - htim is a valid pointer
 *   - the timer is already configured by the application
 *   - the timer has already been started by the application
 *
 * Postconditions:
 *   - the bus object stores the hardware configuration
 *   - the ROM search state is reset
 *   - the bus is left in its idle released state
 */
void OneWire_Init(OneWire_t *ow,
                  GPIO_TypeDef *GPIOx,
                  uint16_t GPIO_Pin,
                  TIM_HandleTypeDef *htim);

/**
 * OneWire_Reset
 *
 * Purpose:
 *   Issue a 1 Wire reset pulse and detect device presence.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 0 if at least one device responded with a presence pulse.
 *   Returns 1 if no device presence was detected.
 */
uint8_t OneWire_Reset(OneWire_t *ow);

/**
 * OneWire_ResetSearch
 *
 * Purpose:
 *   Clear the internal ROM search state so a new search can begin.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
void OneWire_ResetSearch(OneWire_t *ow);

/**
 * OneWire_First
 *
 * Purpose:
 *   Start a ROM search and return the first device found on the bus.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 1 if a device was found.
 *   Returns 0 if no device was found.
 */
uint8_t OneWire_First(OneWire_t *ow);

/**
 * OneWire_Next
 *
 * Purpose:
 *   Continue a ROM search and return the next device found on the bus.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 1 if another device was found.
 *   Returns 0 if no more devices were found.
 */
uint8_t OneWire_Next(OneWire_t *ow);

/**
 * OneWire_Search
 *
 * Purpose:
 *   Perform a ROM search operation using the supplied search command.
 *
 * Inputs:
 *   ow      - Pointer to the initialized 1 Wire bus object
 *   command - ROM search style command, typically ONEWIRE_CMD_SEARCHROM
 *
 * Outputs:
 *   Returns 1 if a device ROM was found.
 *   Returns 0 if the search failed or no device was found.
 */
uint8_t OneWire_Search(OneWire_t *ow, uint8_t command);

/**
 * OneWire_WriteBit
 *
 * Purpose:
 *   Write one bit to the 1 Wire bus.
 *
 * Inputs:
 *   ow  - Pointer to the initialized 1 Wire bus object
 *   bit - Bit value to write, where 0 writes a zero slot and nonzero
 *         writes a one slot
 *
 * Outputs:
 *   None
 */
void OneWire_WriteBit(OneWire_t *ow, uint8_t bit);

/**
 * OneWire_ReadBit
 *
 * Purpose:
 *   Read one bit from the 1 Wire bus.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns the sampled bit value, 0 or 1.
 */
uint8_t OneWire_ReadBit(OneWire_t *ow);

/**
 * OneWire_WriteByte
 *
 * Purpose:
 *   Write one byte to the 1 Wire bus, least significant bit first.
 *
 * Inputs:
 *   ow   - Pointer to the initialized 1 Wire bus object
 *   byte - Byte value to write
 *
 * Outputs:
 *   None
 */
void OneWire_WriteByte(OneWire_t *ow, uint8_t byte);

/**
 * OneWire_ReadByte
 *
 * Purpose:
 *   Read one byte from the 1 Wire bus, least significant bit first.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns the byte read from the bus.
 */
uint8_t OneWire_ReadByte(OneWire_t *ow);

/**
 * OneWire_GetFullROM
 *
 * Purpose:
 *   Copy the most recently discovered 64 bit ROM code into a caller buffer.
 *
 * Inputs:
 *   ow        - Pointer to the initialized 1 Wire bus object
 *   firstByte - Pointer to an 8 byte destination buffer
 *
 * Outputs:
 *   None
 */
void OneWire_GetFullROM(OneWire_t *ow, uint8_t *firstByte);

/**
 * OneWire_Select
 *
 * Purpose:
 *   Send a Match ROM command followed by an 8 byte ROM address.
 *
 * Inputs:
 *   ow   - Pointer to the initialized 1 Wire bus object
 *   addr - Pointer to the 8 byte ROM code to select
 *
 * Outputs:
 *   None
 */
void OneWire_Select(OneWire_t *ow, uint8_t *addr);

/**
 * OneWire_CRC8
 *
 * Purpose:
 *   Compute the Dallas/Maxim 1 Wire CRC8 over a byte buffer.
 *
 * Inputs:
 *   addr - Pointer to the input buffer
 *   len  - Number of bytes to process
 *
 * Outputs:
 *   Returns the computed CRC8 value.
 */
uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len);

#endif /* ONEWIRE_H */
