/**************************************************************************
 * onewire.c
 *
 * Author: Isaac Travers
 * Copyright: (c) 2026 Marine Applied Research & Exploration (MARE)
 * License: MIT
 * Date: 2026-04-17
 *
 * Purpose:
 * --------
 * Generic 1 Wire bus master library for STM32 HAL projects.
 * Provides bit banged 1 Wire reset, bit and byte read/write, ROM search,
 * ROM select, and CRC8 support for device drivers layered above it.
 *************************************************************************/

/* Public API for the generic 1 Wire bus layer. */
#include "onewire.h"

/* ---------------------------------------------------------------------
 * Private Function Prototypes
 * ------------------------------------------------------------------- */

/**
 * OneWire_DelayUs
 *
 * Purpose:
 *   Delay for a specified number of microseconds using the timer assigned
 *   to this 1 Wire bus instance.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *   us - Delay time in microseconds
 *
 * Outputs:
 *   None
 *
 * Preconditions:
 *   - ow is valid
 *   - ow->htim is valid
 *   - the timer is already running
 *   - the timer is configured for 1 microsecond per tick
 */
static void OneWire_DelayUs(OneWire_t *ow, uint16_t us);

/**
 * OneWire_BusInputDirection
 *
 * Purpose:
 *   Release the 1 Wire bus by configuring the data pin as an input.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_BusInputDirection(OneWire_t *ow);

/**
 * OneWire_BusOutputDirection
 *
 * Purpose:
 *   Drive the 1 Wire bus by configuring the data pin as open drain output.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_BusOutputDirection(OneWire_t *ow);

/**
 * OneWire_OutputLow
 *
 * Purpose:
 *   Drive the 1 Wire data pin low.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_OutputLow(OneWire_t *ow);

/**
 * OneWire_OutputHigh
 *
 * Purpose:
 *   Write a high level to the 1 Wire data pin output latch.
 *   On an open drain pin, the line will only go high after the pin is
 *   released and pulled up externally.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_OutputHigh(OneWire_t *ow);

/* ---------------------------------------------------------------------
 * Private Functions
 * ------------------------------------------------------------------- */

/**
 * OneWire_DelayUs
 *
 * Purpose:
 *   Delay for a specified number of microseconds using the configured
 *   hardware timer.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *   us - Delay time in microseconds
 *
 * Outputs:
 *   None
 */
static void OneWire_DelayUs(OneWire_t *ow, uint16_t us)
{
    /* Restart the timer count so the delay begins from zero. */
    ow->htim->Instance->CNT = 0;

    /* Busy wait until the requested number of microseconds has elapsed. */
    while (ow->htim->Instance->CNT <= us)
    {
        /* Intentionally empty. */
    }
}

/**
 * OneWire_BusInputDirection
 *
 * Purpose:
 *   Reconfigure the 1 Wire pin as an input so the bus is released and the
 *   external pullup resistor can pull the line high.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_BusInputDirection(OneWire_t *ow)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Configure the data pin as a floating input.
     * The 1 Wire bus uses an external pullup resistor. */
    GPIO_InitStruct.Pin   = ow->GPIO_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

    /* Apply the new pin configuration. */
    HAL_GPIO_Init(ow->GPIOx, &GPIO_InitStruct);
}

/**
 * OneWire_BusOutputDirection
 *
 * Purpose:
 *   Reconfigure the 1 Wire pin as an open drain output so the master can
 *   pull the line low during reset, write, and read slots.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_BusOutputDirection(OneWire_t *ow)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Configure the data pin as open drain output.
     * The bus is never actively driven high. */
    GPIO_InitStruct.Pin   = ow->GPIO_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

    /* Apply the new pin configuration. */
    HAL_GPIO_Init(ow->GPIOx, &GPIO_InitStruct);
}

/**
 * OneWire_OutputLow
 *
 * Purpose:
 *   Pull the 1 Wire bus low by writing the output latch low.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_OutputLow(OneWire_t *ow)
{
    /* Reset the output bit to drive the line low. */
    ow->GPIOx->BSRR = ((uint32_t)ow->GPIO_Pin << 16);
}

/**
 * OneWire_OutputHigh
 *
 * Purpose:
 *   Set the output latch high for the 1 Wire data pin.
 *   Because the pin is open drain, the line only rises after the bus is
 *   released and the external pullup resistor pulls it high.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
static void OneWire_OutputHigh(OneWire_t *ow)
{
    /* Set the output bit high in the output latch. */
    ow->GPIOx->BSRR = ow->GPIO_Pin;
}

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
 */
void OneWire_Init(OneWire_t *ow,
                  GPIO_TypeDef *GPIOx,
                  uint16_t GPIO_Pin,
                  TIM_HandleTypeDef *htim)
{
    /* Store the hardware resources used by this bus instance. */
    ow->GPIOx    = GPIOx;
    ow->GPIO_Pin = GPIO_Pin;
    ow->htim     = htim;

    /* Reset ROM search state so enumeration begins from a clean state. */
    ow->LastDiscrepancy       = 0;
    ow->LastFamilyDiscrepancy = 0;
    ow->LastDeviceFlag        = 0;

    /* Clear the cached ROM buffer. */
    for (uint8_t i = 0; i < 8; i++)
    {
        ow->ROM_NO[i] = 0;
    }

    /* Leave the bus in its idle released state.
     * Write the output latch high, then release the line so the external
     * pullup resistor holds the bus high. */
    OneWire_OutputHigh(ow);
    OneWire_BusInputDirection(ow);
}

/**
 * OneWire_Reset
 *
 * Purpose:
 *   Issue a 1 Wire reset pulse and check for a presence pulse.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 0 if at least one device responded.
 *   Returns 1 if no presence pulse was detected.
 */
uint8_t OneWire_Reset(OneWire_t *ow)
{
    uint8_t presence;

    /* Drive the bus low for the reset pulse. */
    OneWire_OutputLow(ow);
    OneWire_BusOutputDirection(ow);
    OneWire_DelayUs(ow, 480);

    /* Release the bus and wait into the presence detect window. */
    OneWire_BusInputDirection(ow);
    OneWire_DelayUs(ow, 70);

    /* A responding device pulls the line low during the presence pulse.
     * High means no device responded. */
    presence = HAL_GPIO_ReadPin(ow->GPIOx, ow->GPIO_Pin);

    /* Wait for the remainder of the reset recovery period. */
    OneWire_DelayUs(ow, 410);

    return presence;
}

/**
 * OneWire_WriteBit
 *
 * Purpose:
 *   Write one bit to the 1 Wire bus.
 *
 * Inputs:
 *   ow  - Pointer to the initialized 1 Wire bus object
 *   bit - Bit value to write
 *
 * Outputs:
 *   None
 */
void OneWire_WriteBit(OneWire_t *ow, uint8_t bit)
{
    if (bit)
    {
        /* Write '1' by pulling low briefly, then releasing the line. */
        OneWire_OutputLow(ow);
        OneWire_BusOutputDirection(ow);
        OneWire_DelayUs(ow, 6);

        OneWire_BusInputDirection(ow);
        OneWire_DelayUs(ow, 64);
    }
    else
    {
        /* Write '0' by holding the line low for most of the slot. */
        OneWire_OutputLow(ow);
        OneWire_BusOutputDirection(ow);
        OneWire_DelayUs(ow, 60);

        OneWire_BusInputDirection(ow);
        OneWire_DelayUs(ow, 10);
    }
}

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
 *   Returns the sampled bit value.
 */
uint8_t OneWire_ReadBit(OneWire_t *ow)
{
    uint8_t bit = 0;

    /* Start the read slot by pulling the line low briefly. */
    OneWire_OutputLow(ow);
    OneWire_BusOutputDirection(ow);
    OneWire_DelayUs(ow, 2);

    /* Release the bus so the slave can drive its response bit. */
    OneWire_BusInputDirection(ow);
    OneWire_DelayUs(ow, 10);

    /* Sample the bus during the valid read window. */
    if (HAL_GPIO_ReadPin(ow->GPIOx, ow->GPIO_Pin))
    {
        bit = 1;
    }

    /* Wait for the rest of the read slot to complete. */
    OneWire_DelayUs(ow, 50);

    return bit;
}

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
void OneWire_WriteByte(OneWire_t *ow, uint8_t byte)
{
    uint8_t i = 8;

    do
    {
        /* 1 Wire bytes are transmitted least significant bit first. */
        OneWire_WriteBit(ow, byte & 0x01);
        byte >>= 1;
    } while (--i);
}

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
uint8_t OneWire_ReadByte(OneWire_t *ow)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    do
    {
        /* Shift the accumulated byte right and insert the newly read bit
         * into the most significant position so the final result ends up
         * in normal LSB first order. */
        byte >>= 1;
        byte |= (OneWire_ReadBit(ow) << 7);
    } while (--i);

    return byte;
}

/**
 * OneWire_ResetSearch
 *
 * Purpose:
 *   Clear the internal ROM search state so a fresh device search can begin.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   None
 */
void OneWire_ResetSearch(OneWire_t *ow)
{
    /* Clear all search tracking state. */
    ow->LastDiscrepancy       = 0;
    ow->LastDeviceFlag        = 0;
    ow->LastFamilyDiscrepancy = 0;
}

/**
 * OneWire_Search
 *
 * Purpose:
 *   Perform a 1 Wire ROM search transaction using the supplied command.
 *
 * Inputs:
 *   ow      - Pointer to the initialized 1 Wire bus object
 *   command - Search style ROM command
 *
 * Outputs:
 *   Returns 1 if a ROM code was found.
 *   Returns 0 if no device was found or an error occurred.
 */
uint8_t OneWire_Search(OneWire_t *ow, uint8_t command)
{
    uint8_t id_bit_number;
    uint8_t last_zero;
    uint8_t rom_byte_number;
    uint8_t search_result;
    uint8_t id_bit;
    uint8_t cmp_id_bit;
    uint8_t rom_byte_mask;
    uint8_t search_direction;

    id_bit_number  = 1;
    last_zero      = 0;
    rom_byte_number = 0;
    rom_byte_mask  = 1;
    search_result  = 0;

    /* Stop immediately if the previous search already found the last device. */
    if (!ow->LastDeviceFlag)
    {
        /* Reset the bus and verify that at least one device is present. */
        if (OneWire_Reset(ow))
        {
            ow->LastDiscrepancy       = 0;
            ow->LastDeviceFlag        = 0;
            ow->LastFamilyDiscrepancy = 0;
            return 0;
        }

        /* Send the search command that begins ROM enumeration. */
        OneWire_WriteByte(ow, command);

        /* Walk through all 64 ROM bits using the standard search algorithm. */
        do
        {
            /* Read the current ROM bit and its complement from the bus. */
            id_bit     = OneWire_ReadBit(ow);
            cmp_id_bit = OneWire_ReadBit(ow);

            /* A 1/1 response indicates a search error or no participating device. */
            if ((id_bit == 1) && (cmp_id_bit == 1))
            {
                break;
            }
            else
            {
                if (id_bit != cmp_id_bit)
                {
                    /* No discrepancy at this position.
                     * All active devices share the same bit value. */
                    search_direction = id_bit;
                }
                else
                {
                    /* A 0/0 response means there is a discrepancy between devices.
                     * Choose a branch based on previous search history. */
                    if (id_bit_number < ow->LastDiscrepancy)
                    {
                        search_direction = ((ow->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    {
                        search_direction = (id_bit_number == ow->LastDiscrepancy);
                    }

                    /* Track the most recent zero branch so the next search can
                     * revisit the alternate path. */
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        if (last_zero < 9)
                        {
                            ow->LastFamilyDiscrepancy = last_zero;
                        }
                    }
                }

                /* Store the chosen branch bit into the cached ROM buffer. */
                if (search_direction == 1)
                {
                    ow->ROM_NO[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    ow->ROM_NO[rom_byte_number] &= (uint8_t)~rom_byte_mask;
                }

                /* Write the selected search branch back to the bus. */
                OneWire_WriteBit(ow, search_direction);

                /* Advance to the next ROM bit position. */
                id_bit_number++;
                rom_byte_mask <<= 1;

                /* Move to the next ROM byte after 8 bits. */
                if (rom_byte_mask == 0)
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8);

        /* A full 64 bit ROM was discovered successfully. */
        if (!(id_bit_number < 65))
        {
            ow->LastDiscrepancy = last_zero;

            if (ow->LastDiscrepancy == 0)
            {
                ow->LastDeviceFlag = 1;
            }

            search_result = 1;
        }
    }

    /* If the search failed, clear state so the next search restarts cleanly. */
    if (!search_result || !ow->ROM_NO[0])
    {
        ow->LastDiscrepancy       = 0;
        ow->LastDeviceFlag        = 0;
        ow->LastFamilyDiscrepancy = 0;
        search_result             = 0;
    }

    return search_result;
}

/**
 * OneWire_First
 *
 * Purpose:
 *   Reset search state and return the first device found on the bus.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 1 if a device was found.
 *   Returns 0 if no device was found.
 */
uint8_t OneWire_First(OneWire_t *ow)
{
    /* Start a fresh ROM search from the beginning of the tree. */
    OneWire_ResetSearch(ow);

    return OneWire_Search(ow, ONEWIRE_CMD_SEARCHROM);
}

/**
 * OneWire_Next
 *
 * Purpose:
 *   Continue the current ROM search and return the next device found.
 *
 * Inputs:
 *   ow - Pointer to the initialized 1 Wire bus object
 *
 * Outputs:
 *   Returns 1 if another device was found.
 *   Returns 0 if no more devices were found.
 */
uint8_t OneWire_Next(OneWire_t *ow)
{
    /* Continue the search using the current saved discrepancy state. */
    return OneWire_Search(ow, ONEWIRE_CMD_SEARCHROM);
}

/**
 * OneWire_Select
 *
 * Purpose:
 *   Select a specific device by sending Match ROM followed by its address.
 *
 * Inputs:
 *   ow   - Pointer to the initialized 1 Wire bus object
 *   addr - Pointer to the 8 byte ROM address
 *
 * Outputs:
 *   None
 */
void OneWire_Select(OneWire_t *ow, uint8_t *addr)
{
    uint8_t i;

    /* Send the Match ROM command so only the addressed device responds. */
    OneWire_WriteByte(ow, ONEWIRE_CMD_MATCHROM);

    /* Transmit the full 64 bit ROM code, least significant byte first. */
    for (i = 0; i < 8; i++)
    {
        OneWire_WriteByte(ow, addr[i]);
    }
}

/**
 * OneWire_GetFullROM
 *
 * Purpose:
 *   Copy the most recently discovered ROM code into the caller buffer.
 *
 * Inputs:
 *   ow        - Pointer to the initialized 1 Wire bus object
 *   firstByte - Pointer to an 8 byte destination buffer
 *
 * Outputs:
 *   None
 */
void OneWire_GetFullROM(OneWire_t *ow, uint8_t *firstByte)
{
    uint8_t i;

    /* Copy the cached 64 bit ROM code to the caller buffer. */
    for (i = 0; i < 8; i++)
    {
        firstByte[i] = ow->ROM_NO[i];
    }
}

/**
 * OneWire_CRC8
 *
 * Purpose:
 *   Compute the Dallas/Maxim CRC8 for a block of bytes.
 *
 * Inputs:
 *   addr - Pointer to the input data
 *   len  - Number of bytes to process
 *
 * Outputs:
 *   Returns the computed CRC8 value.
 */
uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t inbyte;
    uint8_t i;
    uint8_t mix;

    /* Process each input byte one bit at a time. */
    while (len--)
    {
        inbyte = *addr++;

        for (i = 8; i; i--)
        {
            /* Mix the incoming data bit with the current CRC LSB. */
            mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;

            /* Apply the Dallas/Maxim reflected CRC polynomial when needed. */
            if (mix)
            {
                crc ^= 0x8C;
            }

            /* Advance to the next input bit. */
            inbyte >>= 1;
        }
    }

    return crc;
}
