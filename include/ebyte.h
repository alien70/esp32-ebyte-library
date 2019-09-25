/*
    MIT License

    Copyright (c) 2019 Maurizio Attanasi

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.    
 */

/**
 * @brief 
 *   Module connection
 *  Module	MCU						Description
 *  MO		Any digital pin*		pin to control working/program modes
 *  M1		Any digital pin*		pin to control working/program modes
 *  Rx		Any digital pin			pin to MCU TX pin (module transmits to MCU, hence MCU must recieve data from module
 *  Tx		Any digital pin			pin to MCU RX pin (module transmits to MCU, hence MCU must recieve data from module
 *  AUX		Any digital pin			pin to indicate when an operation is complete (low is busy, high is done)
 *  Vcc		+3v3 or 5V0				
 *  Gnd		Ground					Ground must be common to module and MCU		
 */

#ifndef __EBYTE_H__
#define __EBYTE_H__

#include "driver/gpio.h"
#include "driver/uart.h"

void dummy_task(void *argc);

/**
 * @brief Available operating modes obtained from the combination 
 * of M0 ans M1 digital inputs
 * M0   M1
 * 0    0 NORMAL
 * 0    1 WAKEUP
 * 1    0 POWER_SAVING
 * 1    1 SLEEP
 */
typedef enum EOperatingModes
{
    NORMAL = 0,   // UART and wireless channel are open, transparent transmission is on
    WAKEUP,       // sends a preamble to waken receiver
    POWER_SAVING, // can't transmit but receive works only in wake up mode
    SLEEP         // Sleep mode, receiving parameter setting command is available
} OperatingModes;

/**
 * @brief Transmission Parity Settins
 * 
 */
typedef enum EParity
{
    _8N1 = 0,
    _8O1,
    _8E1
} Parity;

/**
 * @brief Uart speed setting
 * 
 */
typedef enum EUartBaudRate
{
    _1200 = 0,
    _2400,
    _4800,
    _9600,  // (default)
    _19200,
    _38400,
    _57600,
    _115200
} UartBaudRate;

/**
 * @brief Air frequency settings
 * 
 */
typedef enum EAirDataRate
{
    _1k = 0,
    _2k,
    _5k,    // (default)
    _8k,
    _10k,
    _15k,
    _20k,
    _25k
} AirDataRate;

typedef struct SPinConfiguration {
    uint8_t m0_pin;
    uint8_t m1_pin;
    uint8_t txd_pin;
    uint8_t rxd_pin;
    uint8_t aux_pin;
} PinConfiguration;

PinConfiguration _pin_configuration;

uint8_t get_m0_pin();
void set_m0_pin(uint8_t value);

uint8_t get_m1_pin();
void set_m1_pin(uint8_t value);

uint8_t get_txd_pin();
void set_txd_pin(uint8_t value);

uint8_t get_rxd_pin();
void set_rxd_pin(uint8_t value);

/**
 * @brief Get current operating mode
 * 
 * @return OperatingModes 
 */
OperatingModes get_mode();
/**
 * @brief Set the new operating mode
 * 
 * @param mode 
 */
void set_mode(OperatingModes mode);

/**
 * @brief Get the parity
 * 
 * @return Parity 
 */
Parity get_parity();
/**
 * @brief Set the parity
 * 
 * @param value 
 */
void set_parity(Parity value);

/**
 * @brief Get the uart baud rate
 * 
 * @return UartBaudRate 
 */
UartBaudRate get_uart_baud_rate();
/**
 * @brief Set the uart baud rate
 * 
 * @param value 
 */
void set_uart_baud_rate(UartBaudRate value);

/**
 * @brief Get the air baud rate
 * 
 * @return AirDataRate 
 */
AirDataRate get_air_baud_rate();
/**
 * @brief Set the air baud rate
 * 
 * @param value 
 */
void set_air_baud_rate(AirDataRate value);

/**
 * @brief Initialize the lora module with the provided settings
 * 
 */
void ebyte_init();



#endif // #ifndef __EBYTE_H__