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

// #define _DEBUG

#include "driver/uart.h"

// TODO: Eliminare appena finito il primo test completo
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
    PROGRAM       // Sleep mode, receiving parameter setting command is available
} OperatingModes;

/**
 * @brief Transmission parity_t Settings
 * 
 */
typedef enum
{
    _8N1 = 0,
    _8O1,
    _8E1
} parity_t;

/**
 * @brief Uart speed setting
 * 
 */
typedef enum
{
    _1200 = 0,
    _2400,
    _4800,
    _9600, // (default)
    _19200,
    _38400,
    _57600,
    _115200
} uart_baud_rate_t;

/**
 * @brief Air frequency settings
 * 
 */
typedef enum
{
    _1k = 0,
    _2k,
    _5k, // (default)
    _8k,
    _10k,
    _15k,
    _20k,
    _25k
} air_data_rate_t;

typedef struct
{
    int8_t m0_pin;
    int8_t m1_pin;
    int8_t txd_pin;
    int8_t rxd_pin;
    int8_t aux_pin;
} pin_configuration_t;

typedef struct
{
    uint8_t model;
    uint8_t version;
    uint8_t features;
} model_data_t;

/**
 * @brief 
 * 
 */
model_data_t _model_data;

/**
 * @brief 
 * 
 */
pin_configuration_t _pin_configuration;

uart_port_t _uart_port;
uart_baud_rate_t _baud_rate;
parity_t _parity;
air_data_rate_t _air_data_rate;

uint16_t _address;

uint8_t _channel;

uint8_t _option;

bool _save_on_power_down;

int _rx_buffer_size;
int _tx_buffer_size;

uint8_t _parameters[6];

/**
 * @brief Get the m0 pin 
 * 
 * @return uint8_t 
 */
uint8_t get_m0_pin();
/**
 * @brief Set the m0 pin
 * 
 * @param value 
 */
void set_m0_pin(uint8_t value);

/**
 * @brief Get the m1 pin
 * 
 * @return uint8_t 
 */
uint8_t get_m1_pin();
/**
 * @brief Set the m1 pin
 * 
 * @param value 
 */
void set_m1_pin(uint8_t value);

/**
 * @brief Get the txd pin
 * 
 * @return uint8_t 
 */
uint8_t get_txd_pin();
/**
 * @brief Set the txd pin
 * 
 * @param value 
 */
void set_txd_pin(uint8_t value);

/**
 * @brief Get the rxd pin
 * 
 * @return uint8_t 
 */
uint8_t get_rxd_pin();
/**
 * @brief Set the rxd pin
 * 
 * @param value 
 */
void set_rxd_pin(uint8_t value);

/**
 * @brief Get the aux pin
 * 
 * @param value 
 */
void set_aux_pin(uint8_t value);
/**
 * @brief Get the aux pin
 * 
 * @return uint8_t 
 */
uint8_t get_aux_pin();

/**
 * @brief Get the uart port
 * 
 * @return uart_port_t 
 */
uart_port_t get_uart_port();
/**
 * @brief Set the uart port
 * 
 * @param value 
 */
void set_uart_port(uart_port_t value);

/**
 * @brief Set the new operating mode
 * 
 * @param mode 
 */
void set_mode(OperatingModes mode);

/**
 * @brief Get the parity
 * 
 * @return parity_t 
 */
parity_t get_parity();
/**
 * @brief Set the parity
 * 
 * @param value 
 */
void set_parity(parity_t value);

/**
 * @brief Get the uart baud rate
 * 
 * @return uart_baud_rate_t 
 */
uart_baud_rate_t get_uart_baud_rate();
/**
 * @brief Set the uart baud rate
 * 
 * @param value 
 */
void set_uart_baud_rate(uart_baud_rate_t value);

/**
 * @brief Get the air baud rate
 * 
 * @return air_data_rate_t 
 */
air_data_rate_t get_air_baud_rate();
/**
 * @brief Set the air baud rate
 * 
 * @param value 
 */
void set_air_baud_rate(air_data_rate_t value);

/**
 * @brief Get the model
 * 
 * @return uint8_t 
 */
uint8_t get_model();

/**
 * @brief Get the version
 * 
 * @return uint8_t 
 */
uint8_t get_version();

/**
 * @brief Get the features
 * 
 * @return uint8_t 
 */
uint8_t get_features();

/**
 * @brief Initialize the lora module with the provided settings
 * 
 */
bool ebyte_init();

/**
 * @brief Read version number
 * 
 * @return true 
 * @return false 
 */
bool read_version();

/**
 * @brief Read the current settings from the module
 * 
 * @return true 
 * @return false 
 */
bool read_parameters();

/**
 * @brief Wait delay in milliseconds
 * 
 * @param delay 
 */
void vDelay(uint32_t delay);

/**
 * @brief 
 * 
 */
void complete_task(uint32_t);

/**
 * @brief Prints the model version data
 * 
 */
void print_version();

/**
 * @brief Evaluates uart configuration structure
 * 
 * @return int 
 */
uart_config_t eval_uart_config();

#endif // #ifndef __EBYTE_H__