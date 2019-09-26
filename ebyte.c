#include "ebyte.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED (GPIO_NUM_2)

// TODO: Eliminare dopo il primo test completo
void dummy_task(void *argc)
{
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    int level = 0;
    for (;;)
    {
        gpio_set_level(LED, level);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        level = !level;
    }
}

uint8_t get_m0_pin()
{
    return _pin_configuration.m0_pin;
}
void set_m0_pin(uint8_t value)
{
    _pin_configuration.m0_pin = value;
}

uint8_t get_m1_pin()
{
    return _pin_configuration.m1_pin;
}
void set_m1_pin(uint8_t value)
{
    _pin_configuration.m1_pin = value;
}

uint8_t get_txd_pin()
{
    return _pin_configuration.txd_pin;
}
void set_txd_pin(uint8_t value)
{
    _pin_configuration.txd_pin = value;
}

uint8_t get_rxd_pin()
{
    return _pin_configuration.rxd_pin;
}
void set_rxd_pin(uint8_t value)
{
    _pin_configuration.rxd_pin = value;
}

uart_port_t get_uart_port()
{
    return _uart_port;
}
void set_uart_port(uart_port_t value)
{
    _uart_port = value;
}

void set_mode(OperatingModes mode)
{
    gpio_set_level(_pin_configuration.m0_pin, (mode & 0x02) >> 1);
    gpio_set_level(_pin_configuration.m1_pin, (mode & 0x01));
}

parity_t get_parity()
{
    parity_t parity = _8N1;
    return parity;
}
void set_parity(parity_t value)
{
}

uart_baud_rate_t get_uart_baud_rate()
{
    uart_baud_rate_t baudRate = _9600;
    return baudRate;
}
void set_uart_baud_rate(uart_baud_rate_t value)
{
}

air_data_rate_t get_air_baud_rate()
{
    air_data_rate_t dataRate = _5k;
    return dataRate;
}
void set_air_baud_rate(air_data_rate_t value)
{
}

void ebyte_init()
{
}

bool read_parameters()
{
    memset(_parameters, 0, sizeof(_parameters));

    set_mode(PROGRAM);

    const char c = 0xC1;
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);

    uart_read_bytes(_uart_port, (uint8_t*)&_parameters, sizeof(_parameters), 20 / portTICK_RATE_MS);

    set_mode(NORMAL);

    if (_parameters[0] != 0xC0)
        return false;

    return true;
}