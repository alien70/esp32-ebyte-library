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
        vDelay(200);
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

uint8_t get_aux_pin()
{
    return _pin_configuration.aux_pin;
}
void set_aux_pin(uint8_t value)
{
    _pin_configuration.aux_pin = value;
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

    // waits until aux returns high or 1s
    complete_task(1000);
}

parity_t get_parity()
{
    return _parity;
}
void set_parity(parity_t value)
{
    _parity = value;
}

uart_baud_rate_t get_uart_baud_rate()
{
    return _baud_rate;
}
void set_uart_baud_rate(uart_baud_rate_t value)
{
    _baud_rate = value;
}

air_data_rate_t get_air_baud_rate()
{
    return _air_data_rate;
}
void set_air_baud_rate(air_data_rate_t value)
{
    _air_data_rate = value;
}

uint8_t get_model()
{
    return _model_data.model;
}

uint8_t get_version()
{
    return _model_data.version;
}

uint8_t get_features()
{
    return _model_data.features;
}

void ebyte_init()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    int baud_rate = 1200 * (1 << _baud_rate);
    if (_baud_rate == _57600)
        baud_rate = 57600;
    else if (_baud_rate == _115200)
        baud_rate = 115200;

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    switch (_parity)
    {
    default:
    case _8N1:
    {
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
    }
    break;

    case _8O1:
    {
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_ODD;
        uart_config.stop_bits = UART_STOP_BITS_1;
    }
    break;

    case _8E1:
    {
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_EVEN;
        uart_config.stop_bits = UART_STOP_BITS_1;
    }
    break;
    }

    _rx_buffer_size = 128;
    _tx_buffer_size = 128;

    uart_param_config(_uart_port, &uart_config);
    uart_set_pin(_uart_port, _pin_configuration.txd_pin, _pin_configuration.rxd_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(_uart_port, _rx_buffer_size * 2, _tx_buffer_size * 2, 0, NULL, 0);
}

bool read_version()
{
    // set _parameters to zero
    memset(_parameters, 0, sizeof(_parameters));

    set_mode(PROGRAM);

    const char c = 0xC3;
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);

    set_mode(NORMAL);

    uart_read_bytes(_uart_port, (uint8_t *)&_parameters, sizeof(_parameters), 20 / portTICK_RATE_MS);

    // printf("============\n");
    // printf("%02x %02x %02x %02x\n", _parameters[0], _parameters[1], _parameters[2], _parameters[3]);
    // printf("============\n");

    if (_parameters[0] != 0xC3)
        return false;

    // printf("+-+-+-+\n");

    // Read parameters below
    _model_data.model = _parameters[1];
    _model_data.version = _parameters[2];
    _model_data.features = _parameters[3];

    return true;
}

bool read_parameters()
{
    // set _parameters to zero
    memset(_parameters, 0, sizeof(_parameters));

    set_mode(PROGRAM);

    const char c = 0xC1;
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);
    uart_write_bytes(_uart_port, &c, 1);

    uart_read_bytes(_uart_port, (uint8_t *)&_parameters, sizeof(_parameters), 20 / portTICK_RATE_MS);

    set_mode(NORMAL);

    if (_parameters[0] != 0xC0)
        return false;

    // Read parameters below

    return true;
}

void vDelay(uint32_t delay)
{
    vTaskDelay(delay / portTICK_PERIOD_MS);
}

/**
 * @brief Checks AUX pin  for operation completion
 * 
 */
void complete_task(uint32_t delay)
{
    vDelay(20);

    TickType_t t0 = xTaskGetTickCount(), delta_t = 0;
    int _aux = gpio_get_level(_pin_configuration.aux_pin);

    for (;;)
    {
        _aux = gpio_get_level(_pin_configuration.aux_pin);
        delta_t = xTaskGetTickCount() - t0;

        if (_aux || delta_t >= (delay / portTICK_PERIOD_MS))
            break;

        vDelay(20);
    }

    // after AUX returns HIGH wait 2ms (20 to be sure)
    vDelay(20);
}