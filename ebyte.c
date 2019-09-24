#include "ebyte.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED      (GPIO_NUM_2)

void dummy_task(void *argc)
{
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    int level = 0;
    for (;;)
    {
        gpio_set_level(LED, level);        
        vTaskDelay(200 / portTICK_PERIOD_MS);
        level = ! level;
    }
}