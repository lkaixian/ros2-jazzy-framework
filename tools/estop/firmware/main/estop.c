/* E-stop reader. Button NC contact between ESTOP_GPIO and GND, internal pull-up.
 * Contact closed (armed, wire intact) = LOW  -> "E0".
 * Pressed, wire cut or contact dirty  = HIGH -> "E1".
 * One line every 50 ms on the USB serial port (115200) is the heartbeat: the laptop treats silence as a stop too. */
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESTOP_GPIO GPIO_NUM_4

void app_main(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ESTOP_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&cfg);
    for (;;) {
        printf("E%d\n", gpio_get_level(ESTOP_GPIO) ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
