#include "blink.h"

/* BLINK_TASK

Blinking led at 1s period in a separate thread

*/
void blink_task(void *pvParameter)
{
    while(1) 
    {
        // Blinking green LED
        gpio_set_level(GPIO_LED_G, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        gpio_set_level(GPIO_LED_G, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Be careful to always kill the thread if exited!!
}


/* BLINK_INIT

    Init the LED GPIOs

*/
void blink_init()
{
    printf("Blink init...\n");
    // Set RGB LED to output
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_LED_G, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LED_G, 1));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_LED_B, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LED_B, 0));
    // ESP_ERROR_CHECK(gpio_set_direction(GPIO_LED_R, GPIO_MODE_OUTPUT));
    // ESP_ERROR_CHECK(gpio_set_level(GPIO_LED_R, 0));

}