/* App firmware

*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <driver/ledc.h>

#include "blink.h"
#include "wifi_tcp_server_sta.h"
#include "ov7670.h"


static const char* tag = "app";

/* APP_MAIN_TASK()

    Infinite loop through application task

*/
void app_main_task(void *pvParam)
{
    UBaseType_t uxHighWaterMark;
     uint8_t val = 0;
    while(1)
    {
        // Check the socket received command
        switch(socket_command_received)
        {
            case SOCKET_COMMAND_NONE:
                break;
            case SOCKET_COMMAND_PRINT_IP_ADDRESS:
                wifi_print_ip_address();
                break;
            case SOCKET_COMMAND_SCAN_I2C:
                i2c_scan();
                break;
            case SOCKET_COMMAND_READ_OV7670:
                ov7670_read_reg(&val, 0x0A);
                printf("%.02x\n", val);
                ov7670_set_QQVGA();
                ov7670_dump();
                break;
            default:
                printf("Unknown request: %d", socket_command_received);
                break;
        }

        socket_command_received = SOCKET_COMMAND_NONE;

        if(pclkCounter%10000 == 0)
        {
            printf("Reached 100 000\n");
            pclkCounter = 1;
        }

        vTaskDelay(1000/portTICK_PERIOD_MS);

        //  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        //  printf("Stack usage: %d", uxHighWaterMark);
    }

    // never ends
    vTaskDelete(NULL);
}


int test;
/* ISR_PCLK

    Interrupt on each pclk rising edge

*/
// static void /*IRAM_ATTR*/ pclkHandler(void* arg) 
// {
//     if(test != 0)
//     {
//         gpio_set_level(4, 0);
//         test = 0;
//     }
//     else
//     {
//         test = 1;
//         gpio_set_level(4, 1);
//     }
//     // uint32_t gpio_num = (uint32_t) arg;
//     // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }
/* APP_MAIN()

*/
void app_main()
{
    ESP_LOGI(tag, "Starting %s \n", __func__);

    // Initialisation
    blink_init();

    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    
    wifi_event_group = xEventGroupCreate(); // Create the WiFi event pool

    wifi_initialise(); // initialise the WiFi as server station

    // Camera initialization
    esp_err_t err = ov7670_init();
    if(err != ESP_OK)
    {
        ESP_LOGE(tag, "Issue init OV7670");
    }

    // Create the blinking led task
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    // Create the WiFi IP/server tasks
    xTaskCreate(&wifi_print_ip_task,"printWiFiIP",2048,NULL,5,NULL);
    xTaskCreate(&tcp_server_task,"tcp_server",4096,NULL,5,NULL);

    // Launch the app task
    xTaskCreate(&app_main_task,"app_main_task",8096,NULL,5,NULL);


    // while(1)
    // {
    //     err = camera_run();

	//     ESP_LOGI(tag, "Printing.......");
	//     camera_print_fb();
        
    //     vTaskDelay(1000 / portTICK_RATE_MS);
    // }
}
