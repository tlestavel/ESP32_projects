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


#define CAMERA_PIN_D0  
#define CAMERA_PIN_D1  
#define CAMERA_PIN_D2  
#define CAMERA_PIN_D3  
#define CAMERA_PIN_D4  
#define CAMERA_PIN_D5  
#define CAMERA_PIN_D6  
#define CAMERA_PIN_D7  
#define CAMERA_PIN_XCLK  
#define CAMERA_PIN_PCLK  
#define CAMERA_PIN_VSYNC  
#define CAMERA_PIN_HREF  
#define CAMERA_PIN_SDA  
#define CAMERA_PIN_SCL  
#define CAMERA_PIN_RESET 

#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
#define CAMERA_FRAME_SIZE   CAMERA_FS_QVGA

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

#define USE_WIFI_TCP
/* APP_MAIN()

*/
void app_main()
{
    ESP_LOGI(tag, "Starting %s \n", __func__);

    // Initialisation
    blink_init();

    // Flash init
    ESP_ERROR_CHECK( nvs_flash_init() );

#ifdef USE_WIFI_TCP
    // WiFi Init
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    
    wifi_event_group = xEventGroupCreate(); // Create the WiFi event pool

    wifi_initialise(); // initialise the WiFi as station
#endif

    // Camera initialization
    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d7 = CAMERA_PIN_D7,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_pclk = CAMERA_PIN_PCLK,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_i2c_sda = CAMERA_PIN_SDA,
        .pin_i2c_scl = CAMERA_PIN_SCL,
        .pin_reset = CAMERA_PIN_RESET,
        .xclk_freq_hz = 10000000,
    };

    // Check if the camera is connected
    camera_model_t camera_model;
    err = camera_probe(&camera_config, &camera_model);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
        return;
    }


    if (camera_model == CAMERA_OV7725) 
    {
        s_pixel_format = CAMERA_PIXEL_FORMAT;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        ESP_LOGI(TAG, "Detected OV7725 camera, using %s bitmap format",
                CAMERA_PIXEL_FORMAT == CAMERA_PF_GRAYSCALE ?
                        "grayscale" : "RGB565");
    } 
    else if (camera_model == CAMERA_OV2640) 
    {
        ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        camera_config.jpeg_quality = 15;
    } 
    else if(camera_model == CAMERA_OV7670)
    {
        ESP_LOGI(TAG, "Detected OV7670 camera, using RGB565 format");
    }
    else 
    {
        ESP_LOGE(TAG, "Camera not supported");
        return;
    }

    camera_config.pixel_format = s_pixel_format;
    err = camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }











    // Create the blinking led task
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

#ifdef USE_WIFI_TCP
    // // Create the WiFi IP/server tasks
    // xTaskCreate(&wifi_print_ip_task,"printWiFiIP",2048,NULL,5,NULL);
    // xTaskCreate(&tcp_server_task,"tcp_server",4096,NULL,5,NULL);
#endif

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
