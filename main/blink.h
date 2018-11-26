#ifndef _BLINK_H_
#define	_BLINK_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"


#define GPIO_LED_G 2
#define GPIO_LED_R 0
#define GPIO_LED_B 4

void blink_task(void *pvParameter);
void blink_init();

#endif