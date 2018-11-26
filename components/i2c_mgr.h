#ifndef __I2C_MGR_H__
#define __I2C_MGR_H__

#include <driver/i2c.h>
#include "esp_system.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include <driver/ledc.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include <stdlib.h>
#include <string.h>
#include "rom/lldesc.h"
#include "esp_intr_alloc.h"

#define I2C_WRITE                  0x00
#define I2C_READ                   0x01
#define ACK_CHECK_EN               0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS              0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                    0x0              /*!< I2C ack value */
#define NACK_VAL                   0x1  

void i2c_scan();
uint8_t i2c_read_reg(uint8_t slv_addr, uint8_t reg);
void i2c_write_reg(uint8_t slv_addr, uint8_t reg, uint8_t data);
void i2c_write_bit(uint8_t slv_addr, uint8_t reg, uint8_t pos, uint8_t value);
void i2c_init(int pin_sda, int pin_scl);



#endif // __I2C_MGR__