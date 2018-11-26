#include "i2c_mgr.h"

#define tag "I2C_MGR"

/* i2c_read_reg

    Read a OV7670 register

*/
uint8_t i2c_read_reg(uint8_t slv_addr, uint8_t reg)
{    
    uint8_t val = 0;

    esp_err_t ret = ESP_OK;

    // Write register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // Repeated start for reading
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &val, NACK_VAL);
    
    i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // printf("[0x%.2x]=0x%.2x\n", reg, data_rd[0]);

    if(ret != ESP_OK)
    {
        ESP_LOGE(tag, "I2C Read error: %d", ret);
    }

    return val;
}

/* i2c_write_reg

    Writes a OV7670 register

*/
void i2c_write_reg(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    // Write register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK)
    {
        ESP_LOGE(tag, "I2C write failed with error: %d", ret);
    }
}

/* i2c_write_bit

    Change only 1 bit in a register

*/
void i2c_write_bit(uint8_t slv_addr, uint8_t reg, uint8_t pos, uint8_t value)
{ 
    if(bit > 7 || value > 1)
        return;

    uint8_t regValue = 0;

    // Read the register
    regValue = i2c_read_reg(slv_addr, reg);

    if(value == 0)
    {
        regValue &= 0xFF - (1 << pos);
    }
    else
    {
        // Value = 1
        regValue |= 1 << pos;
    }

    // Write the register back with the updated bit
    i2c_write_reg(slv_addr, reg, regValue);
}

/* I2C_INIT

    Initialize I2C communication

*/
void i2c_init(int pin_sda, int pin_scl)
{
    esp_err_t ret = ESP_OK;

	ESP_LOGI(tag, "Initializing I2C...\n");

    // Configure the I2C communication
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = pin_sda;
	conf.scl_io_num = pin_scl;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ret = i2c_param_config(I2C_NUM_0, &conf);
	ret |= i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Check if no error occured
    if(ret != ESP_OK)
    {
        ESP_LOGE(tag, "Error initializing I2C: %d", ret);
    }
} 

/* i2c_scan

    Scan the I2C connected devices

*/
void i2c_scan() 
{
    ESP_LOGI(tag, "Entering %s\n", __func__);

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");

    // Go through I2C addresses
	for (i=3; i< 0x78; i++) 
    {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();

		i2c_master_start(cmd);
        // Expecting an ACK to signify a slave is there
		i2c_master_write_byte(cmd, (i << 1) | I2C_WRITE, ACK_CHECK_EN);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		
        if (i%16 == 0) 
        {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) 
        {
			printf(" %.2x", i);
		} 
        else 
        {
			printf(" --");
		}

		//ESP_LOGI(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
}