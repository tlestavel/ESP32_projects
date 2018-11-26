#include "ov7670.h"

static const char *tag="ov7670";



static lldesc_t s_dma_desc[2];
static uint32_t* s_dma_buf[2];
static uint8_t* s_fb;
static bool s_initialized = false;
static int s_fb_w;
static int s_fb_h;
static size_t s_fb_size;
static volatile int s_isr_count = 0;
static volatile int s_line_count = 0;
static volatile int s_cur_buffer = 0;
static int s_buf_line_width;
static int s_buf_height;
static volatile bool s_i2s_running = 0;
static SemaphoreHandle_t s_data_ready;
static SemaphoreHandle_t s_frame_ready;
static intr_handle_t s_i2s_intr_handle = NULL;





static esp_err_t xclk_init();
static void ov7670_i2s_init();
static void i2s_run(size_t line_width, int height);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init(int line_width);
static void frame_buffer_fill_row_task(void *pvParameters);




int test;
/* ISR_PCLK

    Interrupt on each pclk rising edge

*/
static void pclkHandler(void* arg) 
{

    if(pclkCounter%10000 == 0)
    {

    }
    else
    {
        pclkCounter++;
    }
    // if(test != 0)
    // {
    //     gpio_set_level(4, 0);
    //     test = 0;
    // }
    // else
    // {
    //     test = 1;
    //     gpio_set_level(4, 1);
    // }
    // uint32_t gpio_num = (uint32_t) arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/* I2C_SCAN_TASK

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


/* OV7670_READ

    Read a OV7670 register

*/
esp_err_t ov7670_read_reg(uint8_t* data_rd, uint8_t reg)
{    
    // Write register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( OV7670_ADDRESS << 1 ) | I2C_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // Repeated start for reading
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( OV7670_ADDRESS << 1 ) | I2C_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    
    i2c_master_stop(cmd);

    ret |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // printf("[0x%.2x]=0x%.2x\n", reg, data_rd[0]);

    // ESP_ERROR_CHECK(ret);

    return ret;
}

/* OV7670_WRITE

    Writes a OV7670 register

*/
esp_err_t ov7670_write_reg(uint8_t reg, uint8_t value)
{
    // Write register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( OV7670_ADDRESS << 1 ) | I2C_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    // ESP_ERROR_CHECK(ret);

    return ret;
}

/* OV7670_write_bit_in_reg

    Change only 1 bit in a register

*/
esp_err_t ov7670_write_bit_in_reg(uint8_t reg, uint8_t bit, uint8_t value)
{ 
    esp_err_t ret = ESP_OK;

    if(bit > 7 || value > 1)
        return ESP_ERR_INVALID_ARG;

    uint8_t regValue = 0;

    // Read the register
    ret = ov7670_read_reg(&regValue, reg);

    if(value == 0)
    {
        regValue &= 0xFF - (1 << bit);
    }
    else
    {
        // Value = 1
        regValue |= 1 << bit;
    }

    // Write the register back with the updated bit
    ret = ov7670_write_reg(reg, regValue);

    return ret;
}

/* OV7670_I2C_INIT

    Initialize I2C communication

*/
void ov7670_i2c_init()
{
	ESP_LOGI(tag, "Initializing I2C...\n");

    // Configure the I2C communication
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = OV7670_SDA_PIN;
	conf.scl_io_num = OV7670_SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
} 

// static QueueHandle_t q1;
// // PCLOCK TEST
// esp_err_t ov7670_init() 
// {
//     ESP_LOGI(tag, "Entering %s\n", __func__);

//     // Set the RESET pin to high
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_CAMERA_RSTN, GPIO_MODE_OUTPUT));
//     ESP_ERROR_CHECK(gpio_set_level(OV7670_CAMERA_RSTN, 1));

//     // // Set the camera parallel interface pins
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D0, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D1, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D2, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D3, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D4, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D5, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D6, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_D7, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_PCLK, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_VSYNC, GPIO_MODE_INPUT));
//     ESP_ERROR_CHECK(gpio_set_direction(OV7670_PIN_HREF, GPIO_MODE_INPUT));

//     // //PCLK is interruptible, falling edge
    
// 	// // gpio_num_t gpio;
// 	// // q1 = xQueueCreate(10, sizeof(gpio_num_t));
//     // gpio_config_t gpioConfig;
// 	// gpioConfig.pin_bit_mask = GPIO_SEL_22;
// 	// gpioConfig.mode         = GPIO_MODE_INPUT;
// 	// gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
// 	// gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
// 	// gpioConfig.intr_type    = GPIO_INTR_NEGEDGE;
// 	// gpio_config(&gpioConfig);
//     // gpio_set_intr_type(OV7670_PIN_PCLK, GPIO_INTR_NEGEDGE);
//     // // //install gpio isr service
//     // gpio_install_isr_service(0);
//     // // //hook isr handler for specific gpio pin
//     // gpio_isr_handler_add(OV7670_PIN_PCLK, pclkHandler, NULL);

    
// }

/* OV7670_dump
    Dumping some OV7670 parameters
 */
void ov7670_dump() 
{
    ESP_LOGI(tag, "Entering %s\n", __func__);

	uint8_t com7 = 0; 
    ov7670_read_reg(&com7, OV7670_REG_COM7);
	uint8_t outputFormat = GET_BIT(com7, 2) << 1 | GET_BIT(com7, 0);
	switch (outputFormat) 
    {
		case 0b00:
			printf("YUV\n");
			break;
		case 0b10:
			printf("RGB\n");
			break;
		case 0b01:
			printf("Raw Bayer RGB\n");
			break;
		case 0b11:
			printf("Process Bayer RGB\n");
			break;
		default:
			printf("Unknown\n");
			break;
	}
    
	if (outputFormat == 0b10) 
    {
		uint8_t com15 = 0;
        ov7670_read_reg(&com15, OV7670_REG_COM15);
		uint8_t rgbType = GET_BIT(com15, 5) << 1 | GET_BIT(com15, 4);
	
		switch (rgbType) 
        {
			case 0b00:
			case 0b10:
				printf("Normal RGB Output\n");
				break;
			case 0b01:
				printf("RGB 565\n");
				break;
			case 0b11:
				printf("RGB 555\n");
				break;
			default:
				printf("Unknown\n");
				break;
		}
	}

    if(GET_BIT(com7, 1) == 1)
    {
        printf("Color bar enabled\n");
    }
    else
    {
        printf("Color bar disabled \n");
    }

	uint8_t scaling_xsc = 0;
    ov7670_read_reg(&scaling_xsc, OV7670_REG_SCALING_XSC);
	uint8_t scaling_ysc = 0;
    ov7670_read_reg(&scaling_ysc, OV7670_REG_SCALING_YSC);

	uint8_t testPattern = GET_BIT(scaling_xsc, 7) << 1 | GET_BIT(scaling_ysc, 7);

	switch (testPattern) 
    {
		case 0b00:
			printf("Not test output\n");
			break;
		case 0b01:
			printf("Shifting 1\n");
			break;
		case 0b10:
			printf("8-bar color pattern\n");
			break;
		case 0b11:
			printf("Fade to gray color bar\n");
			break;
		default:
			printf("unknown\n");
			break;
	}
    
	printf("Horizontal scale factor: %d\n", scaling_xsc & 0x3f);
	printf("Vertical scale factor: %d\n", scaling_ysc & 0x3f);

	uint8_t com15 = 0;
    ov7670_read_reg(&com15, OV7670_REG_COM15);
	switch ((com15 & 0b11000000) >> 6) 
    {
		case 0b00:
		case 0b01:
			ESP_LOGI(tag, "Output range: 0x10 to 0xf0\n");
			break;
		case 0b10:
			ESP_LOGI(tag, "Output range: 0x01 to 0xfe\n");
			break;
		case 0b11:
			ESP_LOGI(tag, "Output range: 0x00 to 0xff\n");
			break;
		default:
			break;
	}

    uint8_t pclk = 0;
    ov7670_read_reg(&pclk, OV7670_REG_COM10);
    if(GET_BIT(pclk, 4) == 1)
    {
        printf("Data on PCLK rising edge\n");
    }
    else
    {
        printf("Data on PCLK falling edge\n");
    }
} 

/* OV7670_I2C_INIT

    Set the camera mode to QQVGA
    
*/
esp_err_t ov7670_set_QQVGA()
{
    ESP_LOGI(tag, "Entering %s\n", __func__);

    esp_err_t ret = ESP_OK;

    ret = ov7670_write_reg(OV7670_REG_CLKRC, 0x01);
    ret |= ov7670_write_reg(OV7670_REG_COM7, 0x00);
    ret |= ov7670_write_reg(OV7670_REG_COM3, 0x0C);
    ret |= ov7670_write_reg(OV7670_REG_COM14, 0x12);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_XSC, 0x3A);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_YSC, 0x35);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_DCWCTR, 0x22);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_PCLK_DIV, 0xF2);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_PCLK_DELAY, 0x2A);

    return ret;
}



/* OV7670_I2C_INIT

    Set the camera mode to QVGA
    
*/
esp_err_t ov7670_set_QVGA()
{
    ESP_LOGI(tag, "Entering %s\n", __func__);

    esp_err_t ret = ESP_OK;

    ret = ov7670_write_reg(OV7670_REG_CLKRC, 0x01);
    ret |= ov7670_write_reg(OV7670_REG_COM7, 0x00);
    ret |= ov7670_write_reg(OV7670_REG_COM3, 0x04);
    ret |= ov7670_write_reg(OV7670_REG_COM14, 0x19);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_XSC, 0x3A);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_YSC, 0x35);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_DCWCTR, 0x11);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_PCLK_DIV, 0xF1);
    ret |= ov7670_write_reg(OV7670_REG_SCALING_PCLK_DELAY, 0x02);

    return ret;
}





















static esp_err_t xclk_init()
{
    // Starting timer for the master clk
	periph_module_enable(PERIPH_LEDC_MODULE);

	ledc_timer_config_t timer_conf;
	timer_conf.freq_hz    = OV7670_XCLK_FREQ;
	timer_conf.duty_resolution = (ledc_timer_bit_t) 1;
	timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	timer_conf.timer_num  = LEDC_TIMER_0;

	esp_err_t err = ledc_timer_config(&timer_conf);
	if (err != ESP_OK) 
    {
		ESP_LOGE(tag, "ledc_timer_config failed, rc=%x", err);
		return err;
	}

    // Using the LEDC module for generating the master clk
	ledc_channel_config_t ch_conf;
	ch_conf.channel    = LEDC_CHANNEL_0;
	ch_conf.timer_sel  = LEDC_TIMER_0;
	ch_conf.intr_type  = LEDC_INTR_DISABLE;
	ch_conf.duty       = 1;
	ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	ch_conf.gpio_num   = OV7670_XCLK_PIN;

	err = ledc_channel_config(&ch_conf);
	if (err != ESP_OK) 
    {
		ESP_LOGE(tag, "ledc_channel_config failed, rc=%x", err);
		return err;
	}

	return ESP_OK;
}

esp_err_t ov7670_init()
{
    // Protect from re-init
    if (s_initialized) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;

    // XCLK init
    err = xclk_init();
    if(err != ESP_OK)
    {
        return err;
    }

    // I2C init
    ov7670_i2c_init();

    // Reset
    ESP_ERROR_CHECK(gpio_set_direction(OV7670_CAMERA_RSTN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(OV7670_CAMERA_RSTN, 0));
	vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(gpio_set_level(OV7670_CAMERA_RSTN, 1));
	vTaskDelay(10 / portTICK_PERIOD_MS);

    // Check that the camera is present
    uint8_t pid = 0;
    uint8_t ver = 0;
    uint8_t midl = 0;
    uint8_t midh = 0;

    ESP_LOGI(tag, "Reading OV7670 registers\n");
    err = ov7670_read_reg(&pid, OV7670_REG_PID);
    err |= ov7670_read_reg(&ver, OV7670_REG_VER);
    err |= ov7670_read_reg(&midl, OV7670_REG_MIDL);
    err |= ov7670_read_reg(&midh, OV7670_REG_MIDH);

    if(err != ESP_OK)
    {
        ESP_LOGI(tag, "Camera not detected!\n");
        return err;
    }
    else
    {
        ESP_LOGI(tag, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x\n",
            pid, ver, midl, midh);
    }

    // Sensor init
    // ov7670_init(&s_sensor);

    // Sensor reset (mandatory??)
    // ESP_LOGI(tag, "Doing SW reset of sensor");
    // s_sensor.reset(&s_sensor);

#if ENABLE_TEST_PATTERN
    /* Test pattern may get handy
       if you are unable to get the live image right.
       Once test pattern is enable, sensor will output
       vertical shaded bars instead of live image.
    */
    // s_sensor.set_colorbar(&s_sensor, 1);
    ESP_LOGI(tag, "Test pattern enabled");
#endif

    // Set the frame size (QVGA)
    s_fb_w = OV7670_QVGA_WIDTH; //resolution[framesize][0];
    s_fb_h = OV7670_QVGA_HEIGHT; //resolution[framesize][1];
    ESP_LOGI(tag, "Setting frame size at %dx%d", s_fb_w, s_fb_h);
    // if (s_sensor.set_framesize(&s_sensor, framesize) != 0) 
    // {
    //     ESP_LOGE(tag, "Failed to set frame size");
    //     return ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
    // }

    err = ov7670_set_QVGA();
    if(err != ESP_OK)
    {
        ESP_LOGE(tag, "Set QVGA failed\n");
        return err;
    }


    // Allocating the frame buffer memory
    s_fb_size = s_fb_w * s_fb_h;
    ESP_LOGI(tag, "Allocating frame buffer (%dx%d, %d bytes)", s_fb_w, s_fb_h,
            s_fb_size);
    s_fb = (uint8_t*) malloc(s_fb_size);
    if (s_fb == NULL) 
    {
        ESP_LOGE(tag, "Failed to allocate frame buffer");
        return ESP_ERR_NO_MEM;
    }

    // Initialize I2S, pins and DMA buffers
    ESP_LOGI(tag, "Initializing I2S and DMA");
    ov7670_i2s_init();
    err = dma_desc_init(s_fb_w);
    if (err != ESP_OK) 
    {
        free(s_fb);
        return err;
    }

    // Create the sync signals
    s_data_ready = xSemaphoreCreateBinary();
    s_frame_ready = xSemaphoreCreateBinary();

    // Force this thread to the CORE 0
    xTaskCreatePinnedToCore(&frame_buffer_fill_row_task, "frame_buf_fill", 2048, NULL, 10,
            NULL, 0);

    
    ESP_LOGI(tag, "Skipping the first frame");
    // skip at least one frame after changing camera settings
    while (gpio_get_level(OV7670_PIN_VSYNC) == 0) 
    {
        ;
    }
    while (gpio_get_level(OV7670_PIN_VSYNC) != 0) 
    {
        ;
    }
    while (gpio_get_level(OV7670_PIN_VSYNC) == 0) 
    {
        ;
    }

    ESP_LOGI(tag, "Init done");
    s_initialized = true;
    return ESP_OK;
}

/*  
    Returns the frame buffer (image buffer)
*/
uint8_t* ov7670_get_fb()
{
    if (!s_initialized) 
    {
        return NULL;
    }
    return s_fb;
}

/*  
    Returns the frame width
*/
int ov7670_get_fb_width()
{
    if (!s_initialized) {
        return 0;
    }
    return s_fb_w;
}


/*  
    Returns the frame height
*/
int over7670_get_fb_height()
{
    if (!s_initialized) 
    {
        return 0;
    }
    return s_fb_h;
}

esp_err_t camera_run()
{
    // The camera needs to be initialized
    if (!s_initialized) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    // Start the I2S
    i2s_run(s_fb_w, s_fb_h);

    ESP_LOGI(tag, "Waiting for frame");

    // Synchronize with the end of the frame
    xSemaphoreTake(s_frame_ready, portMAX_DELAY);
    
    ESP_LOGI(tag, "Frame completed");
    
    return ESP_OK;
}

void camera_print_fb()
{
	/* Number of pixels to skip
	   in order to fit into terminal screen.
	   Assumed picture to be 80 columns wide.
	   Skip twice as more rows as they look higher.
	 */
	int pixels_to_skip = s_fb_w / 80;

    for (int ih = 0; ih < s_fb_h; ih += pixels_to_skip * 2)
    {
        for (int iw = 0; iw < s_fb_w; iw += pixels_to_skip)
        {
    	    uint8_t px = (s_fb[iw + (ih * s_fb_w)]);
    	    if      (px <  26) printf("1");
    	    else if (px <  51) printf("2");
    	    else if (px <  77) printf("3");
    	    else if (px < 102) printf("4");
    	    else if (px < 128) printf("5");
    	    else if (px < 154) printf("6");
    	    else if (px < 179) printf("7");
    	    else if (px < 205) printf("8");
    	    else if (px < 230) printf("9");
    	    else               printf("X");
        }
        printf("\n");
    }
}

static esp_err_t dma_desc_init(int line_width)
{
	/* I2S peripheral captures 16 bit of data every clock cycle,
	   even though we are only using 8 bits.
	   On top of that we need two bytes per pixel.
	 */
    // DMA buffer size
    size_t buf_size = line_width * 4; // 16 = TWO*8 bits, TWO bytes per pixel ==> x4
    
    // Configure the 2 DMA buffers
    for (int i = 0; i < 2; ++i) 
    {
        ESP_LOGI(tag, "Allocating DMA buffer #%d, size=%d", i, buf_size);

        // Allocating DMA buffer memory
        s_dma_buf[i] = (uint32_t*) malloc(buf_size);
        if (s_dma_buf[i] == NULL) 
        {
            return ESP_ERR_NO_MEM;
        }
        
        ESP_LOGV(tag, "dma_buf[%d]=%p", i, s_dma_buf[i]);

        // Sets the DMA buffer descriptor associated to the DMA buffer
        s_dma_desc[i].length = buf_size;     // size of a single DMA buf
        s_dma_desc[i].size = buf_size;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) s_dma_buf[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = NULL;
    }
    return ESP_OK;
}

static inline void i2s_conf_reset()
{
    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
            | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;

    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;

    while (I2S0.state.rx_fifo_reset_back) 
    {
        ;
    }
}

static void ov7670_i2s_init()
{
    // Configure OV7670 input GPIOs
    gpio_num_t pins[] = { 
        OV7670_PIN_D7, //D7
        OV7670_PIN_D6, //D6
        OV7670_PIN_D5, //D5
        OV7670_PIN_D4, //D4
        OV7670_PIN_D3, //D3
        OV7670_PIN_D2, //D2
        OV7670_PIN_D1, //D1
        OV7670_PIN_D0, //D0
        OV7670_PIN_VSYNC, //VSYNC
        OV7670_PIN_HREF, //HREF
        OV7670_PIN_PCLK //PCLK
        };
    gpio_config_t conf = { 
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE 
        };
    
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) 
    {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(OV7670_PIN_D0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(OV7670_PIN_D1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(OV7670_PIN_D2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(OV7670_PIN_D3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(OV7670_PIN_D4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(OV7670_PIN_D5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(OV7670_PIN_D6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(OV7670_PIN_D7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(OV7670_PIN_VSYNC, I2S0I_V_SYNC_IDX, true);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(OV7670_PIN_HREF, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(OV7670_PIN_PCLK, I2S0I_WS_IN_IDX, true);
    // false x x is infinite waiting
    // true true x only zeros
    // true false x seems not so bad

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_S | I2S_AHBM_RST_S
            | I2S_AHBM_FIFO_RST_S;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;
    // Toggle some reset bits in CONF register
    i2s_conf_reset();
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration, TBD if needed
    I2S0.fifo_conf.rx_fifo_mod = 1;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Grab 16 samples
    I2S0.sample_rate_conf.rx_bits_mod = 16;
    // Clear flags which are used in I2S serial mode
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
            ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_i2s_intr_handle);
}

static void i2s_fill_buf(int index)
{
    // Disable the interrupt
    esp_intr_disable(s_i2s_intr_handle);

    // Reset the I2S buffers ???
    i2s_conf_reset();

    // Configure some I2S data
    I2S0.rx_eof_num = s_buf_line_width;
    I2S0.in_link.addr = (uint32_t) &s_dma_desc[index];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.in_done = 1;

    // Enable I2S interrupt
    esp_intr_enable(s_i2s_intr_handle);

    I2S0.conf.rx_start = 1;
}

static void i2s_stop()
{
    // Disable the interrupt
    esp_intr_disable(s_i2s_intr_handle);
    
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;

    // I2S is not running anymore
    s_i2s_running = false;
}

static void i2s_run(size_t line_width, int height)
{
    s_buf_line_width = line_width;
    s_buf_height = height;

    // wait for vsync
    ESP_LOGI(tag, "Waiting for positive edge on VSYNC");
    while (gpio_get_level(OV7670_PIN_VSYNC) == 0) {
        ;
    }
    while (gpio_get_level(OV7670_PIN_VSYNC) != 0) {
        ;
    }
    ESP_LOGI(tag, "Got VSYNC, starting serious business");

    // start RX
    s_cur_buffer = 0;
    s_line_count = 0;
    s_isr_count = 0;
    s_i2s_running = true;

    // Fills the first line (not in ISR)
    i2s_fill_buf(s_cur_buffer);
}

static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    s_cur_buffer = !s_cur_buffer;
    
    // Check if we got to the end of the frame
    if (s_isr_count == s_buf_height) 
    {
        i2s_stop();
    }
    else 
    {
        // Fills one row
        i2s_fill_buf(s_cur_buffer);
        ++s_isr_count;
    }

    BaseType_t xHigherPriorityTaskWoken;
    // s_data_ready semaphore unblock
    xSemaphoreGiveFromISR(s_data_ready, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken != pdFALSE) 
    {
        portYIELD_FROM_ISR();
    }
}

static void frame_buffer_fill_row_task(void *pvParameters)
{
    int prev_buf = -1;

    while (true) 
    {
        // Synchronize with the readiness of s_data_ready (DMA buffer full)
        xSemaphoreTake(s_data_ready, portMAX_DELAY);
        
        int buf_idx = !s_cur_buffer;
        if (prev_buf != -1 && prev_buf == buf_idx) 
        {
            ets_printf("! %d\n", s_line_count);
        }
        
        // Points to the next buffer slot to be written
        uint8_t* pfb = s_fb + s_line_count * s_buf_line_width;
        
        // Get pointer to the current DMA buffer
        const uint32_t* buf = s_dma_buf[buf_idx];

        // Copy the current line DMA buffer to the Image buffer
        for (int i = 0; i < s_buf_line_width; ++i) 
        {
            // Get 32 bit from DMA buffer
            // 1 Pixel = (2Byte i2s overhead + 2Byte pixeldata)
            uint32_t v = *buf;
            // Extract third byte (only the important information from the pixel)
            uint8_t comp = (v & 0xff0000) >> 16;
            
            // Write byte to target buffer
            *pfb = comp;

            // Set source pointer (DMA) 32 bit forward
            ++buf;
            // Set target pointer (Image buffer) 8 bit forward
            ++pfb;
        }
        
        // Next line
        ++s_line_count;
        prev_buf = buf_idx;
        
        if (!s_i2s_running) 
        {
            prev_buf = -1;
            xSemaphoreGive(s_frame_ready);
        }
    }
}
