// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "sensor.h"
#include "i2c_mgr.h"
#include "wiring.h"
#include "camera.h"
#include "camera_common.h"
#include "xclk.h"
#if CONFIG_OV2640_SUPPORT
#include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
#include "ov7725.h"
#endif

#define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN

#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

static const char* tag = "camera";

camera_state_t* s_state = NULL;

const int resolution[][2] = {
        { 40, 30 }, /* 40x30 */
        { 64, 32 }, /* 64x32 */
        { 64, 64 }, /* 64x64 */
        { 88, 72 }, /* QQCIF */
        { 160, 120 }, /* QQVGA */
        { 128, 160 }, /* QQVGA2*/
        { 176, 144 }, /* QCIF  */
        { 240, 160 }, /* HQVGA */
        { 320, 240 }, /* QVGA  */
        { 352, 288 }, /* CIF   */
        { 640, 480 }, /* VGA   */
        { 800, 600 }, /* SVGA  */
        { 1280, 1024 }, /* SXGA  */
        { 1600, 1200 }, /* UXGA  */
};

static void i2s_init();
static void i2s_run();
static void IRAM_ATTR gpio_isr(void* arg);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void dma_desc_deinit();
static void dma_filter_task(void *pvParameters);
static void dma_filter_grayscale(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_grayscale_highspeed(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void dma_filter_rgb565(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void i2s_stop();

/* 

*/
static bool is_hs_mode()
{
    // Return true if frequency above 10MHz
    return s_state->config.xclk_freq_hz > 10000000;
}

/*  

*/
static size_t i2s_bytes_per_sample(i2s_sampling_mode_t mode)
{
    // I2S Sampling mode
    switch(mode) 
    {
        case SM_0A00_0B00:
            return 4;
        case SM_0A0B_0B0C:
            return 4;
        case SM_0A0B_0C0D:
            return 2;
        default:
            assert(0 && "invalid sampling mode");
            return 0;
    }
}
/*
    Disable VSYNC interrupt
*/
static void vsync_intr_disable()
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_DISABLE);
}

/*
    Enable VSYNC interrupt
*/
static void vsync_intr_enable()
{
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
}

/*
    
*/
esp_err_t camera_probe(const camera_config_t* config, camera_model_t* out_camera_model)
{
    if (s_state != NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
    if (!s_state) 
    {
        return ESP_ERR_NO_MEM;
    }

    // Configure the camera XCLK
    ESP_LOGD(tag, "Enabling XCLK output");
    camera_enable_out_clock(config);

    // Configure the SCCB (I2C mode)
    ESP_LOGD(tag, "Initializing SSCB");
    i2c_init(config->pin_i2c_sda, config->pin_i2c_scl);

    // Reset the camera 
    ESP_LOGD(tag, "Resetting camera");
    gpio_config_t conf = { 0 };
    conf.pin_bit_mask = 1LL << config->pin_reset;
    conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&conf);

    gpio_set_level(config->pin_reset, 0);
    delay(10);
    gpio_set_level(config->pin_reset, 1);
    delay(10);

    // Detects the plugged camera
    ESP_LOGD(tag, "Searching for camera address");
    /* Probe the sensor */
    delay(10);

    // Read all 127 I2C addresses
    // Suppose there is only 1 I2C device connected
    // TODO : to change and only check for OV7670
    uint8_t slv_addr = SCCB_Probe();
    if (slv_addr == 0) 
    {
        *out_camera_model = CAMERA_NONE;
        return ESP_ERR_CAMERA_NOT_DETECTED;
    }
    s_state->sensor.slv_addr = slv_addr;
    ESP_LOGD(tag, "Detected camera at address=0x%02x", slv_addr);

    // Write the Camera IDs
    sensor_id_t* id = &s_state->sensor.id;
    id->PID = SCCB_Read(slv_addr, REG_PID);
    id->VER = SCCB_Read(slv_addr, REG_VER);
    id->MIDL = SCCB_Read(slv_addr, REG_MIDL);
    id->MIDH = SCCB_Read(slv_addr, REG_MIDH);
    delay(10);
    ESP_LOGD(tag, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
            id->PID, id->VER, id->MIDH, id->MIDL);

    // TODO: support for OV7670
    switch (id->PID) 
    {
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            *out_camera_model = CAMERA_OV2640;
            ov2640_init(&s_state->sensor);
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            *out_camera_model = CAMERA_OV7725;
            ov7725_init(&s_state->sensor);
            break;
#endif
        default:
            id->PID = 0;
            *out_camera_model = CAMERA_UNKNOWN;
            ESP_LOGD(tag, "Detected camera not supported.");
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    // Reset and write all default register 
    // TODO : clean, but needed?
    ESP_LOGD(tag, "Doing SW reset of sensor");
    s_state->sensor.reset(&s_state->sensor);

    return ESP_OK;
}

/*
    Initialize the camera pins, format, framesize, ...
*/
esp_err_t camera_init(const camera_config_t* config)
{
    // Check if the sensor has been created
    if (!s_state) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    // Check whether the sensor is supported
    if (s_state->sensor.id.PID == 0) 
    {
        return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }

    // Variable init
    memcpy(&s_state->config, config, sizeof(*config));
    esp_err_t err = ESP_OK;
    framesize_t frame_size = (framesize_t) config->frame_size;
    pixformat_t pix_format = (pixformat_t) config->pixel_format;
    s_state->width = resolution[frame_size][0];
    s_state->height = resolution[frame_size][1];
    // s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

    ////// TO DO : REPLACE the frame size init by a datasheet-compliant init

    // Set the frame size
    ESP_LOGD(tag, "Setting frame size to %dx%d", s_state->width, s_state->height);
    // if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
    //     ESP_LOGE(tag, "Failed to set frame size");
    //     err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
    //     goto fail;
    // }
    // s_state->sensor.set_pixformat(&s_state->sensor, pix_format);

#if ENABLE_TEST_PATTERN
    /* Test pattern may get handy
     if you are unable to get the live image right.
     Once test pattern is enable, sensor will output
     vertical shaded bars instead of live image.
     */
    s_state->sensor.set_colorbar(&s_state->sensor, 1);
    ESP_LOGD(tag, "Test pattern enabled");
#endif

    // Set the pixel format 
    if (pix_format == PIXFORMAT_GRAYSCALE) 
    {
        if (s_state->sensor.id.PID != OV7725_PID) 
        {
            ESP_LOGE(tag, "Grayscale format is only supported for ov7225");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        s_state->fb_size = s_state->width * s_state->height;
        if (is_hs_mode()) 
        {
            s_state->sampling_mode = SM_0A0B_0B0C;
            s_state->dma_filter = &dma_filter_grayscale_highspeed;
        } 
        else 
        {
            s_state->sampling_mode = SM_0A0B_0C0D;
            s_state->dma_filter = &dma_filter_grayscale;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends YUYV
        s_state->fb_bytes_per_pixel = 1;       // frame buffer stores Y8
    } 
    else if (pix_format == PIXFORMAT_RGB565) 
    {
        if (s_state->sensor.id.PID != OV7725_PID) 
        {
            ESP_LOGE(tag, "RGB565 format is only supported for ov7225");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        s_state->fb_size = s_state->width * s_state->height * 3;
        if (is_hs_mode()) 
        {
            s_state->sampling_mode = SM_0A0B_0B0C;
        } 
        else 
        {
            s_state->sampling_mode = SM_0A00_0B00;
        }
        s_state->in_bytes_per_pixel = 2;       // camera sends RGB565 (2 bytes)
        s_state->fb_bytes_per_pixel = 3;       // frame buffer stores RGB888
        s_state->dma_filter = &dma_filter_rgb565;

    } 
    else if (pix_format == PIXFORMAT_JPEG) 
    {
        if (s_state->sensor.id.PID != OV2640_PID) 
        {
            ESP_LOGE(tag, "JPEG format is only supported for ov2640");
            err = ESP_ERR_NOT_SUPPORTED;
            goto fail;
        }
        int qp = config->jpeg_quality;
        int compression_ratio_bound;
        if (qp >= 30) 
        {
            compression_ratio_bound = 5;
        } 
        else if (qp >= 10) 
        {
            compression_ratio_bound = 10;
        } 
        else 
        {
            compression_ratio_bound = 20;
        }

        (*s_state->sensor.set_quality)(&s_state->sensor, qp);
        size_t equiv_line_count = s_state->height / compression_ratio_bound;
        s_state->fb_size = s_state->width * equiv_line_count * 2 /* bpp */;
        s_state->dma_filter = &dma_filter_jpeg;
        if (is_hs_mode()) {
            s_state->sampling_mode = SM_0A0B_0B0C;
        } 
        else 
        {
            s_state->sampling_mode = SM_0A00_0B00;
        }
        s_state->in_bytes_per_pixel = 2;
        s_state->fb_bytes_per_pixel = 2;
    } 
    else 
    {
        ESP_LOGE(tag, "Requested format is not supported");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    ESP_LOGD(tag, "in_bpp: %d, fb_bpp: %d, fb_size: %d, mode: %d, width: %d height: %d",
            s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel,
            s_state->fb_size, s_state->sampling_mode,
            s_state->width, s_state->height);

    // Allocate the frame buffer
    ESP_LOGD(tag, "Allocating frame buffer (%d bytes)", s_state->fb_size);
    s_state->fb = (uint8_t*) calloc(s_state->fb_size, 1);
    if (s_state->fb == NULL) 
    {
        ESP_LOGE(tag, "Failed to allocate frame buffer");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    // Initialize the I2S and the DMA
    ESP_LOGD(tag, "Initializing I2S and DMA");
    // I2S init
    i2s_init();
    // DMA init
    err = dma_desc_init();
    if (err != ESP_OK) 
    {
        ESP_LOGE(tag, "Failed to initialize I2S and DMA");
        goto fail;
    }

    // Semamphore init
    s_state->data_ready = xQueueCreate(16, sizeof(size_t));
    s_state->frame_ready = xSemaphoreCreateBinary();
    if (s_state->data_ready == NULL || s_state->frame_ready == NULL) 
    {
        ESP_LOGE(tag, "Failed to create semaphores");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    // Launch the DMA task, attach it to the core n°1
    if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1)) {
        ESP_LOGE(tag, "Failed to create DMA filter task");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    // Initialize VSYNC interrupt
    ESP_LOGD(tag, "Initializing GPIO interrupts");
    vsync_intr_disable();
    err = gpio_isr_handler_add(s_state->config.pin_vsync, &gpio_isr, NULL);
    if (err != ESP_OK) 
    {
        ESP_LOGE(tag, "gpio_isr_handler_add failed (%x)", err);
        goto fail;
    }

    // skip at least one frame after changing camera settings
    while (gpio_get_level(s_state->config.pin_vsync) == 0) 
    {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) != 0) 
    {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) == 0) 
    {
        ;
    }

    s_state->frame_count = 0;
    ESP_LOGD(tag, "Init done");
    return ESP_OK;

// In case initailization fails, deinit the camera
fail:
    camera_deinit();
    return err;
}

/*
    Camera de-init
*/
esp_err_t camera_deinit()
{
    if (s_state == NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state->dma_filter_task) 
    {
        vTaskDelete(s_state->dma_filter_task);
    }
    if (s_state->data_ready) 
    {
        vQueueDelete(s_state->data_ready);
    }
    if (s_state->frame_ready) 
    {
        vSemaphoreDelete(s_state->frame_ready);
    }
    gpio_isr_handler_remove(s_state->config.pin_vsync);
    if (s_state->i2s_intr_handle) 
    {
        esp_intr_disable(s_state->i2s_intr_handle);
        esp_intr_free(s_state->i2s_intr_handle);
    }
    dma_desc_deinit();
    free(s_state->fb);
    free(s_state);
    s_state = NULL;
    camera_disable_out_clock();
    periph_module_disable(PERIPH_I2S0_MODULE);
    return ESP_OK;
}

/*
    Returns the frame buffer
*/
uint8_t* camera_get_fb()
{
    if (s_state == NULL) {
        return NULL;
    }
    return s_state->fb;
}

/*
    Return the frame width
*/
int camera_get_fb_width()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->width;
}

/*
    Return the frame height
*/
int camera_get_fb_height()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->height;
}

/*
    
*/
size_t camera_get_data_size()
{
    if (s_state == NULL) {
        return 0;
    }
    return s_state->data_size;
}

/*
    Launch the camera recording
*/
esp_err_t camera_run()
{
    // Check if the sensor has been initialized
    if (s_state == NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    // Time of the day storage - start of frame processing
    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);

#ifndef _NDEBUG
    memset(s_state->fb, 0, s_state->fb_size);
#endif // _NDEBUG

    // 
    i2s_run();


    ESP_LOGD(tag, "Waiting for frame");
    // Sync with the frame ready signal
    xSemaphoreTake(s_state->frame_ready, portMAX_DELAY);

    //  Time of the day storage - end of frame processing
    struct timeval tv_end;
    gettimeofday(&tv_end, NULL);

    // Check how much time we need to process one frame
    int time_ms = (tv_end.tv_sec - tv_start.tv_sec) * 1000 + (tv_end.tv_usec - tv_start.tv_usec) / 1000;
    ESP_LOGI(tag, "Frame %d done in %d ms", s_state->frame_count, time_ms);

    // Increment the frame count
    s_state->frame_count++;

    return ESP_OK;
}

/*
    Initialize the DMA
*/
static esp_err_t dma_desc_init()
{
    // ??
    assert(s_state->width % 4 == 0);

    // DMA line width if the frame width * number of bytes per pixel * number of DMA buffers depending on its structure 
    size_t line_size = s_state->width * s_state->in_bytes_per_pixel *
            i2s_bytes_per_sample(s_state->sampling_mode);
    ESP_LOGD(tag, "Line width (for DMA): %d bytes", line_size);
    size_t dma_per_line = 1;
    size_t buf_size = line_size;

    // Split the DMA buffer if above 4096
    while (buf_size >= 4096) 
    {
        buf_size /= 2;
        dma_per_line *= 2;
    }

    // ??
    size_t dma_desc_count = dma_per_line * 4;
    s_state->dma_buf_width = line_size;
    s_state->dma_per_line = dma_per_line;
    s_state->dma_desc_count = dma_desc_count;
    ESP_LOGD(tag, "DMA buffer size: %d, DMA buffers per line: %d", buf_size, dma_per_line);
    ESP_LOGD(tag, "DMA buffer count: %d", dma_desc_count);

    // Allocate memory for the DMA buffers pointers
    s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
    if (s_state->dma_buf == NULL) 
    {
        return ESP_ERR_NO_MEM;
    }

    // Allocate memory for the DMA descriptor
    s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if (s_state->dma_desc == NULL) 
    {
        return ESP_ERR_NO_MEM;
    }

    //
    size_t dma_sample_count = 0;
    for (int i = 0; i < dma_desc_count; ++i) 
    {
        // Allocate memory for the DMA buffers
        ESP_LOGD(tag, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
        if (buf == NULL) 
        {
            return ESP_ERR_NO_MEM;
        }
        s_state->dma_buf[i] = buf;
        ESP_LOGV(tag, "dma_buf[%d]=%p", i, buf);

        // Fills the descriptor data
        lldesc_t* pd = &s_state->dma_desc[i];
        pd->length = buf_size;
        if (s_state->sampling_mode == SM_0A0B_0B0C &&
            (i + 1) % dma_per_line == 0) 
        {
            pd->length -= 4;
        }
        dma_sample_count += pd->length / 4;
        pd->size = pd->length;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 1;
        pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
    }
    s_state->dma_done = false;
    s_state->dma_sample_count = dma_sample_count;

    return ESP_OK;
}

/*
    De-init the DMA descriptors
*/
static void dma_desc_deinit()
{
    if (s_state->dma_buf) 
    {
        for (int i = 0; i < s_state->dma_desc_count; ++i) 
        {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}

/*
    Reset the I2C configuration
*/
static inline void i2s_conf_reset()
{
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M
            | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;

    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M
            | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while (I2S0.state.rx_fifo_reset_back) 
    {
        ;
    }
}

/*
    Initialize the I2S
*/
static void i2s_init()
{
    camera_config_t* config = &s_state->config;

    // Configure input GPIOs
    gpio_num_t pins[] = 
    {
            config->pin_d7,
            config->pin_d6,
            config->pin_d5,
            config->pin_d4,
            config->pin_d3,
            config->pin_d2,
            config->pin_d1,
            config->pin_d0,
            config->pin_vsync,
            config->pin_href,
            config->pin_pclk
    };

    // Configure all GPIO as input, pulled up, no interrupt
    gpio_config_t conf = 
    {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };

    // Configure every single GPIO
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) 
    {
        if (rtc_gpio_is_valid_gpio(pins[i])) 
        {
            rtc_gpio_deinit(pins[i]);
        }
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false); // TODO: POLARITY CHECK
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false); // TODO: POLARITY CHECK
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
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
    // FIFO configuration
    I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_state->i2s_intr_handle);
}

/*
    Stop the I2S
*/
static void i2s_stop()
{
    // Disable interrupt
    esp_intr_disable(s_state->i2s_intr_handle);
    vsync_intr_disable();

    // Reset I2S
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;

    size_t val = SIZE_MAX;
    BaseType_t higher_priority_task_woken;
    xQueueSendFromISR(s_state->data_ready, &val, &higher_priority_task_woken);
}

/*
    Start the I2S HW block for recording camera data
*/
static void i2s_run()
{
#ifndef _NDEBUG
    for (int i = 0; i < s_state->dma_desc_count; ++i) {
        lldesc_t* d = &s_state->dma_desc[i];
        ESP_LOGV(tag, "DMA desc %2d: %u %u %u %u %u %u %p %p",
                i, d->length, d->size, d->offset, d->eof, d->sosf, d->owner, d->buf, d->qe.stqe_next);
        memset(s_state->dma_buf[i], 0, d->length);
    }
#endif

    // Wait for vsync
    ESP_LOGD(tag, "Waiting for positive pulse on VSYNC");
    while (gpio_get_level(s_state->config.pin_vsync) == 0) 
    {
        ;
    }
    while (gpio_get_level(s_state->config.pin_vsync) != 0) 
    {
        ;
    }
    ESP_LOGD(tag, "Got VSYNC");

    // Reset frame information
    s_state->dma_done = false;
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    s_state->dma_filtered_count = 0;
    esp_intr_disable(s_state->i2s_intr_handle);
    i2s_conf_reset();

    // Set the I2S registers
    I2S0.rx_eof_num = s_state->dma_sample_count;
    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;

    // Enable the I2S interrupt
    esp_intr_enable(s_state->i2s_intr_handle);

    // If in JPEG mode, enable VSYNC interrupt
    if (s_state->config.pixel_format == CAMERA_PF_JPEG) {
        vsync_intr_enable();
    }

    // Start the I2S HW block
    I2S0.conf.rx_start = 1;
}

/*
    Switch to the next available DMA buffer and signal that we are ready to process the current buffer
*/
static void IRAM_ATTR signal_dma_buf_received(bool* need_yield)
{
    size_t dma_desc_filled = s_state->dma_desc_cur;

    // Switching to the next DMA buffer
    s_state->dma_desc_cur = (dma_desc_filled + 1) % s_state->dma_desc_count;
    s_state->dma_received_count++;

    // Signal sent that the DMA buffer is ready to be processed
    BaseType_t higher_priority_task_woken;
    BaseType_t ret = xQueueSendFromISR(s_state->data_ready, &dma_desc_filled, &higher_priority_task_woken);
    
    if (ret != pdTRUE) 
    {
        ESP_EARLY_LOGW(tag, "queue send failed (%d), dma_received_count=%d", ret, s_state->dma_received_count);
    }

    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}

/*
    
*/
static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    bool need_yield;

    // Switch to the next DMA buffer and signal the data is ready to be processed 
    signal_dma_buf_received(&need_yield);

    ESP_EARLY_LOGV(tag, "isr, cnt=%d", s_state->dma_received_count);

    // check if we received the entire frame
    if (s_state->dma_received_count == s_state->height * s_state->dma_per_line) 
    {
        i2s_stop();
    }

    // Check if we had an issue yiedling the data_ready semaphore
    if (need_yield) 
    {
        portYIELD_FROM_ISR();
    }
}

/*
    
*/
static void IRAM_ATTR gpio_isr(void* arg)
{
    bool need_yield = false;
    ESP_EARLY_LOGV(tag, "gpio isr, cnt=%d", s_state->dma_received_count);

    if (gpio_get_level(s_state->config.pin_vsync) == 0 &&
            s_state->dma_received_count > 0 &&
            !s_state->dma_done) 
    {
        signal_dma_buf_received(&need_yield);
        i2s_stop();
    }
    
    // Check if we had an issue yiedling the data_ready semaphore
    if (need_yield) 
    {
        portYIELD_FROM_ISR();
    }
}

/*
    Return the frame current pixel position
*/
static size_t get_fb_pos()
{
    return s_state->dma_filtered_count * s_state->width *
            s_state->fb_bytes_per_pixel / s_state->dma_per_line;
}

/*
    Background task processing the DMA buffers
*/
static void IRAM_ATTR dma_filter_task(void *pvParameters)
{
    while (true) 
    {
        // Sync the semaphore
        size_t buf_idx;
        xQueueReceive(s_state->data_ready, &buf_idx, portMAX_DELAY);

        // If we reached the end of the frame, send the signal we did and exit the while loop
        if (buf_idx == SIZE_MAX) 
        {
            s_state->data_size = get_fb_pos();
            xSemaphoreGive(s_state->frame_ready);
            continue;
        }

        // Position in the frame buffer
        size_t fb_pos = get_fb_pos();
        assert(fb_pos <= s_state->fb_size + s_state->width *
                s_state->fb_bytes_per_pixel / s_state->dma_per_line);

        // Get the pointer to the frame buffer line to register
        uint8_t* pfb = s_state->fb + fb_pos;

        // Get the DMA buffer pointer
        const dma_elem_t* buf = s_state->dma_buf[buf_idx];

        // Get the DMA buffer description
        lldesc_t* desc = &s_state->dma_desc[buf_idx];

        // Start the associated DMA filter function????
        (*s_state->dma_filter)(buf, desc, pfb);

        // Increment the DMA filtered buffer counter
        s_state->dma_filtered_count++;
        ESP_LOGV(tag, "dma_flt: flt_count=%d ", s_state->dma_filtered_count);
    }
}

/*
    
*/
static void IRAM_ATTR dma_filter_grayscale(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0C0D);
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    for (size_t i = 0; i < end; ++i) {
        // manually unrolling 4 iterations of the loop here
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[3].sample1; 
        src += 4;
        dst += 4;
    }
}

/*
    
*/
static void IRAM_ATTR dma_filter_grayscale_highspeed(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0B0C);
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 8;
    for (size_t i = 0; i < end; ++i) {
        // manually unrolling 4 iterations of the loop here
        dst[0] = src[0].sample1;
        dst[1] = src[2].sample1;
        dst[2] = src[4].sample1;
        dst[3] = src[6].sample1;
        src += 8;
        dst += 4;
    }
    // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if ((dma_desc->length & 0x7) != 0) {
        dst[0] = src[0].sample1;
        dst[1] = src[2].sample1;
    }
}

/*
    
*/
static void IRAM_ATTR dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0B0C ||
           s_state->sampling_mode == SM_0A00_0B00 );
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    // manually unrolling 4 iterations of the loop here
    for (size_t i = 0; i < end; ++i) {
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[3].sample1;
        src += 4;
        dst += 4;
    }
    // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if ((dma_desc->length & 0x7) != 0) {
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[2].sample2;
    }
}

/*
    
*/
static inline void rgb565_to_888(uint8_t in1, uint8_t in2, uint8_t* dst)
{
    dst[0] = (in2 & 0b00011111) << 3; // blue
    dst[1] = ((in1 & 0b111) << 5) | ((in2 & 0b11100000 >> 5)); // green
    dst[2] = in1 & 0b11111000; // red
}

/*
    
*/
static void IRAM_ATTR dma_filter_rgb565(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst)
{
    assert(s_state->sampling_mode == SM_0A0B_0B0C ||
           s_state->sampling_mode == SM_0A00_0B00);

    const int unroll = 2;         // manually unrolling 2 iterations of the loop
    const int samples_per_pixel = 2;
    const int bytes_per_pixel = 3;
    size_t end = dma_desc->length / sizeof(dma_elem_t) / unroll / samples_per_pixel;
    for (size_t i = 0; i < end; ++i) {
        rgb565_to_888(src[0].sample1, src[1].sample1, &dst[0]);
        rgb565_to_888(src[2].sample1, src[3].sample1, &dst[3]);
        dst += bytes_per_pixel * unroll;
        src += samples_per_pixel * unroll;
    }
    if ((dma_desc->length & 0x7) != 0) {
        rgb565_to_888(src[0].sample1, src[1].sample1, &dst[0]);
        rgb565_to_888(src[2].sample1, src[2].sample2, &dst[3]);
    }
}
