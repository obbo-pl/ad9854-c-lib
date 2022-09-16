/*
 * ad9854_spi.c
 *
 * Created on: 20 sie 2021
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2021 Krzysztof Markiewicz
 */

#include "ad9854_spi.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <esp_timer.h>


#define AD9854_SPI_RESET_DELAY_US       10

void ad9854_delay_microseconds(uint32_t us);


spi_device_handle_t spi;
bool external_update_enabled = false;

esp_err_t ad9854_spi_init()
{
    esp_err_t result;

    result = gpio_set_level(AD9854_SPI_IO_MASTER_RESET, 0);
    if (result != ESP_OK) goto finish;
    result = gpio_set_level(AD9854_SPI_RST, 0);
    if (result != ESP_OK) goto finish;
    gpio_config_t io_conf;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;
    io_conf.intr_type = GPIO_INTR_DISABLE ;
    io_conf.mode = GPIO_MODE_OUTPUT ;
    io_conf.pin_bit_mask = (1ULL << AD9854_SPI_IO_MASTER_RESET) | (1ULL << AD9854_SPI_RST);
    result = gpio_config(&io_conf);
    if (result != ESP_OK) goto finish;
    result = ad9854_spi_set_external_update(false);
    if (result != ESP_OK) goto finish;
    //
    spi_bus_config_t buscfg = {
        .miso_io_num = AD9854_SPI_MISO,
        .mosi_io_num = AD9854_SPI_MOSI,
        .sclk_io_num = AD9854_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPI_TRANS_MODE_DIO
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = AD9854_SPI_FREQ_HZ,
        .mode = 0,
        .spics_io_num = AD9854_SPI_CS,
        .queue_size = 4,
        .flags = SPI_DEVICE_3WIRE |SPI_DEVICE_HALFDUPLEX
    };
    // Initialize the SPI bus
    result = spi_bus_initialize(AD9854_SPI, &buscfg, AD9854_SPI_DMA_CHAN);
    if (result == ESP_OK) {
        result = spi_bus_add_device(AD9854_SPI, &devcfg, &spi);
    }
finish:
    return result;
}

esp_err_t ad9854_spi_set_gpio(uint8_t io, uint8_t level)
{
    return gpio_set_level(io, level);
}

esp_err_t ad9854bus_spi_send_cmd(uint8_t cmd)
{
    esp_err_t result;
    result = gpio_set_level(AD9854_SPI_RST, 1);
    if (result == ESP_OK) {
        ad9854_delay_microseconds(AD9854_SPI_RESET_DELAY_US);
        result = gpio_set_level(AD9854_SPI_RST, 0);
        if (result == ESP_OK) {
            ad9854_delay_microseconds(AD9854_SPI_RESET_DELAY_US);
            spi_transaction_t t;
            memset(&t, 0, sizeof(t));
            t.length = 8;
            t.tx_buffer = &cmd;
            result = spi_device_polling_transmit(spi, &t);
        }
    }
    return result;
}

esp_err_t ad9854_spi_read_reg(uint8_t cmd, uint8_t* data_rd, size_t size)
{
    esp_err_t result;
    if (size == 0) return ESP_OK;
    result = ad9854bus_spi_send_cmd(cmd);
    if (result == ESP_OK) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.rxlength = size * 8;
        t.rx_buffer = data_rd;
        result = spi_device_polling_transmit(spi, &t);
    }
    return result;
}

esp_err_t ad9854_spi_write_reg(uint8_t cmd, uint8_t* data_wr, size_t size)
{
    esp_err_t result;
    if (size == 0) return ESP_OK;
    result = ad9854bus_spi_send_cmd(cmd);
    if (result == ESP_OK) {
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = size * 8;
        t.tx_buffer = data_wr;
        result = spi_device_polling_transmit(spi, &t);
        if ((result == ESP_OK) && (external_update_enabled)) {
            ad9854_delay_microseconds(1);
            result = gpio_set_level(AD9854_SPI_IO_UPDATE_CLOCK, 1);
            if (result == ESP_OK) {
                ad9854_delay_microseconds(1);
                result = gpio_set_level(AD9854_SPI_IO_UPDATE_CLOCK, 0);
            }
        }
    }
    return result;
}

esp_err_t ad9854_spi_set_external_update(bool enable)
{
    esp_err_t result;
    result = gpio_set_level(AD9854_SPI_IO_UPDATE_CLOCK, 0);
    if (result == ESP_OK) {
        gpio_config_t io_conf;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE ;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;
        io_conf.intr_type = GPIO_INTR_DISABLE ;
        if (enable) {
            io_conf.mode = GPIO_MODE_OUTPUT ;
        } else {
            io_conf.mode = GPIO_MODE_INPUT ;
        }
        io_conf.pin_bit_mask = (1ULL << AD9854_SPI_IO_UPDATE_CLOCK);
        result = gpio_config(&io_conf);
        if (result == ESP_OK) {
            external_update_enabled = enable;
        }
    }
    return result;
}

void ad9854_delay_microseconds(uint32_t us)
{
    int64_t t0 = esp_timer_get_time();
    while((esp_timer_get_time() - t0) < us){
        asm("nop");
    }
}
