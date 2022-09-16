/*
 * ad9854_spi.h
 *
 * Created on: 20 sie 2021
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2021 Krzysztof Markiewicz
 */

#ifndef _AD9854_SPI_H_
#define _AD9854_SPI_H_

#include <stdio.h>
#include <driver/gpio.h>
#include "driver/spi_master.h"



#define AD9854_SPI_MISO             21       // CONFIG_AD9854_SPI_MISO
#define AD9854_SPI_MOSI             23       // CONFIG_AD9854_SPI_MOSI
#define AD9854_SPI_CLK              19       // CONFIG_AD9854_SPI_CLK
#define AD9854_SPI_CS               22       // CONFIG_AD9854_SPI_CS
#define AD9854_SPI_RST              18       // CONFIG_AD9854_SPI_RST

#define AD9854_SPI_IO_MASTER_RESET  5        // CONFIG_AD9854_SPI_IO_MASTER_RESET
#define AD9854_SPI_IO_UPDATE_CLOCK  4        // CONFIG_AD9854_SPI_IO_UPDATE_CLOCK

#define AD9854_SPI_FREQ_HZ          100000   // CONFIG_AD9854_SPI_FREQ_HZ


#ifdef CONFIG_IDF_TARGET_ESP32
#define AD9854_SPI                  HSPI_HOST
#define AD9854_SPI_DMA_CHAN         2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define AD9854_SPI                  SPI2_HOST
#define AD9854_SPI_DMA_CHAN         AD9854_SPI

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define AD9854_SPI                  SPI2_HOST
#define AD9854_SPI_DMA_CHAN         AD9854_SPI
#endif


esp_err_t ad9854_spi_init();
esp_err_t ad9854_spi_set_gpio(uint8_t io, uint8_t level);
esp_err_t ad9854_spi_read_reg(uint8_t cmd, uint8_t* data_rd, size_t size);
esp_err_t ad9854_spi_write_reg(uint8_t cmd, uint8_t* data_wr, size_t size);
esp_err_t ad9854_spi_set_external_update(bool enable);

#endif /* _AD9854_SPI_H_ */
