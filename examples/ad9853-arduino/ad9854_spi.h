/*
 * ad9854_spi.h
 *
 * Created on: 21 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#ifndef _AD9854_SPI_H_
#define _AD9854_SPI_H_

#include <Arduino.h>


#define AD9854_SPI_MOSI                23
#define AD9854_SPI_CLK                 19
#define AD9854_SPI_CS                  22
#define AD9854_SPI_RST                 18

#define AD9854_SPI_IO_MASTER_RESET     5
#define AD9854_SPI_IO_UPDATE_CLOCK     4


#ifdef __cplusplus
extern "C" {
#endif


uint8_t ad9854_spi_init();
uint8_t ad9854_spi_set_gpio(uint8_t io, uint8_t level);
uint8_t ad9854_spi_read_reg(uint8_t cmd, uint8_t* data_rd, uint8_t size);
uint8_t ad9854_spi_write_reg(uint8_t cmd, uint8_t* data_wr, uint8_t size);
uint8_t ad9854_spi_set_external_update(bool enable);


#ifdef __cplusplus
}
#endif


#endif /* _AD9854_SPI_H_ */
