/*
 * ad9854_spi.cpp
 *
 * Created on: 21 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#include "ad9854_spi.h"


#define AD9854_SPI_RESET_DELAY_US       10

// function prototype
void ad9854_spi_clk_delay();
void ad9854_delay_microseconds(uint32_t us);


bool external_update_enabled = false;

uint8_t ad9854_spi_init()
{
    digitalWrite(AD9854_SPI_IO_MASTER_RESET, LOW);
    pinMode(AD9854_SPI_IO_MASTER_RESET, OUTPUT);
    digitalWrite(AD9854_SPI_RST, LOW);
    pinMode(AD9854_SPI_RST, OUTPUT);
    digitalWrite(AD9854_SPI_CS, HIGH);
    pinMode(AD9854_SPI_CS, OUTPUT);
    pinMode(AD9854_SPI_CLK, OUTPUT);
    pinMode(AD9854_SPI_MOSI, INPUT);
    pinMode(AD9854_SPI_IO_UPDATE_CLOCK, INPUT);
    return 0;
}

uint8_t ad9854_spi_set_gpio(uint8_t io, uint8_t level)
{
    digitalWrite(AD9854_SPI_IO_MASTER_RESET, level);
    return 0;
}

uint8_t ad9854bus_spi_send_cmd(uint8_t cmd)
{
    digitalWrite(AD9854_SPI_RST, HIGH);
    ad9854_delay_microseconds(AD9854_SPI_RESET_DELAY_US);
    digitalWrite(AD9854_SPI_RST, LOW);
    ad9854_delay_microseconds(AD9854_SPI_RESET_DELAY_US);

    digitalWrite(AD9854_SPI_CS, LOW);
    digitalWrite(AD9854_SPI_MOSI, LOW);
    pinMode(AD9854_SPI_MOSI, OUTPUT);
    ad9854_spi_clk_delay();
    for (uint8_t i = 0; i < 8; i++) {
        if (cmd & 0x80)
            digitalWrite(AD9854_SPI_MOSI, HIGH);
        else
            digitalWrite(AD9854_SPI_MOSI, LOW);
        ad9854_spi_clk_delay();
        digitalWrite(AD9854_SPI_CLK, HIGH);
        cmd = cmd << 1;
        ad9854_spi_clk_delay();
        digitalWrite(AD9854_SPI_CLK, LOW);
    }
    pinMode(AD9854_SPI_MOSI, INPUT);
    digitalWrite(AD9854_SPI_CS, HIGH);
    return 0;
}

uint8_t ad9854_spi_read_reg(uint8_t cmd, uint8_t* data_rd, uint8_t size)
{
    if (size == 0) return 0;

    ad9854bus_spi_send_cmd(cmd);
    digitalWrite(AD9854_SPI_CS, LOW);
    ad9854_spi_clk_delay();
    while (size--) {
        uint8_t result = 0;
        for (uint8_t i = 0; i < 8; i++) {
            result = result << 1;
            if (digitalRead(AD9854_SPI_MOSI)) result |= 0x01;
            digitalWrite(AD9854_SPI_CLK, HIGH);
            ad9854_spi_clk_delay();
            digitalWrite(AD9854_SPI_CLK, LOW);
            ad9854_spi_clk_delay();
        }
        *data_rd = result;
        data_rd++;
    }
    digitalWrite(AD9854_SPI_CS, HIGH);
    return 0;
}

uint8_t ad9854_spi_write_reg(uint8_t cmd, uint8_t* data_wr, uint8_t size)
{
    if (size == 0) return 0;

    ad9854bus_spi_send_cmd(cmd);
    digitalWrite(AD9854_SPI_CS, LOW);
    digitalWrite(AD9854_SPI_MOSI, LOW);
    pinMode(AD9854_SPI_MOSI, OUTPUT);
    ad9854_spi_clk_delay();
    while (size--) {
        uint8_t command = *(data_wr++);
        for (uint8_t i = 0; i < 8; i++) {
            if (command & 0x80)
                digitalWrite(AD9854_SPI_MOSI, HIGH);
            else
                digitalWrite(AD9854_SPI_MOSI, LOW);
            ad9854_spi_clk_delay();
            digitalWrite(AD9854_SPI_CLK, HIGH);
            command = command << 1;
            ad9854_spi_clk_delay();
            digitalWrite(AD9854_SPI_CLK, LOW);
        }
    }
    pinMode(AD9854_SPI_MOSI, INPUT);
    digitalWrite(AD9854_SPI_CS, HIGH);
    ad9854_spi_clk_delay();
    digitalWrite(AD9854_SPI_IO_UPDATE_CLOCK, HIGH);
    ad9854_spi_clk_delay();
    digitalWrite(AD9854_SPI_IO_UPDATE_CLOCK, LOW);
    ad9854_spi_clk_delay();
   return 0;
}

uint8_t ad9854_spi_set_external_update(bool enable)
{
    digitalWrite(AD9854_SPI_IO_UPDATE_CLOCK, LOW);
    if (enable) {
        pinMode(AD9854_SPI_IO_UPDATE_CLOCK, OUTPUT);
    } else {
        pinMode(AD9854_SPI_IO_UPDATE_CLOCK, INPUT);
    }
    external_update_enabled = enable;
    return 0;
}

void ad9854_spi_clk_delay()
{
    ad9854_delay_microseconds(2);
}

void ad9854_delay_microseconds(uint32_t us)
{
    int64_t t0 = micros();
    while((micros() - t0) < us){
        asm("nop");
    }
}
