/*
 * ad9854.h
 *
 * Created on: 20 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#ifndef _AD9854_H_
#define _AD9854_H_


#include "ad9854_def.h"
#include <stdbool.h>
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif



typedef enum ad9854_variant ad9854_variant_t;
typedef enum ad9854_mode ad9854_mode_t;



typedef struct {
    ad9854_variant_t variant;
    uint32_t control_reg;
    uint32_t clkin_frequency;
    bool initialised;
} ad9854_t;



#if ARDUINO >= 100
#include <Arduino.h>
#include "ad9854_spi.h"

typedef uint8_t ad9854_err_t;

#define AD9854_OK                         0x00
#define AD9854_ERR_TIMEOUT                0x11
#define AD9854_ERR_INVALID_STATE          0x12
#define AD9854_ERR_NOT_INITIALISED        0x13
#define AD9854_ERR_INVALID_ARG            0x14


#define ad9854_delay_msec(x)        do {            \
        delay(x);                                   \
    } while(0)

inline ad9854_err_t ad9854_set_gpio_master_reset(uint8_t level)
{
    if (level) level = 0x01;
    return (ad9854_err_t)ad9854_spi_set_gpio(AD9854_SPI_IO_MASTER_RESET, level);
}

inline ad9854_err_t ad9854_read_reg(uint8_t cmd, uint8_t *data, uint8_t count)
{
    cmd &= AD9854_INSTRUCTION_MASK;
    cmd |= AD9854_READ_TRANSFER;
    return (ad9854_err_t)ad9854_spi_read_reg(cmd, data, count);
}

inline ad9854_err_t ad9854_write_reg(uint8_t cmd, uint8_t *data, uint8_t count)
{
    cmd &= AD9854_INSTRUCTION_MASK;
    return (ad9854_err_t)ad9854_spi_write_reg(cmd, data, count);
}

#elif defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "ad9854_spi.h"


typedef esp_err_t ad9854_err_t;

#define AD9854_OK                         ESP_OK
#define AD9854_ERR_TIMEOUT                ESP_ERR_TIMEOUT
#define AD9854_ERR_INVALID_STATE          ESP_ERR_INVALID_STATE
#define AD9854_ERR_NOT_INITIALISED        ESP_ERR_INVALID_STATE
#define AD9854_ERR_INVALID_ARG            ESP_ERR_INVALID_ARG


#define ad9854_delay_msec(x)        do {            \
        vTaskDelay((x) / portTICK_PERIOD_MS);       \
    } while(0)


inline ad9854_err_t ad9854_set_gpio_master_reset(uint8_t level)
{
    if (level) level = 0x01;
    return (ad9854_err_t)ad9854_spi_set_gpio(AD9854_SPI_IO_MASTER_RESET, level);
}

inline ad9854_err_t ad9854_read_reg(uint8_t cmd, uint8_t *data, uint8_t count)
{
    cmd &= AD9854_INSTRUCTION_MASK;
    cmd |= AD9854_READ_TRANSFER;
    return (ad9854_err_t)ad9854_spi_read_reg(cmd, data, count);
}

inline ad9854_err_t ad9854_write_reg(uint8_t cmd, uint8_t *data, uint8_t count)
{
    cmd &= AD9854_INSTRUCTION_MASK;
    return (ad9854_err_t)ad9854_spi_write_reg(cmd, data, count);
}

#else
// Here you can put functions specific to your framework
#endif


ad9854_err_t ad9854_init(ad9854_variant_t variant, uint32_t clkin_frequency, bool unbreakable);
ad9854_err_t ad9854_reset_master();
ad9854_err_t ad9854_set_mode(ad9854_mode_t mode);
ad9854_err_t ad9854_set_powerdown(bool comparator, bool q_dac, bool full_dac, bool digital);
ad9854_err_t ad9854_set_pll(bool bypass, uint8_t multiplier);
ad9854_err_t sd9854_set_frequency_1(uint32_t frequency);
ad9854_err_t sd9854_set_frequency_2(uint32_t frequency);
ad9854_err_t ad9854_set_amplitude_i(uint16_t amplitude);
ad9854_err_t ad9854_set_amplitude_q(uint16_t amplitude);
ad9854_err_t ad9854_set_phase_1(uint16_t phase);
ad9854_err_t ad9854_set_phase_2(uint16_t phase);
ad9854_err_t ad9854_set_osk_rate_counter(uint8_t counter);
ad9854_err_t ad9854_set_amplitude_control(bool enable, bool osk_int);
ad9854_err_t ad9854_set_inverse_sinc_filter(bool enable);
ad9854_err_t ad9854_set_update_clock_counter(uint32_t counter);
ad9854_err_t ad9854_set_external_update_clock(bool enable);



#ifdef __cplusplus
}
#endif


#endif // _AD9854_H_
