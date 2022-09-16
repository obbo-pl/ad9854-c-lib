/*
 * ad9854-test.c
 *
 * Created on: 20 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#include <stdio.h>
#include <ctype.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <driver/gpio.h>
#include "soc/soc.h"
#include <esp_log.h>
#include "ad9854_spi.h"
#include "ad9854.h"

#include "driver/ledc.h"

// function prototype
void show_control_reg();
void clkin_generator_start(ledc_timer_t timer_num, ledc_channel_t channel_num, int out_pin, uint32_t frequency);
void clkin_generator_stop(ledc_channel_t channel_num);

/* Test setup
 * ---------------------
 * AD9854ASTZ
 * CLKIN:       5MHz
 * Mode:        Single Tone
 * Frequency:   2740 Hz
 * Amplitude:   98%
 */

static const char *TAG = "ad9854";
ad9854_err_t err;

void app_main(void)
{
    ad9854_spi_init();
	// Initialization
    clkin_generator_start(LEDC_TIMER_0, LEDC_CHANNEL_0, GPIO_NUM_15, 5000000);
    vTaskDelay(10 / portTICK_PERIOD_MS);
   	err = ad9854_init(AD9854ASTZ, 5000000, false);
   	if (err != AD9854_OK) ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(err));
   	clkin_generator_stop(LEDC_CHANNEL_0);
   	// Switch to an external update clock
   	if (ad9854_set_external_update_clock(true) == AD9854_OK) {
        clkin_generator_start(LEDC_TIMER_0, LEDC_CHANNEL_0, GPIO_NUM_15, 5000000);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ad9854_spi_set_external_update(true);
   	}
   	// Status check before configuration
   	show_control_reg();
   	//
    ad9854_set_mode(AD9854_MODE_SINGLE);
	ad9854_set_pll(false, 4);
	sd9854_set_frequency_1(2740);
 	ad9854_set_amplitude_i(0xFC0);
	ad9854_set_amplitude_q(0xFC0);
	while (1)
    {
		vTaskDelay(10000 / portTICK_PERIOD_MS);
    	printf("\n");
  		show_control_reg();
    }
}

void show_control_reg()
{
    uint8_t data[4];
	printf("AD9854 DEVICE STATUS \n");
    ad9854_read_reg(AD9854_CONTROL, data, AD9854_CONTROL_LENGTH);
    printf("   Control REG:         %02X %02X %02X %02X \n", data[0], data[1], data[2], data[3]);
}

void clkin_generator_start(ledc_timer_t timer_num, ledc_channel_t channel_num, int out_pin, uint32_t frequency)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = timer_num,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel_config = {
        .gpio_num   = out_pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = channel_num,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = timer_num,
        .duty       = 1,
        .hpoint     = 0
    };
    ledc_timer_config(&timer_config);
    ledc_channel_config(&channel_config);
}

void clkin_generator_stop(ledc_channel_t channel_num)
{
    ledc_stop(LEDC_HIGH_SPEED_MODE, channel_num, 0);
}

