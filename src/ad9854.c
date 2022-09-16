/*
 * ad9854.c
 *
 * Created on: 20 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#include "ad9854.h"
#include <byteswap.h>
#include <inttypes.h>


// function prototype
uint64_t ad9854_get_frequency_tuning_word(uint32_t frequency);
ad9854_err_t ad9854_write_control_reg();
ad9854_err_t ad9854_read_control_reg();
uint8_t ad9854_get_clkin_multiplier();


ad9854_t chip = {
        .initialised = false
};



ad9854_err_t ad9854_init(ad9854_variant_t variant, uint32_t clkin_frequency, bool unbreakable)
{
    ad9854_err_t result = AD9854_ERR_INVALID_ARG;
    chip.variant = variant;
    uint32_t f_max = 0;
    switch (chip.variant) {
    case AD9854AST:
    case AD9854ASTZ:
        f_max = AD9854_CLKIN_FREQUENCY_AST_MAX;
        break;
    case AD9854ASQ:
        f_max = AD9854_CLKIN_FREQUENCY_ASQ_MAX;
        break;
    case AD9854ASVZ:
        f_max = AD9854_CLKIN_FREQUENCY_ASV_MAX;
        break;
    default:
        result = AD9854_ERR_INVALID_ARG;
    }
    if ((clkin_frequency >= AD9854_CLKIN_FREQUENCY_MIN) && (clkin_frequency <= f_max)) {
        result = AD9854_OK;
        chip.clkin_frequency = clkin_frequency;
        if (unbreakable) {
            // get current configuration
            result = ad9854_read_control_reg();
        } else {
            // reset to default
            result = ad9854_reset_master();
            chip.control_reg = AD9854_CONTROL_DEFAULT_VALUE;
        }
        if (result == AD9854_OK) chip.initialised = true;
    }
	return result;
}

ad9854_err_t ad9854_reset_master()
{
    ad9854_err_t result;
    ad9854_delay_msec(10);
    result = ad9854_set_gpio_master_reset(1);
    if (result == AD9854_OK) {
        ad9854_delay_msec(10);
        result = ad9854_set_gpio_master_reset(0);
        ad9854_delay_msec(10);
    }
    return result;
}

ad9854_err_t ad9854_write_control_reg()
{
    ad9854_err_t result = AD9854_OK;
    uint32_t data = __bswap_32(chip.control_reg);
    result = ad9854_write_reg(AD9854_CONTROL, (uint8_t*)&data, AD9854_CONTROL_LENGTH);
    return result;
}

ad9854_err_t ad9854_read_control_reg()
{
    ad9854_err_t result;
    uint32_t data;
    result = ad9854_read_reg(AD9854_CONTROL, (uint8_t*)&data, AD9854_CONTROL_LENGTH);
    chip.control_reg = __bswap_32(data);
    return result;
}

uint8_t ad9854_get_clkin_multiplier()
{
    return (uint8_t)((chip.control_reg & AD9854_CONTROL_REFCLK_MULTIP_bm) >> AD9854_CONTROL_REFCLK_MULTIP_bp);
}

ad9854_err_t ad9854_set_mode(ad9854_mode_t mode)
{
    ad9854_err_t result;
    chip.control_reg &= ~(AD9854_CONTROL_MODE_bm);
    chip.control_reg |= ((mode << AD9854_CONTROL_MODE_bp) & AD9854_CONTROL_MODE_bm);
    result = ad9854_write_control_reg();
    return result;
}

ad9854_err_t ad9854_set_pll(bool bypass, uint8_t multiplier)
{
    ad9854_err_t result;
    if (chip.initialised) {
        if (multiplier == 1) {
            bypass = true;
        } else {
            if (multiplier < AD9854_REFCLK_MULTIP_MIN) multiplier = AD9854_REFCLK_MULTIP_MIN;
            if (multiplier > AD9854_REFCLK_MULTIP_MAX) multiplier = AD9854_REFCLK_MULTIP_MAX;
        }
        if (bypass) {
            chip.control_reg |= AD9854_CONTROL_PLL_BYPASS_bm;
            multiplier = 1;
        } else {
            chip.control_reg &= ~(AD9854_CONTROL_PLL_BYPASS_bm);
        }
        chip.control_reg &= ~(AD9854_CONTROL_REFCLK_MULTIP_bm | AD9854_CONTROL_PLL_RANGE_bm);
        chip.control_reg |= (multiplier << AD9854_CONTROL_REFCLK_MULTIP_bp) & AD9854_CONTROL_REFCLK_MULTIP_bm;
        if (chip.clkin_frequency * multiplier >= AD9854_PLL_RANGE_BIT_FREQUENCY) {
            chip.control_reg |= AD9854_CONTROL_PLL_RANGE_bm;
        }
        result = ad9854_write_control_reg();
    } else {
        result = AD9854_ERR_NOT_INITIALISED;
    }
    return result;
}

ad9854_err_t ad9854_set_powerdown(bool comparator, bool q_dac, bool full_dac, bool digital)
{
    ad9854_err_t result;
    if (comparator) {
        chip.control_reg |= AD9854_CONTROL_POWER_DOWN_COMP_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_POWER_DOWN_COMP_bm);
    }
    if (q_dac) {
        chip.control_reg |= AD9854_CONTROL_POWER_DOWN_QDAC_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_POWER_DOWN_QDAC_bm);
    }
    if (full_dac) {
        chip.control_reg |= AD9854_CONTROL_POWER_DOWN_DAC_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_POWER_DOWN_DAC_bm);
    }
    if (digital) {
        chip.control_reg |= AD9854_CONTROL_POWER_DOWN_DIG_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_POWER_DOWN_DIG_bm);
    }
    result = ad9854_write_control_reg();
    return result;
}

uint64_t ad9854_get_frequency_tuning_word(uint32_t frequency)
{
    uint8_t shift = 32;
    uint64_t ft = (uint64_t)frequency << shift;
    uint8_t div = AD9854_PHASE_ACCUMULATOR_RESOLUTION - shift;
    while (div) {
        ft <<= 1;
        div--;
        if (ft >= 0x8000000000000000) break;
    }
    uint32_t sysclk = (uint32_t)(chip.clkin_frequency * ad9854_get_clkin_multiplier()) >> div;
    ft /= sysclk;
    return ft;
}

ad9854_err_t sd9854_set_frequency_1(uint32_t frequency)
{
    ad9854_err_t result;
    if (chip.initialised) {
        if (frequency > (chip.clkin_frequency * ad9854_get_clkin_multiplier()) / 2) {
            frequency = (chip.clkin_frequency * ad9854_get_clkin_multiplier()) / 2;
        }
        uint64_t ft = ad9854_get_frequency_tuning_word(frequency);
        ft = __bswap_64(ft);
        ft = ft >> 16;
        result = ad9854_write_reg(AD9854_FREQUENCY_TUNING_1, (uint8_t*)&ft, AD9854_FREQUENCY_TUNING_1_LENGTH);
    } else {
        result = AD9854_ERR_NOT_INITIALISED;
    }
    return result;
}

ad9854_err_t sd9854_set_frequency_2(uint32_t frequency)
{
    ad9854_err_t result;
    if (chip.initialised) {
        if (frequency > (chip.clkin_frequency * ad9854_get_clkin_multiplier()) / 2) {
            frequency = (chip.clkin_frequency * ad9854_get_clkin_multiplier()) / 2;
        }
        uint64_t ft = ad9854_get_frequency_tuning_word(frequency);
        ft = __bswap_64(ft);
        ft = ft >> 16;
        result = ad9854_write_reg(AD9854_FREQUENCY_TUNING_2, (uint8_t*)&ft, AD9854_FREQUENCY_TUNING_2_LENGTH);
    } else {
        result = AD9854_ERR_NOT_INITIALISED;
    }
    return result;
}

ad9854_err_t ad9854_set_amplitude_i(uint16_t amplitude)
{
    ad9854_err_t result;
    if (amplitude > AD9854_I_PATH_MULTIPLIER_MASK) amplitude = AD9854_I_PATH_MULTIPLIER_MASK;
    amplitude = __bswap_16(amplitude);
    result = ad9854_write_reg(AD9854_I_PATH_MULTIPLIER, (uint8_t*)&amplitude, AD9854_I_PATH_MULTIPLIER_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_amplitude_q(uint16_t amplitude)
{
    ad9854_err_t result;
    if (amplitude > AD9854_Q_PATH_MULTIPLIER_MASK) amplitude = AD9854_Q_PATH_MULTIPLIER_MASK;
    amplitude = __bswap_16(amplitude);
    result = ad9854_write_reg(AD9854_Q_PATH_MULTIPLIER, (uint8_t*)&amplitude, AD9854_Q_PATH_MULTIPLIER_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_phase_1(uint16_t phase)
{
    ad9854_err_t result;
    if (phase > AD9854_PHASE_OFFSET_1_MASK) phase = AD9854_PHASE_OFFSET_1_MASK;
    phase = __bswap_16(phase);
    result = ad9854_write_reg(AD9854_PHASE_OFFSET_1, (uint8_t*)&phase, AD9854_PHASE_OFFSET_1_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_phase_2(uint16_t phase)
{
    ad9854_err_t result;
    if (phase > AD9854_PHASE_OFFSET_2_MASK) phase = AD9854_PHASE_OFFSET_2_MASK;
    phase = __bswap_16(phase);
    result = ad9854_write_reg(AD9854_PHASE_OFFSET_2, (uint8_t*)&phase, AD9854_PHASE_OFFSET_2_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_osk_rate_counter(uint8_t counter)
{
    ad9854_err_t result;
    result = ad9854_write_reg(AD9854_SHAPED_KEYING_RAMP, &counter, AD9854_SHAPED_KEYING_RAMP_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_amplitude_control(bool enable, bool osk_pin)
{
    ad9854_err_t result;
    if (enable) {
        chip.control_reg |= AD9854_CONTROL_OSK_EN_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_OSK_EN_bm);
    }
    if (osk_pin) {
        chip.control_reg |= AD9854_CONTROL_OSK_INT_bm;
    } else {
        chip.control_reg &= ~(AD9854_CONTROL_OSK_INT_bm);
    }
    result = ad9854_write_control_reg();
    return result;
}

ad9854_err_t ad9854_set_inverse_sinc_filter(bool enable)
{
    ad9854_err_t result;
    if (enable) {
        chip.control_reg &= ~(AD9854_CONTROL_BYPASS_INV_SINC_bm);
    } else {
        chip.control_reg |= AD9854_CONTROL_BYPASS_INV_SINC_bm;
    }
    result = ad9854_write_control_reg();
    return result;
}

ad9854_err_t ad9854_set_update_clock_counter(uint32_t counter)
{
    ad9854_err_t result;
    counter = __bswap_32(counter);
    result = ad9854_write_reg(AD9854_UPDATE_CLOCK_RATE, (uint8_t*)&counter, AD9854_UPDATE_CLOCK_RATE_LENGTH);
    return result;
}

ad9854_err_t ad9854_set_external_update_clock(bool enable)
{
    ad9854_err_t result;
    if (enable) {
        chip.control_reg &= ~(AD9854_CONTROL_UPDATE_CLOCK_bm);
    } else {
        chip.control_reg |= AD9854_CONTROL_UPDATE_CLOCK_bm;
    }
    result = ad9854_write_control_reg();
    return result;
}
