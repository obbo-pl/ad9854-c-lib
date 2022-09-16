/*
 * ad9854_def.h
 *
 * Created on: 20 sie 2022
 *     Author: Krzysztof Markiewicz <obbo.pl>
 *
 * MIT License
 *
 * Copyright (c) 2022 Krzysztof Markiewicz
 */

#ifndef _AD9854_DEF_H_
#define _AD9854_DEF_H_


enum ad9854_variant {
    AD9854AST,                               // LQFP, Internal System Clock Frequency Range Max 200Mhz
    AD9854ASQ,                               // Thermally-Enhanced LQFP, Internal System Clock Frequency Range Max 300Mhz
    AD9854ASTZ,                              // LQFP, Internal System Clock Frequency Range Max 200Mhz
    AD9854ASVZ                               // TQFP_EP, Internal System Clock Frequency Range Max 300Mhz
};

#define AD9854_READ_TRANSFER                 0x80
#define AD9854_INSTRUCTION_MASK              0x0F

// Mode Selection
enum ad9854_mode {
    AD9854_MODE_SINGLE                       = 0x00,
    AD9854_MODE_FSK                          = 0x01,
    AD9854_MODE_RAMPFSK                      = 0x02,
    AD9854_MODE_CHIRP                        = 0x03,
    AD9854_MODE_BPSK                         = 0x04
};

// REFCLK Multiplier Control Register Values
#define AD9854_REFCLK_MULTIP_MIN             (4)
#define AD9854_REFCLK_MULTIP_MAX             (20)
#define AD9854_CLKIN_FREQUENCY_MIN           (5000000UL)
#define AD9854_CLKIN_FREQUENCY_AST_MAX       (50000000UL)
#define AD9854_CLKIN_FREQUENCY_ASQ_MAX       (75000000UL)
#define AD9854_CLKIN_FREQUENCY_ASV_MAX       (75000000UL)
#define AD9854_REFCLK_FREQUENCY_MIN          (20000000UL)
#define AD9854_REFCLK_FREQUENCY_AST_MAX      (200000000UL)
#define AD9854_REFCLK_FREQUENCY_ASQ_MAX      (300000000UL)
#define AD9854_REFCLK_FREQUENCY_ASV_MAX      (300000000UL)
#define AD9854_PLL_RANGE_BIT_FREQUENCY       (200000000UL)


// Phase Offset Tuning Word Register 1
#define AD9854_PHASE_OFFSET_1                0x00
#define AD9854_PHASE_OFFSET_1_LENGTH         2
#define AD9854_PHASE_OFFSET_1_MASK           0x3FFF
// Phase Offset Tuning Word Register 2
#define AD9854_PHASE_OFFSET_2                0x01
#define AD9854_PHASE_OFFSET_2_LENGTH         2
#define AD9854_PHASE_OFFSET_2_MASK           0x3FFF
// Frequency Tuning Word 1
#define AD9854_FREQUENCY_TUNING_1            0x02
#define AD9854_FREQUENCY_TUNING_1_LENGTH     6
// Frequency Tuning Word 2
#define AD9854_FREQUENCY_TUNING_2            0x03
#define AD9854_FREQUENCY_TUNING_2_LENGTH     6
// Delta frequency register
#define AD9854_DELTA_FREQUENCY               0x04
#define AD9854_DELTA_FREQUENCY_LENGTH        6
// Update clock rate register
#define AD9854_UPDATE_CLOCK_RATE             0x05
#define AD9854_UPDATE_CLOCK_RATE_LENGTH      4
// Ramp rate clock register
#define AD9854_RAMP_RATE_CLOCK               0x06
#define AD9854_RAMP_RATE_CLOCK_LENGTH        3
#define AD9854_RAMP_RATE_CLOCK_MASK          0x0FFFFF
// Control register
#define AD9854_CONTROL                       0x07
#define AD9854_CONTROL_LENGTH                4
#define AD9854_CONTROL_POWER_DOWN_COMP_bm    0x10000000
#define AD9854_CONTROL_STOP_UNTIL_MRESET_bm  0x08000000
#define AD9854_CONTROL_POWER_DOWN_QDAC_bm    0x04000000
#define AD9854_CONTROL_POWER_DOWN_DAC_bm     0x02000000
#define AD9854_CONTROL_POWER_DOWN_DIG_bm     0x01000000
#define AD9854_CONTROL_PLL_RANGE_bm          0x00400000
#define AD9854_CONTROL_PLL_BYPASS_bm         0x00200000
#define AD9854_CONTROL_REFCLK_MULTIP_bm      0x001F0000
#define AD9854_CONTROL_REFCLK_MULTIP_bp      16
#define AD9854_CONTROL_REFCLK_MULTIP_DEFAULT 4
#define AD9854_CONTROL_ACC1_CLR_bm           0x00008000
#define AD9854_CONTROL_ACC2_CLR_bm           0x00004000
#define AD9854_CONTROL_TRIANGLE_bm           0x00002000
#define AD9854_CONTROL_QDAC_SRC_bm           0x00001000
#define AD9854_CONTROL_MODE_bm               0x00000E00
#define AD9854_CONTROL_MODE_bp               9
#define AD9854_CONTROL_UPDATE_CLOCK_bm       0x00000100
#define AD9854_CONTROL_BYPASS_INV_SINC_bm    0x00000040
#define AD9854_CONTROL_OSK_EN_bm             0x00000020
#define AD9854_CONTROL_OSK_INT_bm            0x00000010
#define AD9854_CONTROL_LBS_FIRST_bm          0x00000002
#define AD9854_CONTROL_SDO_bm                0x00000001
// I path digital multiplier register
#define AD9854_I_PATH_MULTIPLIER             0x08
#define AD9854_I_PATH_MULTIPLIER_LENGTH      2
#define AD9854_I_PATH_MULTIPLIER_MASK        0x0FFF
// Q path digital multiplier register
#define AD9854_Q_PATH_MULTIPLIER             0x09
#define AD9854_Q_PATH_MULTIPLIER_LENGTH      2
#define AD9854_Q_PATH_MULTIPLIER_MASK        0x0FFF
// Shaped on/off keying ramp rate register
#define AD9854_SHAPED_KEYING_RAMP            0x0A
#define AD9854_SHAPED_KEYING_RAMP_LENGTH     1
// Q DAC register
#define AD9854_Q_DAC                         0x0B
#define AD9854_Q_DAC_LENGTH                  2
#define AD9854_Q_DAC_MASK                    0x0FFF

#define AD9854_PHASE_ACCUMULATOR_RESOLUTION  48
#define AD9854_CONTROL_DEFAULT_VALUE         0x10640120



#endif /* AD9854_DEF_H_ */
