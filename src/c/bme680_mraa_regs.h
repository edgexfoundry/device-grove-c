/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/* Based on code from https://github.com/BoschSensortec/BME680_driver */
/**\mainpage
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 */

#pragma once

/** Macro to combine two 8 bit data's to form a 16 bit data */
#define BME680_CONCAT_BYTES(msb, lsb)  (((uint16_t)msb << 8) | (uint16_t)lsb)

/** Macro to SET and GET BITS of a register */
#define BME680_SET_BITS(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
#define BME680_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

/** Macro variant to handle the bitname position if it is zero */
#define BME680_SET_BITS_POS_0(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))
#define BME680_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

//BME680
#define BME680_DEFAULT_I2C_BUS 0
#define BME680_DEFAULT_ADDR 0x76
#define BME680_CHIPID 0x61

/** Soft reset command */
#define BME680_SOFT_RESET_CMD   0xb6

/** Over-sampling settings */
#define BME680_OS_NONE    0
#define BME680_OS_1X    1
#define BME680_OS_2X    2
#define BME680_OS_4X    3
#define BME680_OS_8X    4
#define BME680_OS_16X    5

/** IIR filter settings */
#define BME680_FILTER_SIZE_0  0
#define BME680_FILTER_SIZE_1  1
#define BME680_FILTER_SIZE_3  2
#define BME680_FILTER_SIZE_7  3
#define BME680_FILTER_SIZE_15  4
#define BME680_FILTER_SIZE_31  5
#define BME680_FILTER_SIZE_63  6
#define BME680_FILTER_SIZE_127  7

/** Settings selector */
#define BME680_OST_SEL      UINT16_C(1)
#define BME680_OSP_SEL      UINT16_C(2)
#define BME680_OSH_SEL      UINT16_C(4)
#define BME680_GAS_MEAS_SEL    UINT16_C(8)
#define BME680_FILTER_SEL    UINT16_C(16)
#define BME680_HCNTRL_SEL    UINT16_C(32)
#define BME680_RUN_GAS_SEL    UINT16_C(64)
#define BME680_NBCONV_SEL    UINT16_C(128)
#define BME680_GAS_SENSOR_SEL    (BME680_GAS_MEAS_SEL | BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)

/** Number of conversion settings*/
#define BME680_NBCONV_MIN    UINT8_C(0)
#define BME680_NBCONV_MAX    UINT8_C(10)

/** BME680 coefficients related defines */
#define BME680_COEFF_SIZE  41
#define BME680_COEFF_ADDR1_LEN  25
#define BME680_COEFF_ADDR2_LEN  16

/** Register map */
/** Other coefficient's address */
#define BME680_ADDR_RES_HEAT_VAL_ADDR  0x00
#define BME680_ADDR_RES_HEAT_RANGE_ADDR  0x02
#define BME680_ADDR_RANGE_SW_ERR_ADDR  0x04
#define BME680_ADDR_SENS_CONF_START  0x5A
#define BME680_ADDR_GAS_CONF_START  0x64

/** BME680 field_x related defines */
#define BME680_FIELD_LENGTH    (15)
#define BME680_FIELD_ADDR_OFFSET  (17)

/** BME680 register buffer index settings*/
#define BME680_REG_FILTER_INDEX    UINT8_C(5)
#define BME680_REG_TEMP_INDEX    UINT8_C(4)
#define BME680_REG_PRES_INDEX    UINT8_C(4)
#define BME680_REG_HUM_INDEX    UINT8_C(2)
#define BME680_REG_NBCONV_INDEX    UINT8_C(1)
#define BME680_REG_RUN_GAS_INDEX  UINT8_C(1)
#define BME680_REG_HCTRL_INDEX    UINT8_C(0)

/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG  (1)
#define BME680_T2_MSB_REG  (2)
#define BME680_T3_REG    (3)
#define BME680_P1_LSB_REG  (5)
#define BME680_P1_MSB_REG  (6)
#define BME680_P2_LSB_REG  (7)
#define BME680_P2_MSB_REG  (8)
#define BME680_P3_REG    (9)
#define BME680_P4_LSB_REG  (11)
#define BME680_P4_MSB_REG  (12)
#define BME680_P5_LSB_REG  (13)
#define BME680_P5_MSB_REG  (14)
#define BME680_P7_REG    (15)
#define BME680_P6_REG    (16)
#define BME680_P8_LSB_REG  (19)
#define BME680_P8_MSB_REG  (20)
#define BME680_P9_LSB_REG  (21)
#define BME680_P9_MSB_REG  (22)
#define BME680_P10_REG    (23)
#define BME680_H2_MSB_REG  (25)
#define BME680_H2_LSB_REG  (26)
#define BME680_H1_LSB_REG  (26)
#define BME680_H1_MSB_REG  (27)
#define BME680_H3_REG    (28)
#define BME680_H4_REG    (29)
#define BME680_H5_REG    (30)
#define BME680_H6_REG    (31)
#define BME680_H7_REG    (32)
#define BME680_T1_LSB_REG  (33)
#define BME680_T1_MSB_REG  (34)
#define BME680_GH2_LSB_REG  (35)
#define BME680_GH2_MSB_REG  (36)
#define BME680_GH1_REG    (37)
#define BME680_GH3_REG    (38)

/** Ambient humidity shift value for compensation */
#define BME680_HUM_REG_SHIFT_VAL  4

/** Heater control settings */
#define BME680_ENABLE_HEATER    0x00
#define BME680_DISABLE_HEATER    0x08

/** Run gas enable and disable settings */
#define BME680_RUN_GAS_DISABLE  0
#define BME680_RUN_GAS_ENABLE  1

/** Register map */

/** Field settings */
#define BME680_FIELD0_ADDR    UINT8_C(0x1d)

/** Heater settings */
#define BME680_RES_HEAT0_ADDR    UINT8_C(0x5a)
#define BME680_GAS_WAIT0_ADDR    UINT8_C(0x64)

/** Sensor configuration registers */
#define BME680_CONF_HEAT_CTRL_ADDR    UINT8_C(0x70)
#define BME680_CONF_ODR_RUN_GAS_NBC_ADDR  UINT8_C(0x71)
#define BME680_CONF_OS_H_ADDR      UINT8_C(0x72)
#define BME680_MEM_PAGE_ADDR      UINT8_C(0xf3)
#define BME680_CONF_T_P_MODE_ADDR    UINT8_C(0x74)
#define BME680_CONF_ODR_FILT_ADDR    UINT8_C(0x75)

/** Coefficient's address */
#define BME680_COEFF_ADDR1  UINT8_C(0x89)
#define BME680_COEFF_ADDR2  UINT8_C(0xe1)

/** Chip identifier */
#define BME680_CHIP_ID_ADDR  UINT8_C(0xd0)

/** Soft reset register */
#define BME680_SOFT_RESET_ADDR    UINT8_C(0xe0)

/** Gas measurement settings */
#define BME680_DISABLE_GAS_MEAS    UINT8_C(0x00)
#define BME680_ENABLE_GAS_MEAS    UINT8_C(0x01)

/** Buffer length macro declaration */
#define BME680_TMP_BUFFER_LENGTH  UINT8_C(40)
#define BME680_REG_BUFFER_LENGTH  UINT8_C(6)
#define BME680_FIELD_DATA_LENGTH  UINT8_C(3)
#define BME680_GAS_REG_BUF_LENGTH  UINT8_C(20)

/** Settings selector */
#define BME680_OST_SEL      UINT16_C(1)
#define BME680_OSP_SEL      UINT16_C(2)
#define BME680_OSH_SEL      UINT16_C(4)
#define BME680_GAS_MEAS_SEL    UINT16_C(8)
#define BME680_FILTER_SEL    UINT16_C(16)
#define BME680_HCNTRL_SEL    UINT16_C(32)
#define BME680_RUN_GAS_SEL    UINT16_C(64)
#define BME680_NBCONV_SEL    UINT16_C(128)
#define BME680_GAS_SENSOR_SEL    (BME680_GAS_MEAS_SEL | BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)

/** Number of conversion settings*/
#define BME680_NBCONV_MIN    UINT8_C(0)
#define BME680_NBCONV_MAX    UINT8_C(10)

/** Mask definitions */
#define BME680_GAS_MEAS_MSK  UINT8_C(0x30)
#define BME680_NBCONV_MSK  UINT8_C(0X0F)
#define BME680_FILTER_MSK  UINT8_C(0X1C)
#define BME680_OST_MSK    UINT8_C(0XE0)
#define BME680_OSP_MSK    UINT8_C(0X1C)
#define BME680_OSH_MSK    UINT8_C(0X07)
#define BME680_HCTRL_MSK  UINT8_C(0x08)
#define BME680_RUN_GAS_MSK  UINT8_C(0x10)
#define BME680_MODE_MSK    UINT8_C(0x03)
#define BME680_RHRANGE_MSK  UINT8_C(0x30)
#define BME680_RSERROR_MSK  UINT8_C(0xf0)
#define BME680_NEW_DATA_MSK  UINT8_C(0x80)
#define BME680_GAS_INDEX_MSK  UINT8_C(0x0f)
#define BME680_GAS_RANGE_MSK  UINT8_C(0x0f)
#define BME680_GASM_VALID_MSK  UINT8_C(0x20)
#define BME680_HEAT_STAB_MSK  UINT8_C(0x10)
#define BME680_MEM_PAGE_MSK  UINT8_C(0x10)
#define BME680_SPI_RD_MSK  UINT8_C(0x80)
#define BME680_SPI_WR_MSK  UINT8_C(0x7f)
#define  BME680_BIT_H1_DATA_MSK  UINT8_C(0x0F)

/** Bit position definitions for sensor settings */
#define BME680_GAS_MEAS_POS  UINT8_C(4)
#define BME680_FILTER_POS  UINT8_C(2)
#define BME680_OST_POS    UINT8_C(5)
#define BME680_OSP_POS    UINT8_C(2)
#define BME680_RUN_GAS_POS  UINT8_C(4)

typedef enum
{
  BME680_SLEEP_MODE = 0,
  BME680_FORCED_MODE = 1
} BME680_MODES_T;

typedef enum
{
  BME680_OK = 0,

  BME680_E_NULL_PTR = -1,
  BME680_E_COM_FAIL = -2,
  BME680_E_DEV_NOT_FOUND = -3,
  BME680_E_INVALID_LENGTH = -4,

  BME680_W_DEFINE_PWR_MODE = 1,
  BME680_W_NO_NEW_DATA = 2
} bme680_results_t;

