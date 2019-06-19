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

#include <string.h>
#include <assert.h>

#include "bme680_mraa.h"

static bme680_results_t bme680_write_reg (const bme680_context dev, uint8_t reg, uint8_t val)
{
  mraa_result_t status = MRAA_SUCCESS;
  bme680_results_t result = BME680_OK;
  assert (dev != NULL);

  status = mraa_i2c_write_byte_data (dev->i2c, val, reg);
  if (status)
  {
    result = BME680_E_COM_FAIL;
  }
  return result;
}

static bme680_results_t bme680_read_regs (const bme680_context dev, uint8_t reg, uint16_t len, uint8_t *buffer)
{
  bme680_results_t result = BME680_OK;
  assert (dev != NULL);

  if (mraa_i2c_read_bytes_data (dev->i2c, reg, buffer, len) != len)
  {
    result = BME680_E_COM_FAIL;
  }
  return result;
}

/*!
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @note heatr_temp and heatr_dur values are currently register data
 * and not the actual values set
 */
static bme680_results_t get_gas_config (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  /* starting address of the register array for burst read*/
  uint8_t reg_addr1 = BME680_ADDR_SENS_CONF_START;
  uint8_t reg_addr2 = BME680_ADDR_GAS_CONF_START;
  uint8_t reg_data = 0;

  /* Check for null pointer in the device structure*/
  assert (dev != NULL);

  result = bme680_read_regs (dev, reg_addr1, 1, &reg_data);
  assert (result == BME680_OK);
  dev->gas_sett.heatr_temp = reg_data;

  result = bme680_read_regs (dev, reg_addr2, 1, &reg_data);
  assert (result == BME680_OK);
  /* Heating duration register value */
  dev->gas_sett.heatr_dur = reg_data;

  return result;
}

/*!
 * @brief This internal API is used to calculate the
 * temperature value in float format
 */
static float calc_temperature (uint32_t temp_adc, const bme680_context dev)
{
  float var1 = 0;
  float var2 = 0;
  float calc_temp = 0;

  /* calculate var1 data */
  var1 = ((((float) temp_adc / 16384.0f) - ((float) dev->calib.par_t1 / 1024.0f)) * ((float) dev->calib.par_t2));

  /* calculate var2 data */
  var2 = (((((float) temp_adc / 131072.0f) - ((float) dev->calib.par_t1 / 8192.0f)) *
           (((float) temp_adc / 131072.0f) - ((float) dev->calib.par_t1 / 8192.0f))) *
          ((float) dev->calib.par_t3 * 16.0f));

  /* t_fine value*/
  dev->calib.t_fine = (var1 + var2) + (float) (dev->calib.offset_temp_in_t_fine);

  /* compensated temperature data*/
  calc_temp = ((dev->calib.t_fine) / 5120.0f);
  return calc_temp;
}

/*!
 * @brief This internal API is used to calculate the
 * pressure value in float format
 */
static float calc_pressure (uint32_t pres_adc, const bme680_context dev)
{
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float calc_pres = 0;

  var1 = (((float) dev->calib.t_fine / 2.0f) - 64000.0f);
  var2 = var1 * var1 * (((float) dev->calib.par_p6) / (131072.0f));
  var2 = var2 + (var1 * ((float) dev->calib.par_p5) * 2.0f);
  var2 = (var2 / 4.0f) + (((float) dev->calib.par_p4) * 65536.0f);
  var1 = (((((float) dev->calib.par_p3 * var1 * var1) / 16384.0f)
           + ((float) dev->calib.par_p2 * var1)) / 524288.0f);
  var1 = ((1.0f + (var1 / 32768.0f)) * ((float) dev->calib.par_p1));
  calc_pres = (1048576.0f - ((float) pres_adc));

  /* Avoid exception caused by division by zero */
  if ((int) var1 != 0)
  {
    calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
    var1 = (((float) dev->calib.par_p9) * calc_pres * calc_pres) / 2147483648.0f;
    var2 = calc_pres * (((float) dev->calib.par_p8) / 32768.0f);
    var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f)
            * (dev->calib.par_p10 / 131072.0f));
    calc_pres = (calc_pres + (var1 + var2 + var3 + ((float) dev->calib.par_p7 * 128.0f)) / 16.0f);
  }
  else
  {
    calc_pres = 0;
  }
  return calc_pres;
}

/*!
 * @brief This internal API is used to calculate the
 * humidity value in float format
 */
static float calc_humidity (uint16_t hum_adc, const bme680_context dev)
{
  float calc_hum = 0;
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float var4 = 0;
  float temp_comp;

  /* compensated temperature data*/
  temp_comp = ((dev->calib.t_fine) / 5120.0f);

  var1 = (float) ((float) hum_adc) - (((float) dev->calib.par_h1 * 16.0f) + (((float) dev->calib.par_h3 / 2.0f)
                                                                             * temp_comp));

  var2 = var1 * ((float) (((float) dev->calib.par_h2 / 262144.0f) * (1.0f + (((float) dev->calib.par_h4 / 16384.0f)
                                                                             * temp_comp) +
                                                                     (((float) dev->calib.par_h5 / 1048576.0f) *
                                                                      temp_comp * temp_comp))));

  var3 = (float) dev->calib.par_h6 / 16384.0f;

  var4 = (float) dev->calib.par_h7 / 2097152.0f;

  calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

  if (calc_hum > 100.0f)
  {
    calc_hum = 100.0f;
  }
  else if (calc_hum < 0.0f)
  {
    calc_hum = 0.0f;
  }

  return calc_hum;
}

/*!
 * @brief This internal API is used to calculate the
 * gas resistance value in float format
 */
static float calc_gas_resistance (uint16_t gas_res_adc, uint8_t gas_range, const bme680_context dev)
{
  float calc_gas_res;
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;

  const float lookup_k1_range[16] = {
    0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8,
    0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
  const float lookup_k2_range[16] = {
    0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8,
    -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  var1 = (1340.0f + (5.0f * dev->calib.range_sw_err));
  var2 = (var1) * (1.0f + lookup_k1_range[gas_range] / 100.0f);
  var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);

  calc_gas_res = 1.0f / (float) (var3 * (0.000000125f) * (float) (1 << gas_range) * (((((float) gas_res_adc)
                                                                                       - 512.0f) / var2) + 1.0f));
  return calc_gas_res;
}

/*!
 * @brief This internal API is used to calculate the
 * heater resistance value in float format
 */
static float calc_heater_res (uint16_t temp, const bme680_context dev)
{
  float var1 = 0;
  float var2 = 0;
  float var3 = 0;
  float var4 = 0;
  float var5 = 0;
  float res_heat = 0;

  if (temp > 400)
  { /* Cap temperature */
    temp = 400;
  }

  var1 = (((float) dev->calib.par_gh1 / (16.0f)) + 49.0f);
  var2 = ((((float) dev->calib.par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
  var3 = ((float) dev->calib.par_gh3 / (1024.0f));
  var4 = (var1 * (1.0f + (var2 * (float) temp)));
  var5 = (var4 + (var3 * (float) dev->amb_temp));
  res_heat = (uint8_t) (3.4f * ((var5 * (4 / (4 + (float) dev->calib.res_heat_range)) *
                                 (1 / (1 + ((float) dev->calib.res_heat_val * 0.002f)))) - 25));
  return res_heat;
}

/*!
 * @brief This internal API is used to calculate the Heat duration value.
 */
static uint8_t calc_heater_dur (uint16_t dur)
{
  uint8_t factor = 0;
  uint8_t durval;

  if (dur >= 0xfc0)
  {
    durval = 0xff; /* Max duration*/
  }
  else
  {
    while (dur > 0x3F)
    {
      dur = dur / 4;
      factor += 1;
    }
    durval = (uint8_t) (dur + (factor * 64));
  }
  return durval;
}

static bme680_results_t bme680_read_field_data (struct bme680_field_data *data, const bme680_context dev)
{
  // Ref: bme680.c - read_field_data() from BoschSensorTec
  bme680_results_t result = BME680_OK;
  uint8_t buff[BME680_FIELD_LENGTH] = {0};
  uint8_t gas_range;
  uint32_t adc_temp;
  uint32_t adc_pres;
  uint16_t adc_hum;
  uint16_t adc_gas_res;
  uint8_t tries = 5;

  assert (dev != NULL);

  do
  {
    result = bme680_read_regs (dev, ((uint8_t) (BME680_FIELD0_ADDR)), (uint16_t) BME680_FIELD_LENGTH, buff);
    assert (result == BME680_OK);

    data->status = buff[0] & BME680_NEW_DATA_MSK;
    data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
    data->meas_index = buff[1];

    /* read the raw data from the sensor */
    adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16) | ((uint32_t) buff[4] / 16));
    adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16) | ((uint32_t) buff[7] / 16));
    adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
    adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
    gas_range = buff[14] & BME680_GAS_RANGE_MSK;

    data->status |= buff[14] & BME680_GASM_VALID_MSK;
    data->status |= buff[14] & BME680_HEAT_STAB_MSK;

    if (data->status & BME680_NEW_DATA_MSK)
    {
      data->temperature = calc_temperature (adc_temp, dev);
      data->pressure = calc_pressure (adc_pres, dev);
      data->humidity = calc_humidity (adc_hum, dev);
      data->gas_resistance = calc_gas_resistance (adc_gas_res, gas_range, dev);
      break;
    }
    usleep (10000); //10ms
  } while (--tries);

  if (!tries)
  {
    result = BME680_W_NO_NEW_DATA;
  }

  return result;
}

static bme680_results_t set_gas_config (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  assert (dev != NULL);
  uint8_t reg_addr = 0;
  uint8_t reg_data = 0;

  if (dev->mode == BME680_FORCED_MODE)
  {
    reg_addr = BME680_RES_HEAT0_ADDR;
    reg_data = calc_heater_res (dev->gas_sett.heatr_temp, dev);

    result = bme680_write_reg (dev, reg_addr, reg_data);
    assert (result == BME680_OK);

    reg_addr = BME680_GAS_WAIT0_ADDR;
    reg_data = calc_heater_dur (dev->gas_sett.heatr_dur);
    dev->gas_sett.nb_conv = 0;

    result |= bme680_write_reg (dev, reg_addr, reg_data);
    assert (result == BME680_OK);
  }
  return result;
}


/*!
 * @brief This internal API is used to validate the boundary
 * conditions.
 */
static bme680_results_t boundary_check (uint8_t *value, uint8_t min, uint8_t max, const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  if (value != NULL)
  {
    /* Check if value is below minimum value */
    if (*value < min)
    {
      /* Auto correct the invalid value to minimum value */
      *value = min;
    }
    /* Check if value is above maximum value */
    if (*value > max)
    {
      /* Auto correct the invalid value to maximum value */
      *value = max;
    }
  }
  else
  {
    result = BME680_E_NULL_PTR;
  }
  return result;
}

static bme680_results_t bme680_get_calibration_data (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  uint8_t coeff_array[BME680_COEFF_SIZE] = {0};
  uint8_t temp_var = 0;

  assert (dev != NULL);

  result = bme680_read_regs (dev, BME680_COEFF_ADDR1, BME680_COEFF_ADDR1_LEN, coeff_array);
  assert (result == BME680_OK);

  /* Append the second half in the same array */
  result |= bme680_read_regs (dev, BME680_COEFF_ADDR2, BME680_COEFF_ADDR2_LEN, &coeff_array[BME680_COEFF_ADDR1_LEN]);
  assert (result == BME680_OK);

  /* Temperature related coefficients */
  dev->calib.par_t1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG], coeff_array[BME680_T1_LSB_REG]));
  dev->calib.par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG], coeff_array[BME680_T2_LSB_REG]));
  dev->calib.par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);

  /* Pressure related coefficients */
  dev->calib.par_p1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG], coeff_array[BME680_P1_LSB_REG]));
  dev->calib.par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG], coeff_array[BME680_P2_LSB_REG]));
  dev->calib.par_p3 = (int8_t) coeff_array[BME680_P3_REG];
  dev->calib.par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG], coeff_array[BME680_P4_LSB_REG]));
  dev->calib.par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG], coeff_array[BME680_P5_LSB_REG]));
  dev->calib.par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
  dev->calib.par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
  dev->calib.par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG], coeff_array[BME680_P8_LSB_REG]));
  dev->calib.par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG], coeff_array[BME680_P9_LSB_REG]));
  dev->calib.par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);

  /* Humidity related coefficients */
  dev->calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
                                  | (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
  dev->calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
                                  | ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
  dev->calib.par_h3 = (int8_t) coeff_array[BME680_H3_REG];
  dev->calib.par_h4 = (int8_t) coeff_array[BME680_H4_REG];
  dev->calib.par_h5 = (int8_t) coeff_array[BME680_H5_REG];
  dev->calib.par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
  dev->calib.par_h7 = (int8_t) coeff_array[BME680_H7_REG];

  /* Gas heater related coefficients */
  dev->calib.par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
  dev->calib.par_gh2 = (int16_t) (
    BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG], coeff_array[BME680_GH2_LSB_REG]));
  dev->calib.par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];

  result |= bme680_read_regs (dev, BME680_ADDR_RES_HEAT_RANGE_ADDR, 1, &temp_var);
  assert (result == BME680_OK);

  dev->calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);

  result |= bme680_read_regs (dev, BME680_ADDR_RES_HEAT_VAL_ADDR, 1, &temp_var);
  assert (result == BME680_OK);

  dev->calib.res_heat_val = (int8_t) temp_var;
  result |= bme680_read_regs (dev, BME680_ADDR_RANGE_SW_ERR_ADDR, 1, &temp_var);
  assert (result == BME680_OK);

  dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;

  return result;
}

bme680_results_t bme680_reset (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  assert (dev != NULL);

  result = bme680_write_reg (dev, BME680_SOFT_RESET_ADDR, BME680_SOFT_RESET_CMD);
  usleep (5000); //5ms

  return result;
}

bme680_results_t bme680_set_sensor_mode (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  uint8_t tmp_pow_mode;
  uint8_t pow_mode = 0;
  uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;

  /* Check for null pointer in the device structure*/
  assert (dev != NULL);
  if (result == BME680_OK)
  {
    /* Call repeatedly until in sleep */
    do
    {
      result = bme680_read_regs (dev, reg_addr, 1, &tmp_pow_mode);
      assert (result == BME680_OK);

      if (result == BME680_OK)
      {
        /* Put to sleep before changing mode */
        pow_mode = (tmp_pow_mode & BME680_MODE_MSK);

        if (pow_mode != BME680_SLEEP_MODE)
        {
          tmp_pow_mode = tmp_pow_mode & (~BME680_MODE_MSK); /* Set to sleep */
          result = bme680_write_reg (dev, reg_addr, tmp_pow_mode);
          usleep (10 * 1000);
        }
      }
    } while (pow_mode != BME680_SLEEP_MODE);

    /* Already in sleep */
    if (dev->mode != BME680_SLEEP_MODE)
    {
      pow_mode = (tmp_pow_mode & ~BME680_MODE_MSK) | (dev->mode & BME680_MODE_MSK);
      if (result == BME680_OK)
      {
        result = bme680_write_reg (dev, reg_addr, pow_mode);
      }
    }
  }
  return result;
}

bme680_results_t bme680_get_sensor_mode (const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  uint8_t mode;
  assert (dev != NULL);

  result = bme680_read_regs (dev, BME680_CONF_T_P_MODE_ADDR, 1, &mode);
  dev->mode = mode & BME680_MODE_MSK;

  return result;
}

bme680_context bme680_init (int bus, int addr)
{
  bme680_results_t result = BME680_OK;
  bme680_context dev =
    (bme680_context) malloc (sizeof (struct _bme680_context));

  if (!dev)
  {
    return NULL;
  }

  memset ((void *) dev, 0, sizeof (struct _bme680_context));

  if (addr < 0)
  {
    printf ("SPI support not available\n");
    bme680_close (dev);
    dev = NULL;
  }

  if (!(dev->i2c = mraa_i2c_init (bus)))
  {
    bme680_close (dev);
    dev = NULL;
  }

  if (mraa_i2c_address (dev->i2c, addr))
  {
    bme680_close (dev);
    dev = NULL;
  }

  result = bme680_reset (dev);
  assert (result == BME680_OK);

  dev->mode = BME680_SLEEP_MODE;
  result = bme680_set_sensor_mode (dev);
  assert (result == BME680_OK);

  //read calibration data
  result = bme680_get_calibration_data (dev);
  assert (result == BME680_OK);

  if (result != BME680_OK)
  {
    bme680_close (dev);
    dev = NULL;
  }

  return dev;
}

void bme680_close (bme680_context dev)
{
  assert (dev != NULL);

  if (dev->i2c)
  {
    mraa_i2c_stop (dev->i2c);
  }

  free (dev);
}

void bme680_set_temp_offset (bme680_context dev, float temp_offset)
{
  assert (dev != NULL);

  dev->calib.offset_temp_in_t_fine = (((((int) (temp_offset) * 100) << 8) - 128) / 5);
}

bme680_results_t bme680_get_sensor_data (const bme680_context dev, struct bme680_field_data *data)
{
  bme680_results_t result = BME680_OK;
  assert (dev != NULL);

  result = bme680_read_field_data (data, dev);
  assert (result == BME680_OK);

  return result;
}

bme680_results_t bme680_set_sensor_settings (uint16_t desired_settings, const bme680_context dev)
{
  bme680_results_t result = BME680_OK;
  uint8_t reg_addr;
  uint8_t data = 0;
  uint8_t intended_power_mode = dev->mode; /* Save intended power mode */

  assert (dev != NULL);
  if (desired_settings & BME680_GAS_MEAS_SEL)
  {
    result = set_gas_config (dev);
    assert (result == BME680_OK);
  }

  dev->mode = BME680_SLEEP_MODE;
  result |= bme680_set_sensor_mode (dev);
  assert (result == BME680_OK);

  /* Selecting the filter */
  if (desired_settings & BME680_FILTER_SEL)
  {
    result |= boundary_check (&dev->tph_sett.filter, BME680_FILTER_SIZE_0, BME680_FILTER_SIZE_127, dev);
    reg_addr = BME680_CONF_ODR_FILT_ADDR;

    if (result == BME680_OK)
    {
      result = bme680_read_regs (dev, reg_addr, 1, &data);
    }

    if (desired_settings & BME680_FILTER_SEL)
    {
      data = BME680_SET_BITS(data, BME680_FILTER, dev->tph_sett.filter);
    }

    result |= bme680_write_reg (dev, reg_addr, data);
    assert (result == BME680_OK);
  }

  /* Selecting heater control for the sensor */
  if (desired_settings & BME680_HCNTRL_SEL)
  {
    result = boundary_check (&dev->gas_sett.heatr_ctrl, BME680_ENABLE_HEATER,
                             BME680_DISABLE_HEATER, dev);
    reg_addr = BME680_CONF_HEAT_CTRL_ADDR;

    if (result == BME680_OK)
    {
      result = bme680_read_regs (dev, reg_addr, 1, &data);
      assert (result == BME680_OK);
    }
    data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev->gas_sett.heatr_ctrl);

    result = bme680_write_reg (dev, reg_addr, data);
    assert (result == BME680_OK);
  }

  /* Selecting heater T,P oversampling for the sensor */
  if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL))
  {
    result = boundary_check (&dev->tph_sett.os_temp, BME680_OS_NONE, BME680_OS_16X, dev);
    reg_addr = BME680_CONF_T_P_MODE_ADDR;

    if (result == BME680_OK)
    {
      result = bme680_read_regs (dev, reg_addr, 1, &data);
    }

    if (desired_settings & BME680_OST_SEL)
    {
      data = BME680_SET_BITS(data, BME680_OST, dev->tph_sett.os_temp);
    }

    if (desired_settings & BME680_OSP_SEL)
    {
      data = BME680_SET_BITS(data, BME680_OSP, dev->tph_sett.os_pres);
    }

    result = bme680_write_reg (dev, reg_addr, data);
    assert (result == BME680_OK);
  }

  /* Selecting humidity oversampling for the sensor */
  if (desired_settings & BME680_OSH_SEL)
  {
    result = boundary_check (&dev->tph_sett.os_hum, BME680_OS_NONE, BME680_OS_16X, dev);
    reg_addr = BME680_CONF_OS_H_ADDR;

    if (result == BME680_OK)
    {
      result = bme680_read_regs (dev, reg_addr, 1, &data);
    }

    data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev->tph_sett.os_hum);

    result = bme680_write_reg (dev, reg_addr, data);
    assert (result == BME680_OK);
  }

  /* Selecting the runGas and NB conversion settings for the sensor */
  if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL))
  {
    result = boundary_check (&dev->gas_sett.run_gas, BME680_RUN_GAS_DISABLE,
                             BME680_RUN_GAS_ENABLE, dev);
    if (result == BME680_OK)
    {
      /* Validate boundary conditions */
      result = boundary_check (&dev->gas_sett.nb_conv, BME680_NBCONV_MIN,
                               BME680_NBCONV_MAX, dev);
    }

    reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;

    if (result == BME680_OK)
    {
      result |= bme680_read_regs (dev, reg_addr, 1, &data);
      assert (result == BME680_OK);
    }

    if (desired_settings & BME680_RUN_GAS_SEL)
    {
      data = BME680_SET_BITS(data, BME680_RUN_GAS, dev->gas_sett.run_gas);
    }

    if (desired_settings & BME680_NBCONV_SEL)
    {
      data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev->gas_sett.nb_conv);
    }

    result = bme680_write_reg (dev, reg_addr, data);
    assert (result == BME680_OK);
  }

  /* Restore previous intended power mode */
  dev->mode = intended_power_mode;

  return result;
}

/*!
 * @brief This API is used to get the oversampling, filter and T,P,H, gas selection
 * settings in the sensor.
 */
bme680_results_t bme680_get_sensor_settings (uint16_t desired_settings, const bme680_context dev)
{
  bme680_results_t result;
  /* starting address of the register array for burst read*/
  uint8_t reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
  uint8_t data_array[BME680_REG_BUFFER_LENGTH] = {0};

  /* Check for null pointer in the device structure*/
  assert (dev != NULL);
  result = bme680_read_regs (dev, reg_addr, BME680_REG_BUFFER_LENGTH, data_array);
  assert (result == BME680_OK);

  if (desired_settings & BME680_GAS_MEAS_SEL)
  {
    result = get_gas_config (dev);

    /* get the T,P,H ,Filter,ODR settings here */
    if (desired_settings & BME680_FILTER_SEL)
    {
      dev->tph_sett.filter = BME680_GET_BITS(data_array[BME680_REG_FILTER_INDEX],
                                             BME680_FILTER);
    }

    if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL))
    {
      dev->tph_sett.os_temp = BME680_GET_BITS(data_array[BME680_REG_TEMP_INDEX], BME680_OST);
      dev->tph_sett.os_pres = BME680_GET_BITS(data_array[BME680_REG_PRES_INDEX], BME680_OSP);
    }

    if (desired_settings & BME680_OSH_SEL)
    {
      dev->tph_sett.os_hum = BME680_GET_BITS_POS_0(data_array[BME680_REG_HUM_INDEX],
                                                   BME680_OSH);
    }

    /* get the gas related settings */
    if (desired_settings & BME680_HCNTRL_SEL)
    {
      dev->gas_sett.heatr_ctrl = BME680_GET_BITS_POS_0(data_array[BME680_REG_HCTRL_INDEX],
                                                       BME680_HCTRL);
    }

    if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL))
    {
      dev->gas_sett.nb_conv = BME680_GET_BITS_POS_0(data_array[BME680_REG_NBCONV_INDEX],
                                                    BME680_NBCONV);
      dev->gas_sett.run_gas = BME680_GET_BITS(data_array[BME680_REG_RUN_GAS_INDEX],
                                              BME680_RUN_GAS);
    }
  }
  return result;
}

/*!
 * @brief This API is used to set the profile duration of the sensor.
 */
void bme680_set_profile_dur (uint16_t duration, const bme680_context dev)
{
  uint32_t tph_dur; /* Calculate in us */
  uint32_t meas_cycles;
  uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};

  meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
  meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
  meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];

  /* TPH measurement duration */
  tph_dur = meas_cycles * UINT32_C (1963);
  tph_dur += UINT32_C (477 * 4); /* TPH switching duration */
  tph_dur += UINT32_C (477 * 5); /* Gas measurement duration */
  tph_dur += UINT32_C (500); /* Get it to the closest whole number.*/
  tph_dur /= UINT32_C (1000); /* Convert to ms */

  tph_dur += UINT32_C (1); /* Wake up duration of 1ms */
  /* The remaining time should be used for heating */
  dev->gas_sett.heatr_dur = duration - (uint16_t) tph_dur;
}

/*!
 * @brief This API is used to get the profile duration of the sensor.
 */
void bme680_get_profile_dur (uint16_t *duration, const bme680_context dev)
{
  uint32_t tph_dur; /* Calculate in us */
  uint32_t meas_cycles;
  uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};

  meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
  meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
  meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];

  /* TPH measurement duration */
  tph_dur = meas_cycles * UINT32_C (1963);
  tph_dur += UINT32_C (477 * 4); /* TPH switching duration */
  tph_dur += UINT32_C (477 * 5); /* Gas measurement duration */
  tph_dur += UINT32_C (500); /* Get it to the closest whole number.*/
  tph_dur /= UINT32_C (1000); /* Convert to ms */

  tph_dur += UINT32_C (1); /* Wake up duration of 1ms */

  *duration = (uint16_t) tph_dur;

  /* Get the gas duration only when the run gas is enabled */
  if (dev->gas_sett.run_gas)
  {
    /* The remaining time should be used for heating */
    *duration += dev->gas_sett.heatr_dur;
  }
}
