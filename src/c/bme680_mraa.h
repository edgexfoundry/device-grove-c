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


#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <mraa/i2c.h>

#include "bme680_mraa_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct bme680_calib_data
{
  uint16_t par_h1;
  uint16_t par_h2;
  int8_t par_h3;
  int8_t par_h4;
  int8_t par_h5;
  uint8_t par_h6;
  int8_t par_h7;
  int8_t par_gh1;
  int16_t par_gh2;
  int8_t par_gh3;
  uint16_t par_t1;
  int16_t par_t2;
  int8_t par_t3;
  uint16_t par_p1;
  int16_t par_p2;
  int8_t par_p3;
  int16_t par_p4;
  int16_t par_p5;
  int8_t par_p6;
  int8_t par_p7;
  int16_t par_p8;
  int16_t par_p9;
  uint8_t par_p10;
  float t_fine;
  float offset_temp_in_t_fine;
  uint8_t res_heat_range;
  int8_t res_heat_val;
  int8_t range_sw_err;
};

/*!
 * @brief BME680 sensor settings structure which comprises of ODR,
 * over-sampling and filter settings.
 */
struct bme680_tph_sett
{
  /*! Humidity oversampling */
  uint8_t os_hum;
  /*! Temperature oversampling */
  uint8_t os_temp;
  /*! Pressure oversampling */
  uint8_t os_pres;
  /*! Filter coefficient */
  uint8_t filter;
};

/*!
 * @brief BME680 gas sensor which comprises of gas settings
 *  and status parameters
 */
struct bme680_gas_sett
{
  /*! Variable to store nb conversion */
  uint8_t nb_conv;
  /*! Variable to store heater control */
  uint8_t heatr_ctrl;
  /*! Run gas enable value */
  uint8_t run_gas;
  /*! Heater temperature value */
  uint16_t heatr_temp;
  /*! Duration profile value */
  uint16_t heatr_dur;
};

typedef struct _bme680_context
{
  mraa_i2c_context i2c;
  BME680_MODES_T mode;
  int8_t amb_temp;
  struct bme680_calib_data calib;
  struct bme680_tph_sett tph_sett;
  struct bme680_gas_sett gas_sett;
} *bme680_context;

typedef struct bme680_field_data
{
  uint8_t status;
  uint8_t gas_index;
  uint8_t meas_index;
  float temperature; /*! Temperature in degree celsius */
  float pressure; /*! Pressure in Pascal */
  float humidity; /*! Humidity in % relative humidity x1000 */
  float gas_resistance; /*! Gas resistance in Ohms */
} bme680_data;


bme680_context bme680_init (int bus, int addr);

void bme680_close (bme680_context dev);

void bme680_set_temp_offset (bme680_context dev, float temp_offset);

bme680_results_t bme680_get_sensor_data (const bme680_context dev, struct bme680_field_data *data);

bme680_results_t bme680_reset (const bme680_context dev);

bme680_results_t bme680_set_sensor_settings (uint16_t desired_settings, const bme680_context dev);

bme680_results_t bme680_get_sensor_settings (uint16_t desired_settings, const bme680_context dev);

bme680_results_t bme680_set_sensor_mode (const bme680_context dev);

bme680_results_t bme680_get_sensor_mode (const bme680_context dev);

void bme680_set_profile_dur (uint16_t duration, const bme680_context dev);

void bme680_get_profile_dur (uint16_t *duration, const bme680_context dev);

#ifdef __cplusplus
}
#endif



