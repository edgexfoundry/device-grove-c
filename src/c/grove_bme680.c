/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
/* Based on code from https://github.com/BoschSensortec/BME680_driver - Self Test */
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


#include <assert.h>
#include <unistd.h>

#include "device_grove.h"
#include "grove_bme680.h"

bme680_context grove_bme680_initialize (int bus, int addr, float temp_offset)
{
  bme680_results_t result = BME680_OK;
  uint16_t settings_sel;
  bme680_context tph_dev = bme680_init (bus, addr);
  assert (tph_dev != NULL);

  if (tph_dev != NULL)
  {
    /* Set the temperature, pressure and humidity & filter settings */
    tph_dev->tph_sett.os_hum = BME680_OS_1X;
    tph_dev->tph_sett.os_pres = BME680_OS_16X;
    tph_dev->tph_sett.os_temp = BME680_OS_2X;

    /* Set the remaining gas sensor settings and link the heating profile */
    tph_dev->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;

    settings_sel = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;

    result = bme680_set_sensor_settings (settings_sel, tph_dev);
    assert (result == BME680_OK);

    /* Note: Set the temperature offset as the sensor readings are high - source: Issue as in https://github.com/pimoroni/bme680-python/issues/20 */
    bme680_set_temp_offset (tph_dev, temp_offset);

    if (result != BME680_OK)
    {
      tph_dev = NULL;
    }
  }
  return tph_dev;
}

bool grove_bme_read_data (bme680_context tph_dev, struct bme680_field_data *field_data)
{
  bme680_results_t result = BME680_OK;
  uint16_t profile_dur = 0;

  bme680_get_profile_dur (&profile_dur, tph_dev);

  tph_dev->mode = BME680_FORCED_MODE;
  result = bme680_set_sensor_mode (tph_dev);

  /* Set the sensor mode to read data and wait before measuring values */
  usleep (profile_dur * 1000);

  result = bme680_get_sensor_data (tph_dev, field_data);
  if (result != BME680_OK)
  {
    return false;
  }
  else
  {
    return true;
  }
}
