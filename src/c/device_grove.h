/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _DEVICE_GROVE_H_
#define _DEVICE_GROVE_H_ 1

#include <stdio.h>
#include <pthread.h>

#include <mraa/gpio.h>
#include <mraa/aio.h>
#include <mraa/i2c.h>
#include "edgex/edgex.h"
#include "edgex/devsdk.h"
#include "edgex/device-mgmt.h"

#if defined (__cplusplus)
extern "C" {
#endif


#define GROVE_ERR_CHECK(x) if (x.code) { fprintf (stderr, "Error: %d: %s\n", x.code, x.reason); return x.code; }
#define GROVE_SUBPLATFORM_OFFSET 512
#define GROVE_NO_GPIO_PINS 7
#define GROVE_NO_AIO_PINS 3
#define GROVE_NO_I2C_PINS 3
#define GROVE_NO_PORTS GROVE_NO_GPIO_PINS + GROVE_NO_AIO_PINS + GROVE_NO_I2C_PINS

#define GROVE_I2C_BUS 0

#define GROVE_ROTARY_MAX_ANGLE 300
#define GROVE_ADC_REF 5

#define GROVE_SVC "Device-Grove"

typedef enum
{
  GROVE_GPIO = 0,
  GROVE_PWM = 1,
  GROVE_AIO = 2,
  GROVE_I2C = 3,
  GROVE_SERIAL = 4,
  GROVE_UNKNOWN_INTF = -1
} grove_interface_type_t;

typedef struct
{
  char *pin_no;
  char *pin_type;
  char *type;
  bool normalize;
} grove_attributes_t;

typedef struct
{
  bool is_lcd;
  mraa_i2c_context dev;
  mraa_i2c_context rgb_dev;
} grove_i2c_dev_ctxt_t;

typedef struct
{
  grove_interface_type_t intf_type;
  char *pin_type;
  char *pin_number;
  void *dev_ctxt;
} grove_dev_ctxt_t;

typedef struct
{
  iot_logger_t *lc;
  edgex_device_service *svc;
  grove_dev_ctxt_t *dev[GROVE_NO_PORTS];
  pthread_mutex_t mutex;
} grove_pidriver_t;

#if defined (__cplusplus)
}
#endif
#endif
