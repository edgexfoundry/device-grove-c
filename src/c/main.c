/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <stdbool.h>
#include <assert.h>
#include <signal.h>
#include <math.h>
#include <semaphore.h>

#include "device_grove.h"
#include "grove_lcd.h"
#include "grove_bme680.h"

#define GROVE_ERR_BUFSZ 1024

static sem_t grove_sem;
static float BME680_Temp_Offset = 0.0;

static void grove_inthandler (int i)
{
  sem_post (&grove_sem);
}

static grove_attributes_t *get_groveattributes (const devsdk_nvpairs *device_attr)
{
  grove_attributes_t *grove_attr = (grove_attributes_t *) malloc (sizeof (grove_attributes_t));
  for (; device_attr != NULL; device_attr = device_attr->next)
  {
    if (strcmp (device_attr->name, "Pin_Num") == 0)
    {
      grove_attr->pin_no = device_attr->value;
    }
    else if (strcmp (device_attr->name, "Interface") == 0)
    {
      grove_attr->pin_type = device_attr->value;
    }
    else if (strcmp (device_attr->name, "Type") == 0)
    {
      grove_attr->type = device_attr->value;
    }
    else if (strcmp (device_attr->name, "normalize") == 0)
    {
      grove_attr->normalize = (strcasecmp (device_attr->value, "true") == 0);
    }
  }
  return grove_attr;
}

static grove_dev_ctxt_t *grove_device_lookup (grove_pidriver_t *impln, char *pin)
{
  for (int index = 0; index < GROVE_NO_PORTS; index++)
  {
    if (impln->dev[index] != NULL)
    {
      if (strcmp (impln->dev[index]->pin_number, pin) == 0)
      {
        return impln->dev[index];
      }
    }
    else
    {
      break;
    }
  }
  return NULL;
}

static grove_dev_ctxt_t *grove_set_devctxt
  (grove_pidriver_t *impln, void *dev, char *pin, char *type)
{
  int index = 0;
  grove_dev_ctxt_t *mraa_dev = NULL;
  grove_interface_type_t interface_type = GROVE_UNKNOWN_INTF;

  for (; index < GROVE_NO_PORTS; index++)
  {
    if (impln->dev[index] == NULL)
    {
      mraa_dev = malloc (sizeof (grove_dev_ctxt_t));
      if ((strstr (pin, "I2C")) && (strcmp (type, "LCD") == 0))
      {
        grove_i2c_dev_ctxt_t *i2c_dev = malloc (sizeof (grove_i2c_dev_ctxt_t));
        i2c_dev->dev = dev;
        mraa_dev->dev_ctxt = (void *) i2c_dev;
      }
      else
      {
        mraa_dev->dev_ctxt = dev;
      }
      mraa_dev->pin_number = malloc (strlen (pin));
      strcpy (mraa_dev->pin_number, pin);

      mraa_dev->pin_type = malloc (strlen (type));
      strcpy (mraa_dev->pin_type, type);

      if (strstr (pin, "D"))
      {
        interface_type = GROVE_GPIO;
      }
      else if (strstr (pin, "A"))
      {
        interface_type = GROVE_AIO;
      }
      else if (strstr (pin, "I2C"))
      {
        interface_type = GROVE_I2C;
      }
      mraa_dev->intf_type = interface_type;

      impln->dev[index] = mraa_dev;
      break;
    }
  }
  return mraa_dev;
}

static mraa_result_t grove_gpio_init (grove_pidriver_t *impln, char *pin, char *type)
{
  mraa_result_t status = MRAA_SUCCESS;

  int pin_number = GROVE_SUBPLATFORM_OFFSET + (pin[strlen (pin) - 1] - '0');
  mraa_gpio_context dev = mraa_gpio_init (pin_number);
  if (dev == NULL)
  {
    status = MRAA_ERROR_UNSPECIFIED;
    iot_log_error (impln->lc, "Failed to initialize a device at GPIO %s", pin);
  }
  else
  {
    if (strcmp (type, "OUT") == 0)
    {
      status = mraa_gpio_dir (dev, MRAA_GPIO_OUT);
    }
    else if (strcmp (type, "IN") == 0)
    {
      status = mraa_gpio_dir (dev, MRAA_GPIO_IN);
    }
    else
    {
      /* IN/OUT Configuration not supported - ignore */
    }
  }

  if (status != MRAA_SUCCESS)
  {
    iot_log_error (impln->lc, "Failed to set the GPIO %s to %s mode", pin, type);
  }
  else
  {
    if ((grove_set_devctxt (impln, (void *) dev, pin, type)) == NULL)
    {
      iot_log_error (impln->lc, "Unable to set the mraadev_ctxt at %s", pin, type);
      status = MRAA_ERROR_UNSPECIFIED;
    }
  }
  return status;
}

static mraa_result_t grove_aio_init (grove_pidriver_t *impln, char *pin, char *type)
{
  mraa_result_t status = MRAA_SUCCESS;

  int pin_number = GROVE_SUBPLATFORM_OFFSET + (pin[strlen (pin) - 1] - '0');
  mraa_aio_context dev = mraa_aio_init (pin_number);

  if (dev == NULL)
  {
    status = MRAA_ERROR_UNSPECIFIED;
    iot_log_error (impln->lc, "Failed to initialize a device at AIO %s", pin);
  }
  else
  {
    if ((grove_set_devctxt (impln, (void *) dev, pin, type)) == NULL)
    {
      iot_log_error (impln->lc, "Unable to set the mraadev_ctxt at %s", pin);
      status = MRAA_ERROR_UNSPECIFIED;
    }
  }
  return status;
}

static mraa_result_t grove_lcd_init (grove_pidriver_t *impln, char *pin, char *type)
{
  /* Initialize I2C bus for LCD */
  mraa_i2c_context lcd_dev = NULL;
  mraa_i2c_context rgb_dev = NULL;
  mraa_result_t status = MRAA_SUCCESS;

  if (!(lcd_dev = mraa_i2c_init (GROVE_I2C_BUS)))
  {
    status = MRAA_ERROR_UNSPECIFIED;
    iot_log_error (impln->lc, "mraa_i2c_init(LCD) connected to %s failed\n", pin);
    return status;
  }
  else
  {
    status = mraa_i2c_address (lcd_dev, GROVE_LCD_ADDR);
    if (status != MRAA_SUCCESS)
    {
      iot_log_error (impln->lc, "mraa_i2c_address(LCD) connected to %s failed, status = %d\n", pin, status);
      mraa_i2c_stop (lcd_dev);
      return status;
    }
  }

  /* Initialize I2C bus for Backlight */
  if (!(rgb_dev = mraa_i2c_init (GROVE_I2C_BUS)))
  {
    status = MRAA_ERROR_UNSPECIFIED;
    iot_log_error (impln->lc, "mraa_i2c_init(RGB) connected to %s failed\n", pin);
    return status;
  }
  else
  {
    status = mraa_i2c_address (rgb_dev, GROVE_RGB_ADDR);
    if (status != MRAA_SUCCESS)
    {
      iot_log_error (impln->lc, "mraa_i2c_address(RGB) connected to %s failed, status = %d\n", pin, status);
      mraa_i2c_stop (rgb_dev);
      return status;
    }
  }

  status = grove_lcd_hd44780_init (lcd_dev, rgb_dev);
  if (status == MRAA_SUCCESS)
  {
    /* store the lcd_dev context */
    grove_dev_ctxt_t *lcd_ctx = grove_set_devctxt (impln, (void *) lcd_dev, pin, type);
    if (lcd_ctx == NULL)
    {
      iot_log_error (impln->lc, "Unable to set the mraadev_ctxt at %s", pin);
      status = MRAA_ERROR_UNSPECIFIED;
    }
    else
    {
      grove_i2c_dev_ctxt_t *i2c_ctx = (grove_i2c_dev_ctxt_t *) lcd_ctx->dev_ctxt;
      i2c_ctx->rgb_dev = rgb_dev;
      i2c_ctx->is_lcd = true;
    }
  }
  return status;
}

static mraa_result_t grove_bme680_init (grove_pidriver_t *impln, char *pin, char *type)
{
  mraa_result_t status = MRAA_SUCCESS;

  bme680_context tph_dev = grove_bme680_initialize (GROVE_I2C_BUS, BME680_DEFAULT_ADDR, BME680_Temp_Offset);

  if (tph_dev != NULL)
  {
    /* store the context */
    if (grove_set_devctxt (impln, (void *) tph_dev, pin, type) == NULL)
    {
      iot_log_error (impln->lc, "Unable to set the mraadev_ctxt at %s", pin);
      status = MRAA_ERROR_UNSPECIFIED;
    }
  }
  else
  {
    status = MRAA_ERROR_UNSPECIFIED;
  }

  return status;
}

static mraa_result_t grove_i2c_init (grove_pidriver_t *impln, char *pin, char *type)
{
  mraa_result_t status = MRAA_SUCCESS;

  if (strcmp (type, "LCD") == 0)
  {
    status = grove_lcd_init (impln, pin, type);
  }
  else if (strcmp (type, "BME680") == 0)
  {
    status = grove_bme680_init (impln, pin, type);
  }
  else
  {
    status = MRAA_ERROR_UNSPECIFIED;
    iot_log_warn (impln->lc, "Invalid Type, Ignore I2C initialization");
  }
  return status;
}

static bool grove_init (void *impl, struct iot_logger_t *lc, const iot_data_t *config)
{
  mraa_result_t status = MRAA_SUCCESS;
  grove_pidriver_t *impln = (grove_pidriver_t *) impl;
  impln->lc = lc;
  pthread_mutex_init (&impln->mutex, NULL);

  iot_log_debug (lc, "driver initialization");
  edgex_deviceprofile *profiles = NULL;

  status = mraa_init ();
  if (status != MRAA_SUCCESS)
  {
    iot_log_error (lc, "GrovePI driver initialization failed");
    return false;
  }

  mraa_add_subplatform (MRAA_GROVEPI, "0");

  BME680_Temp_Offset = iot_data_f32 (iot_data_string_map_get (config, "BME680_Temp_Offset"));

  /* read the attributes from the device profile to initialize the driver */
  profiles = edgex_profiles (impln->svc);

  while (profiles)
  {
    edgex_deviceresource *dev_res = profiles->device_resources;
    grove_attributes_t *grove_attr = NULL;
    for (; dev_res != NULL; dev_res = dev_res->next)
    {
      devsdk_nvpairs *dev_attr = dev_res->attributes;
      assert (dev_attr != NULL);

      grove_attr = get_groveattributes (dev_attr);

      grove_dev_ctxt_t *dev = grove_device_lookup (impln, grove_attr->pin_no);
      if (dev != NULL)
      {
        /* device is initialized */
        continue;
      }
      else
      {
        if (strcmp (grove_attr->pin_type, "GPIO") == 0)
        {
          status = grove_gpio_init (impln, grove_attr->pin_no, grove_attr->type);
          assert (!status);
        }
        else if (strcmp (grove_attr->pin_type, "AIO") == 0)
        {
          status = grove_aio_init (impln, grove_attr->pin_no, grove_attr->type);
          assert (!status);
        }
        else if (strcmp (grove_attr->pin_type, "I2C") == 0)
        {
          status = grove_i2c_init (impln, grove_attr->pin_no, grove_attr->type);
          assert (!status);
        }
        else
        {
          /* PWM & Serial interface support not implemented */
          status = MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
        }
      }
      free (grove_attr);
    }
    profiles = profiles->next;
  }
 
  return (status == MRAA_SUCCESS);
}

static void grove_reconfigure (void *impl, const iot_data_t *config)
{
  grove_pidriver_t *driver = (grove_pidriver_t *) impl;
  iot_log_error (driver->lc, "Grove [Driver] configuration cannot be updated while running. Please restart the service.");
}

static bool grove_gethandler
  (
    void *impl,
    const char *devname,
    const devsdk_protocols *protocols,
    uint32_t nreadings,
    const devsdk_commandrequest *requests,
    devsdk_commandresult *readings,
    const devsdk_nvpairs *qparams,
    iot_data_t **exception
  )
{
  grove_pidriver_t *impln = (grove_pidriver_t *) impl;

  pthread_mutex_lock (&impln->mutex);
  char * buff = malloc (GROVE_ERR_BUFSZ);
  const devsdk_nvpairs *dev_attr = requests->attributes;
  assert (dev_attr != NULL);
  grove_attributes_t *grove_attr = get_groveattributes (dev_attr);
  bool ret_status = true;

  grove_dev_ctxt_t *mraa_devctxt = grove_device_lookup (impln, grove_attr->pin_no);
  if (mraa_devctxt != NULL)
  {
    volatile int read_value;
    if (strcmp (grove_attr->pin_type, "GPIO") == 0)
    {
      mraa_gpio_context gpio_dev = (mraa_gpio_context) mraa_devctxt->dev_ctxt;
      read_value = mraa_gpio_read (gpio_dev);

      assert (nreadings == 1);

      if (read_value == -1)
      {
        /* error */
        snprintf (buff, GROVE_ERR_BUFSZ, "error in GPIO read");
        ret_status = false;
      }
        /* Grove Button */
      else if (iot_typecode_type (requests->type) == IOT_DATA_UINT8)
      {
        readings->value = iot_data_alloc_ui8 (read_value);
      }
      else
      {
        /* No other type support available for GPIO in the profile */
        snprintf (buff, GROVE_ERR_BUFSZ, "error in GPIO read for request type: %d", iot_typecode_type(requests->type));
        ret_status = false;
      }
    } /* GPIO */
    else if (strcmp (grove_attr->pin_type, "AIO") == 0)
    {
      mraa_aio_context aio_dev = (mraa_aio_context) mraa_devctxt->dev_ctxt;
      read_value = mraa_aio_read (aio_dev);
      if (read_value == -1)
      {
        /* error */
        snprintf (buff, GROVE_ERR_BUFSZ, "error in AIO read");
        ret_status = false;
      }
      else
      {
        // get adc bit range
        int16_t range = (1 << mraa_aio_get_bit (aio_dev)) - 1;
        if (strcmp (requests->resname, "SoundIntensity") == 0)
        {
          assert (nreadings == 1);
          assert (iot_typecode_type (requests->type) == IOT_DATA_FLOAT32);

          float sound_intensity = read_value;
          if (grove_attr->normalize)
          {
            sound_intensity = read_value * GROVE_ADC_REF / range;
          }
          readings->value = iot_data_alloc_f32 (sound_intensity);
        }
        else if (strcmp (requests->resname, "LightIntensity") == 0)
        {
          assert (nreadings == 1);
          assert (iot_typecode_type (requests->type) == IOT_DATA_FLOAT32);
      
          /* Ref: https://github.com/intel-iot-devkit/upm/src/light/light.c */
          float light_intensity = (float) (10000.0 /
                                                powf ((((float) (range) - read_value) * 10.0 / read_value) * 15.0,
                                                      4.0 / 3.0));
          readings->value = iot_data_alloc_f32 (light_intensity);
        }
        else
        {
          assert (nreadings == 2); /* For RotarySensorMeasurements */
          grove_attributes_t *rotarysensor_attr = NULL;
          for (int index = 0; index < nreadings; index++)
          {
            /* Get attribute for each device object to apply scale if applicable */
            rotarysensor_attr = get_groveattributes (requests[index].attributes);

            if (strcmp (requests[index].resname, "RotaryAngle") == 0)
            {
              float rotaryangle = read_value;
              if (rotarysensor_attr->normalize)
              {
                rotaryangle = read_value * (float) GROVE_ROTARY_MAX_ANGLE / range;
              }
              readings[index].value = iot_data_alloc_f32 (rotaryangle);
            }
            else if (strcmp (requests[index].resname, "RotaryVoltage") == 0)
            {
              float rotaryvoltage = read_value;
              if (rotarysensor_attr->normalize)
              {
                rotaryvoltage = read_value * (float) GROVE_ADC_REF / range;
              }
              readings[index].value = iot_data_alloc_f32 (rotaryvoltage);
            }
            else
            {
              snprintf (buff, GROVE_ERR_BUFSZ, "Undefined %s device resource for RotarySensor", requests[index].resname);
              ret_status = false;
            }
            free (rotarysensor_attr);
          }
        }
      }
    } /* AIO */
    else if ((strcmp (grove_attr->pin_type, "I2C") == 0) && (strcmp (grove_attr->type, "BME680") == 0))
    {
      bme680_context tph_dev = (bme680_context) mraa_devctxt->dev_ctxt;
      struct bme680_field_data read_data;

      ret_status = grove_bme_read_data (tph_dev, &read_data);
      if (ret_status == false)
      {
        snprintf (buff, GROVE_ERR_BUFSZ, "Unable to read new data from BME680 sensor");
      }
      else
      {
        assert (nreadings == 3); // Temperature, Pressure and Humidity
        grove_attributes_t *bme_attr = NULL;
        for (int index = 0; index < nreadings; index++)
        {
          /* Get attribute for each device object to apply scale if applicable */
          bme_attr = get_groveattributes (requests[index].attributes);

          float bme_data = 0.0;

          if (strcmp (requests[index].resname, "Temperature") == 0)
          {
            bme_data = read_data.temperature;
          }
          else if (strcmp (requests[index].resname, "Pressure") == 0)
          {
            if (bme_attr->normalize)
            {
              bme_data = (float) (read_data.pressure / hPA_FACTOR);
            }
            else
            {
              bme_data = read_data.pressure;
            }
          }
          else if (strcmp (requests[index].resname, "Humidity") == 0)
          {
            bme_data = read_data.humidity;
          }
          else
          {
            snprintf (buff, GROVE_ERR_BUFSZ, "Undefined %s device resource for BME680", requests[index].resname);
            ret_status = false;
          }
          readings[index].value = iot_data_alloc_f32 (bme_data);
          free (bme_attr);
          bme_attr = NULL;
        }
      }
    } /* BME680 */
    else
    {
      /* Only GPIO, AIO and I2C interface types are supported */
      snprintf (buff, GROVE_ERR_BUFSZ, "Unsupported type, error in grove_gethandler()");
      ret_status = false;
    }
  } /* dev_ctxt != NULL */
  free (grove_attr);
  if (!ret_status)
  {
    iot_log_error (impln->lc, buff);
    * exception = iot_data_alloc_string (buff, IOT_DATA_TAKE);
  }
  else
  {
    free (buff);
  }

  pthread_mutex_unlock (&impln->mutex);
  
  return ret_status;
}

static bool grove_puthandler
  (
    void *impl,
    const char *devname,
    const devsdk_protocols *protocols,
    uint32_t nvalues,
    const devsdk_commandrequest *requests,
    const iot_data_t *values[],
    iot_data_t **exception
  )
{
  mraa_result_t status = MRAA_SUCCESS;
  grove_pidriver_t *impln = (grove_pidriver_t *) impl;

  pthread_mutex_lock (&impln->mutex);
  char * buff = malloc (GROVE_ERR_BUFSZ);

  /* Get the device context */
  const devsdk_nvpairs *dev_attr = requests[0].attributes;
  assert (dev_attr != NULL);
  grove_attributes_t *grove_attr = get_groveattributes (dev_attr);

  grove_dev_ctxt_t *mraa_devctxt = grove_device_lookup (impln, grove_attr->pin_no);
  if (mraa_devctxt != NULL)
  {
    if (strcmp (grove_attr->pin_type, "GPIO") == 0)
    {
      mraa_gpio_context gpio_dev = (mraa_gpio_context) mraa_devctxt->dev_ctxt;

      assert (iot_typecode_type(requests->type) == IOT_DATA_BOOL);
      assert (nvalues == 1);

      status = mraa_gpio_write (gpio_dev, iot_data_bool (values[--nvalues]));
      if (status != MRAA_SUCCESS)
      {
        snprintf (buff, GROVE_ERR_BUFSZ, "gpio write failure = %d", status);
      }
    }
    else if (strcmp (grove_attr->pin_type, "I2C") == 0)
    {
      assert (nvalues == 3); /* for lcd */

      grove_i2c_dev_ctxt_t *i2c_dev = (grove_i2c_dev_ctxt_t *) (mraa_devctxt->dev_ctxt);
      mraa_i2c_context mraa_i2cdev = (mraa_i2c_context) (i2c_dev->dev);

      uint8_t column = 0;
      uint8_t row = 0;
      char *display_string = NULL;

      for (int index = 0; index < nvalues; index++)
      {
        if (strcmp (requests[index].resname, "Display-String") == 0)
        {
          display_string = (char *)iot_data_string (values[index]);
        }
        else if (strcmp (requests[index].resname, "Row") == 0)
        {
          row = iot_data_ui8 (values[index]);
        }
        else if (strcmp (requests[index].resname, "Column") == 0)
        {
          column = iot_data_ui8 (values[index]);
        }
      }
      status = grove_lcd_set_cursor (mraa_i2cdev, row, column);
      if (status != MRAA_SUCCESS)
      {
        snprintf (buff, GROVE_ERR_BUFSZ, "lcd set_cursor error status = %d", status);
      }
      else
      {
        status = grove_lcd_write (mraa_i2cdev, display_string, strlen (display_string));
        if (status != MRAA_SUCCESS)
        {
          snprintf (buff, GROVE_ERR_BUFSZ, "lcd write error status = %d", status);
        }
      }
    }
    else
    {
      /* PWM & Serial interface support not implemented */
      status = MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
    }
  }
  free (grove_attr);
  if (!status)
  {
    iot_log_error (impln->lc, buff);
    * exception = iot_data_alloc_string (buff, IOT_DATA_TAKE);
  }
  else
  {
    free (buff);
  }

  pthread_mutex_unlock (&impln->mutex);
  return (status == MRAA_SUCCESS);
}

static void grove_stop (void *impl, bool force)
{
  grove_pidriver_t *impln = (grove_pidriver_t *) impl;
  mraa_result_t status = MRAA_SUCCESS;

  iot_log_debug (impln->lc, "Calling grove_stop()");

  /* Release the resources */
  for (int index = 0; index < GROVE_NO_PORTS; index++)
  {
    if (impln->dev[index] != NULL)
    {
      switch (impln->dev[index]->intf_type)
      {
        case GROVE_GPIO:
        {
          mraa_gpio_context gpio_dev = (mraa_gpio_context) impln->dev[index]->dev_ctxt;
          status |= mraa_gpio_close (gpio_dev);
          break;
        }
        case GROVE_AIO:
        {
          mraa_aio_context aio_dev = (mraa_aio_context) impln->dev[index]->dev_ctxt;
          status |= mraa_aio_close (aio_dev);
          break;
        }
        case GROVE_I2C:
        {
          if (!(strcmp (impln->dev[index]->pin_type, "LCD")))
          {
            grove_i2c_dev_ctxt_t *i2c_dev = (grove_i2c_dev_ctxt_t *) impln->dev[index]->dev_ctxt;
            if (i2c_dev->is_lcd)
            {
              grove_lcd_cleardisplay (i2c_dev->dev);
              grove_rgb_backlight_on (i2c_dev->rgb_dev, false);

              status |= mraa_i2c_stop (i2c_dev->rgb_dev);
              i2c_dev->rgb_dev = NULL;
              i2c_dev->is_lcd = false;
            }
            status |= mraa_i2c_stop (i2c_dev->dev);
          }
          else if (!(strcmp (impln->dev[index]->pin_type, "BME680")))
          {
            bme680_context tph_dev = (bme680_context) impln->dev[index]->dev_ctxt;
            bme680_close (tph_dev);
          }
          else
          {
            iot_log_error (impln->lc, "grove_stop(), invalid pin_type: %s\n", impln->dev[index]->pin_type);
          }
          break;
        }
        default:
        {
          status |= MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
          iot_log_error (impln->lc, "grove_stop(), interface type %d not implemented", impln->dev[index]->intf_type);
          break;
        }
      }
      free (impln->dev[index]->pin_number);
      free (impln->dev[index]->pin_type);
      free (impln->dev[index]);
      impln->dev[index] = NULL;

    } /* dev != NULL */
  }

  if (status != MRAA_SUCCESS)
  {
    iot_log_error (impln->lc, "grove_stop() failure");
  }
  mraa_deinit ();
  pthread_mutex_destroy (&impln->mutex);
}

int main (int argc, char *argv[])
{
  iot_data_t *defaults = NULL;
  grove_pidriver_t *implObject = malloc (sizeof (grove_pidriver_t));
  memset (implObject, 0, sizeof (grove_pidriver_t));
  sem_init (&grove_sem, 0, 0);

  devsdk_error err;
  err.code = 0;

  /* Device Callbacks */
  devsdk_callbacks myImpls = 
  {
      grove_init,
      grove_reconfigure,
      NULL,
      grove_gethandler,
      grove_puthandler,
      grove_stop
   };

   /* Initalise a device grove service */
  devsdk_service_t *grove_service = devsdk_service_new
    (GROVE_SVC, GROVE_VERSION, implObject, myImpls, &argc, argv, &err);
  GROVE_ERR_CHECK (err);

  int n = 1;
  while (n < argc)
  {
    if (strcmp (argv[n], "-h") == 0 || strcmp (argv[n], "--help") == 0)
    {
      printf ("Options:\n");
      printf ("  -h, --help\t\t: Show this text\n");
      devsdk_usage ();
      return 0;
    }
    else
    {
      printf ("%s: Unrecognized option %s\n", argv[0], argv[n]);
      return 0;
    }
  }

  implObject->svc = grove_service;

  /* Setup default configuration */

  defaults = iot_data_alloc_map (IOT_DATA_STRING);
  iot_data_string_map_add (defaults, "BME680_Temp_Offset", iot_data_alloc_f32 (0.0));

  /* Start the device service*/
  devsdk_service_start (grove_service, defaults, &err);
  GROVE_ERR_CHECK (err);

  printf ("\nRunning - press ctrl-c to exit\n");
  signal (SIGINT, grove_inthandler);
  signal (SIGTERM, grove_inthandler);

  // wait until the service is interrupted 
  sem_wait (&grove_sem);

  devsdk_service_stop (grove_service, true, &err);
  GROVE_ERR_CHECK (err);

  sem_destroy (&grove_sem);
  devsdk_service_free (grove_service);
  iot_data_free (defaults);
  free (implObject);

  return 0;
}
