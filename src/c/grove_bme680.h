/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <stdbool.h>
#include "bme680_mraa.h"

/* Pascals to hPa conversion */
#define hPA_FACTOR 100

bme680_context grove_bme680_initialize (int bus, int addr, float temp_offset);

bool grove_bme_read_data (bme680_context tph_dev, struct bme680_field_data *field_data);
