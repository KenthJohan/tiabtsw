/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include "app_version.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
struct spi_dt_spec bus = SPI_DT_SPEC_GET(DT_NODELABEL(mcp3204), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);
int a = CONFIG_ADC_INIT_PRIORITY;
int b = CONFIG_SPI_INIT_PRIORITY;

void main(void)
{
	int ret;
	const struct device *sensor;
	const struct device *adc1;

	printk("Zephyr Example Application %s\n", APP_VERSION_STR);

	sensor = DEVICE_DT_GET(DT_NODELABEL(examplesensor0));
	if (!device_is_ready(sensor)) {
		LOG_ERR("examplesensor0 not ready");
		return;
	}

	if (!spi_is_ready(&bus)) {
		LOG_ERR("SPI bus is not ready");
		return;
	}


	adc1 = DEVICE_DT_GET(DT_NODELABEL(mcp3204));
	if (!device_is_ready(adc1)) {
		LOG_ERR("adc not ready");
		return;
	}

	while (1) {
		struct sensor_value val;

		ret = sensor_sample_fetch(sensor);
		if (ret < 0) {
			LOG_ERR("Could not fetch sample (%d)", ret);
			return;
		}

		ret = sensor_channel_get(sensor, SENSOR_CHAN_PROX, &val);
		if (ret < 0) {
			LOG_ERR("Could not get sample (%d)", ret);
			return;
		}

		printk("Sensor value: %d\n", val.val1);

		k_sleep(K_MSEC(1000));
	}
}

