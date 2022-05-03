/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <drivers/sensor.h>

#include "app_version.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);


//struct spi_dt_spec spi_spec1 = SPI_DT_SPEC_GET(DT_NODELABEL(foo), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);
int i = DT_SPI_DEV_CS_GPIOS_PIN(DT_NODELABEL(foo));

const struct device *sensor1 = DEVICE_DT_GET(DT_NODELABEL(foo));

void main(void)
{
	int ret;
	const struct device *sensor;

	printk("Zephyr Example Application %s\n", APP_VERSION_STR);

	sensor = DEVICE_DT_GET(DT_NODELABEL(examplesensor0));
	if (!device_is_ready(sensor)) {
		LOG_ERR("Sensor not ready");
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

