/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include "app_version.h"
#include "egadc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
struct spi_dt_spec bus = SPI_DT_SPEC_GET(DT_NODELABEL(mcp3204), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);


void main(void)
{
	printk("Zephyr Example Application %s\n", APP_VERSION_STR);

	if (!spi_is_ready(&bus))
	{
		LOG_ERR("SPI bus is not ready");
		return;
	}

	egadc_init(&bus);

	while (1)
	{
		egadc_print(&bus);
		k_sleep(K_MSEC(1000));
	}
}

