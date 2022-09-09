/*
west build -b nucleo_wb55rg
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#include "app_version.h"
#include "egadc.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);
//struct spi_dt_spec bus = SPI_DT_SPEC_GET(DT_NODELABEL(mcp3204), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);

struct mcp356x_config c = 
{
	.bus = SPI_DT_SPEC_GET(DT_NODELABEL(examplesensor0), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1),
	.irq = GPIO_DT_SPEC_GET(DT_NODELABEL(examplesensor0), irq_gpios)
};

void main(void)
{
	LOG_INF("Zephyr Example Application %s", APP_VERSION_STR);

	if (!spi_is_ready(&c.bus))
	{
		LOG_ERR("SPI bus is not ready");
		return;
	}
	
	egadc_init(&c);

	while (1)
	{
		printk("avg " MCP356X_PRINTF_PLUS "\n", MCP356X_ARGS(c.avg));
		printk("mv  " MCP356X_PRINTF_PLUS "\n", MCP356X_ARGS(c.mv));
		
		//k_sem_give(&c.acq_sem);
		k_sleep(K_MSEC(1000));
	}
}

