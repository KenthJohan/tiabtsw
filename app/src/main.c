/*
west build -b nucleo_wb55rg

Booting Zephyr OS build zephyr-v3.1.0-3437-gd9ed09357ee0
Booting Zephyr OS build zephyr-v3.1.0-4550-g2cbd287e0c21 (tried on laptop, bluetooth problems)
Booting Zephyr OS build v3.2.0-rc1-57-gb27c5d73ef09







*** Booting Zephyr OS build v3.2.0-rc1-57-gb27c5d73ef09  ***                              
Bluetooth initialized                                                                       
Advertising successfully started                                                           
[00:00:00.326,000] [0m<inf> main: Zephyr Example Application 1.0.0[0m                     
Connected                                                                               
[00:05:08.462,000] [1;33m<wrn> hci_ipm: Invalid peer addr FF:FF:FF:FF:FF:FF (random)[0m    
[00:05:08.464,000] [1;33m<wrn> bt_hci_core: opcode 0x2016 status 0x3a[0m                  
[00:05:08.464,000] [1;31m<err> bt_hci_core: Failed read remote features (-5)[0m          
[00:05:08.464,000] [1;33m<wrn> bt_hci_core: opcode 0x2032 status 0x0c[0m                     
[00:05:08.464,000] [1;31m<err> bt_hci_core: Failed LE Set PHY (-5)[0m                           
[00:05:15.520,000] [0m<inf> bas: BAS Notifications enabled[0m



*** Booting Zephyr OS build v3.2.0-rc1-57-gb27c5d73ef09  ***
*** Booting Zephyr OS build v3.2.0-rc3-24-g8364715998c9  ***





*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>

#include "app_version.h"
#include "egadc.h"
#include "bt.h"

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
	LOG_INF("Zephyr Example Application %s", "" DT_NODE_PATH(DT_NODELABEL(examplesensor0)));

	if (!spi_is_ready(&c.bus))
	{
		LOG_ERR("SPI bus is not ready %i", 0);
		return;
	}
	
	
	egadc_init(&c);
	//mybt_init();
	static int32_t i = 0;
	while (1)
	{
		printk("                      " MCP356X_PRINTF_HEADER "\n");
		printk("%8i %8i avg " MCP356X_PRINTF_PLUS "\n", c.num_irq, c.num_drdy, MCP356X_ARGS(c.avg));
		printk("%8i %8i mv  " MCP356X_PRINTF_PLUS "\n", c.num_irq, c.num_drdy, MCP356X_ARGS(c.mv));
		
		//mybt_progress();
		//bt_bas_set_battery_level(i++);

		//k_sem_give(&c.acq_sem);
		k_sleep(K_MSEC(5000));
	}
}

