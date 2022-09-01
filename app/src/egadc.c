#include "egadc.h"

#include "MCP356X.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(adc_mcp356x, LOG_LEVEL_DBG);

// The ADC9 uses 2000mV voltage ref chip MCP1501
#define VREF  2048

static void set8(const struct spi_dt_spec *bus, uint8_t reg, uint8_t value)
{
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	tx[1] = value;
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	spi_transceive_dt(bus, &tx_buf, &rx_buf);
}


static uint8_t get8(const struct spi_dt_spec *bus, uint8_t reg)
{
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive_dt(bus, &tx_buf, &rx_buf);
	return rx[1];
}

static uint32_t get24(const struct spi_dt_spec *bus, uint8_t reg)
{
	uint8_t tx[4] = {0};
	uint8_t rx[4] = {0};
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive_dt(bus, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 16) | (rx[2] << 8) | (rx[3] << 0);
	return v;
}

static uint32_t get32(const struct spi_dt_spec *bus, uint8_t reg)
{
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[4] = 0; // Need to write in order to read. Exchange 0 for reading data.
	spi_transceive_dt(bus, &tx_buf, &rx_buf);
	uint32_t v = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | (rx[4] << 0);
	return v;
}

static int mcp356x_data11_get(const struct spi_dt_spec *bus, struct mcp356x_data11 * value)
{
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, MCP356X_REG_ADC_DATA, MCP356X_CMD_INC_READ);
	tx[1] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[2] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[3] = 0; // Need to write in order to read. Exchange 0 for reading data.
	tx[4] = 0; // Need to write in order to read. Exchange 0 for reading data.
	
	//printk("rx: %02x %02x %02x %02x %02x\n", rx[0], rx[1], rx[2], rx[3], rx[4]);

	int err;
	err = spi_transceive_dt(bus, &tx_buf, &rx_buf);
	if (err) {return err;}
	value->channel = rx[1] >> 4;
	uint8_t sign = rx[1] & 0x01;
	value->value = (rx[2] << 16) | (rx[3] << 8) | (rx[4] << 0);
	if (sign != 0)
	{
		value->value -= 16777215;
	}
}


static void set24(const struct spi_dt_spec *bus, uint8_t reg, uint32_t value)
{
	uint8_t tx[4] = {0};
	uint8_t rx[4] = {0};
	tx[0] = MCP356X_COMMAND_BYTE(MCP356X_DEVICE_ADR, reg, MCP356X_CMD_INC_WRITE);
	tx[1] = ( value >> 16 ) & 0xFF;
	tx[2] = ( value >> 8 ) & 0xFF;
	tx[3] = ( value >> 0 ) & 0xFF;
	struct spi_buf buf_tx[] = {{.buf = &tx,.len = sizeof(tx)}};
	struct spi_buf buf_rx[] = {{.buf = &rx,.len = sizeof(rx)}};
	struct spi_buf_set tx_buf = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx_buf = {.buffers = buf_rx, .count = 1};
	spi_transceive_dt(bus, &tx_buf, &rx_buf);
}



static void set8_verbose(const struct spi_dt_spec *bus, uint8_t reg, uint8_t value)
{
	set8(bus, reg, value);
	uint8_t v = get8(bus, reg);
	LOG_INF("SET8: %s: %02x %02x", MCP356X_REG_tostring(reg), value, v);
}

static void set24_verbose(const struct spi_dt_spec *bus, uint8_t reg, uint32_t value)
{
	set24(bus, reg, value);
	uint32_t v = get24(bus, reg);
	LOG_INF("SET24: %s: %08x %08x", MCP356X_REG_tostring(reg), value, v);
}








static void drdy_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct mcp356x_config *config = CONTAINER_OF(cb, struct mcp356x_config, drdy_cb);
	k_sem_give(&config->drdy_sem);
	//struct mcp356x_data11 data;
	//mcp356x_data11_get(&config->bus, &data);
}




static void mcp356x_acquisition_thread(struct mcp356x_config * config)
{
	LOG_INF("mcp356x_acquisition_thread started!");
	while (true)
	{
		//k_sem_take(&config->acq_sem, K_FOREVER);
		k_sem_take(&config->drdy_sem, K_SECONDS(12));


		int err = 0;

		struct mcp356x_data11 data;
		err = mcp356x_data11_get(&config->bus, &data);
		if (data.channel < 8)
		{
			config->h[data.channel]++;
		}


		//printk("mcp356x_data11 %i!\n", (int)data.channel);
		//printk("§§err %i!\n", err);
		if (err)
		{
			//printk("mcp356x_acquisition_thread error: %i\n", err);
			//return;
		}
	}
}



/*
5.3.1
ANALOG GAIN
The gain settings from 0.33x to 16x are done in the
analog domain. This analog gain is placed on each ADC
differential input. Each doubling of the gain improves the
thermal noise due to sampling by approximately 3 dB,
which means the lowest noise configuration is obtained
when using the highest analog gain. The SNR, however,
is degraded, since doubling the gain factor reduces the
maximum allowable input signal amplitude by
approximately 6 dB.
If the gain is set to 0.33x, the differential input range
theoretically becomes ±3 * VREF. However, the device
does not support input voltages outside of the power
supply voltage range. If large reference voltages are
used with this gain, the input voltage range will be
clipped between AGND and AVDD, and therefore, the
output code span will be limited. This gain is useful
when the reference voltage is small and when the
input signal voltage is large.
The analog gain stage can be used to amplify very low
signals, but the differential input range of the
Delta-Sigma modulator must not be exceeded.
*/
#define MY_GAIN MCP356X_CFG_2_GAIN_X_033

void egadc_init(struct mcp356x_config * config)
{
	LOG_INF("Init ADC MCP356X");



	set8_verbose(&config->bus, MCP356X_REG_CFG_0,
	MCP356X_CFG_0_VREF_SEL_0 |
	MCP356X_CFG_0_CLK_SEL_2 |
	MCP356X_CFG_0_CS_SEL_0 |
	MCP356X_CFG_0_MODE_CONV
	);
	set8_verbose(&config->bus, MCP356X_REG_CFG_1,
	MCP356X_CFG_1_PRE_1 |
	MCP356X_CFG_1_OSR_32 |
	MCP356X_CFG_1_DITHER_DEF
	);
	set8_verbose(&config->bus, MCP356X_REG_CFG_2,
	MCP356X_CFG_2_BOOST_X_1 |
	//MCP356X_CFG_2_GAIN_X_1 |
	MY_GAIN |
	MCP356X_CFG_2_AZ_MUX_DIS |
	MCP356X_CFG_2_AZ_VREF_EN |
	MCP356X_CFG_2_AZ_FREQ_HIGH
	);
	set8_verbose(&config->bus, MCP356X_REG_CFG_3,
	MCP356X_CFG_3_CONV_MODE_CONT |
	//MCP356X_CFG_3_DATA_FORMAT_DEF |
	MCP356X_CFG_3_DATA_FORMAT_CH_ADC |
	MCP356X_CFG_3_CRC_FORMAT_16 |
	MCP356X_CFG_3_CRC_COM_DIS |
	MCP356X_CFG_3_CRC_OFF_CAL_EN |
	MCP356X_CFG_3_CRC_GAIN_CAL_EN
	);
	set8_verbose(&config->bus, MCP356X_REG_MUX,
	MCP356X_MUX_VIN_POS_CH0 | 
	MCP356X_MUX_VIN_NEG_CH1 |
	//MCP356X_MUX_VIN_POS_CH0 | 
	//MCP356X_MUX_VIN_NEG_AGND |
	0
	);
	//set24_verbose(MCP356X_REG_SCAN, 0);
	set24_verbose(&config->bus, MCP356X_REG_SCAN, MCP356X_SCAN_CH0|MCP356X_SCAN_CH1|MCP356X_SCAN_CH2|MCP356X_SCAN_CH3);
	//set24_verbose(bus, MCP356X_REG_SCAN, MCP356X_SCAN_CH0);
	//set24_verbose(bus, MCP356X_REG_SCAN, MCP356X_SCAN_CH3);
	
	set24_verbose(&config->bus, MCP356X_REG_IRQ, MCP356X_IRQ_MODE_LOGIC_HIGH);
	set24_verbose(&config->bus, MCP356X_REG_OFFSET_CAL, 0);
	set24_verbose(&config->bus, MCP356X_REG_GAIN_CAL, 0x00800000);
	//set24_verbose(MCP356X_RSV_REG_W_A, 0x00900F00);
	set24_verbose(&config->bus, MCP356X_RSV_REG_W_A, 0x00900000);


	int err;
	err = gpio_pin_configure_dt(&config->irq, GPIO_INPUT);
	if (err) {return err;}
	err = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {return err;}
	gpio_init_callback(&config->drdy_cb, drdy_callback, BIT(config->irq.pin));
	LOG_INF("gpio_init_callback %i", config->irq.pin);
	err = gpio_add_callback(config->irq.port, &config->drdy_cb);
	if (err) {return err;}

	k_sem_init(&config->acq_sem, 0, 1);
	k_sem_init(&config->drdy_sem, 0, 1);

	k_thread_create(&config->thread, config->stack,
			ADC_MCP356X_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)mcp356x_acquisition_thread,
			(void *)config, NULL, NULL,
			K_PRIO_COOP(ADC_MCP356X_ACQUISITION_THREAD_PRIO),
			//0,
			0, K_NO_WAIT);
	/* Add instance number to thread name? */
	k_thread_name_set(&config->thread, "mcp356x");


}





void egadc_print(struct mcp356x_config * config)
{
	/*
	printk("Channel:  ch0  ch1  ch2  ch3  ch4  ch5  ch6  ch7\n"), 
	printk("Voltage: %4i %4i %4i %4i %4i %4i %4i %4i\n", 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH0), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH1), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH2), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH3), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH4), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH5), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH6), 
	(int)voltage_ch(MCP356X_MUX_VIN_POS_CH7)
	);
	*/
	// printk("Voltage0: %4i\n", (int)voltage_ch(MCP356X_MUX_VIN_POS_CH0));
	// printk("Voltage7: %4i\n", (int)voltage_ch(MCP356X_MUX_VIN_POS_CH7));
	//printk("ADCDATA: %08x\n", get32(MCP356X_REG_ADC_DATA));
	//struct mcp356x_data11 data;
	//mcp356x_data11_get(&config->bus, &data);
	//printk("Voltage: %02x %08x %08i\n", data.channel, data.value, MCP356X_raw_to_mv(data.value, VREF, MY_GAIN));
	//printk("%x\n", data.channel);
}




