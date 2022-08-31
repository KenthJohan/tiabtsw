#pragma once
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>


#define ADC_MCP356X_ACQUISITION_THREAD_STACK_SIZE 512
#define ADC_MCP356X_ACQUISITION_THREAD_PRIO 0

struct mcp356x_config
{
	const struct spi_dt_spec bus;
	struct gpio_dt_spec irq;
	struct k_sem acq_sem;		/* Signal acq thread for next sample */
	struct k_sem drdy_sem;		/* Signal data ready IRQ */
	struct gpio_callback drdy_cb;	/* For data ready IRQ */
	struct k_thread thread;		/* Acquisition thread */
	int dummy;
	K_KERNEL_STACK_MEMBER(stack, ADC_MCP356X_ACQUISITION_THREAD_STACK_SIZE);
};


void egadc_init(struct mcp356x_config * config);
void egadc_print(struct mcp356x_config * config);