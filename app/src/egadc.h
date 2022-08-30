#pragma once
#include <zephyr/drivers/spi.h>

void egadc_init(const struct spi_dt_spec *bus);
void egadc_print(const struct spi_dt_spec *bus);