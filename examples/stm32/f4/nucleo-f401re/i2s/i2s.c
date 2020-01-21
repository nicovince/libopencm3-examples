/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (c) 2015 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "clock.h"
#include "console.h"

static void gpio_clock_setup(void)
{
	/* Enable GPIOA clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);
}

static void gpio_setup(void)
{

	/* Set GPIO5 (in GPIO port A) to 'output push-pull'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

}

static void i2s_setup(void)
{
    uint32_t cr_temp;
    const uint32_t i2sdiv = 187;
    uint32_t plli2sn = 192;
    uint32_t  plli2sr = 2;

	/* WS: Word select on NSS pin
	 * SD: Serial Data on MOSI pin
	 * CK: Serial Clock on SCK pin
	 * ext_SD: control i2s full duplex on MISO
	 * MCK: Master Clock when i2s configured in master to output clock
	 * I2S3
	 * I2S3_SD: PC12 (AF06)
	 * I2S3ext_SD: PC11 (AF05)
	 * I2S3_CK: PC10 (AF06)
	 * I2S3_WS: PA4 (AF06)
	 *
	 * I2S2
	 * I2S2_SD: PB15 (AF05)
	 * I2S2ext_SD: PB14 (AF06)
	 * I2S2_CK: PB13 (AF05)
	 * I2S2_WS: PB12 (AF05)
	 */
	/* Enable GPIOS ports whose pins we are using */
	rcc_periph_clock_enable(RCC_GPIOA | RCC_GPIOC | RCC_GPIOB);

	/* I2S3 */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN,
	                GPIO10 | GPIO11 | GPIO12);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO4);
	gpio_set_af(GPIOC, GPIO_AF6, GPIO12 | GPIO10);
	gpio_set_af(GPIOC, GPIO_AF5, GPIO11);
	gpio_set_af(GPIOA, GPIO_AF6, GPIO4);
	//TODO: gpio_set_output_options(port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, gpios);
	rcc_periph_clock_enable(RCC_SPI3);

	/* configure PLLI2S */
	cr_temp = ((plli2sn & RCC_PLLI2SCFGR_PLLI2SN_MASK) << RCC_PLLI2SCFGR_PLLI2SN_SHIFT
	           | (plli2sr & RCC_PLLI2SCFGR_PLLI2SR_MASK) << RCC_PLLI2SCFGR_PLLI2SR_SHIFT);
	RCC_PLLI2SCFGR = cr_temp;
	RCC_CR |= RCC_CR_PLLI2SON;
	while (!(RCC_CR & RCC_CR_PLLI2SRDY));

	RCC_CFGR &= ~RCC_CFGR_I2SSRC;


	/* I2S2 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN,
	                GPIO12 | GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF6, GPIO14);
	/* I2S2 as master */
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO12 | GPIO13);
	rcc_periph_clock_enable(RCC_SPI2);

	cr_temp = (i2sdiv & 0xFF) | SPI_I2SPR_ODD;
	SPI_I2SPR(SPI2) = cr_temp;
	cr_temp = SPI_I2SCFGR_I2SMOD |
		SPI_I2SCFGR_I2SSTD_MSB_JUSTIFIED |
		SPI_I2SCFGR_DATLEN_16BIT |
		SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT;
	SPI_I2SCFGR(SPI2) = cr_temp;
	cr_temp = SPI_CR2_RXNEIE |
		SPI_CR2_RXNEIE |
		SPI_CR2_ERRIE;
	SPI_CR2(SPI2) = cr_temp;
	SPI_I2SCFGR(SPI2) = SPI_I2SCFGR_I2SE; /* enable I2S after configuration */
}

int main(void)
{
	gpio_clock_setup();
	gpio_setup();
	clock_setup();
	console_setup(115200);

	i2s_setup();

	/* Blink the LED on the board and print message. */
	while (1) {
		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOA, GPIO5);	/* LED on/off */
		console_puts("Console test program\n");
		msleep(500);
	}

	return 0;
}

