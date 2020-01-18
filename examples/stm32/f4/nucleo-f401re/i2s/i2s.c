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

static void i2s_clock_setup(void)
{
	/* WS: Word select on NSS pin
	 * SD: Serial Data on MOSI pin
	 * CK: Serial Clock on SCK pin
	 * ext_SD: control i2s full duplex on MISO
	 * MCK: Master Clock when i2s configured in master to output clock
	 */
	//I2S3
	//PC7
	//PC10
	//PC11
	//PC12
	//
	//I2S2
	//PC2
	//PC3
	//PC6
}

int main(void)
{
	gpio_clock_setup();
	gpio_setup();
	clock_setup();
	console_setup(115200);

	/* Blink the LED on the board and print message. */
	while (1) {
		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOA, GPIO5);	/* LED on/off */
		console_puts("Console test program\n");
		msleep(500);
	}

	return 0;
}

