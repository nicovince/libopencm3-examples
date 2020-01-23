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
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#include "clock.h"
#include "console.h"
#include "fifo16.h"

#define SPI_SR_FRE (1 << 8)
#define I2S_BUF_SIZE 256

fifo16_t i2s2_rx, i2s2_tx;
uint16_t i2s2_data_rx[I2S_BUF_SIZE];
uint16_t i2s2_data_tx[I2S_BUF_SIZE];

static void gpio_clock_setup(void)
{
	/* Enable GPIO clock for LED & Debug. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
}

static void gpio_setup(void)
{

	/* Set GPIO5 (in GPIO port A) to 'output push-pull'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO3);

}

static void dbg_set(void)
{
	gpio_set(GPIOC, GPIO3);
}
static void dbg_clear(void)
{
	gpio_clear(GPIOC, GPIO3);
}

static void i2s_set_prescaler(uint32_t spi, uint8_t i2sdiv, uint8_t odd)
{
	uint32_t tmp;
	tmp = i2sdiv;
	if (odd) {
		tmp |= SPI_I2SPR_ODD;
	}
	SPI_I2SPR(spi) = tmp;
}

static void i2s_enable(uint32_t spi)
{
	SPI_I2SCFGR(spi) |= SPI_I2SCFGR_I2SE;
}

static void i2s_set_config(uint32_t spi, uint8_t i2smod, uint8_t i2scfg,
                           uint8_t pcmsync, uint8_t i2sstd, uint8_t ckpol,
                           uint8_t datlen, uint8_t chlen)
{
	uint32_t tmp = 0;
	if (chlen) {
		tmp |= SPI_I2SCFGR_CHLEN;
	}

	tmp |= (datlen & 0x3) << SPI_I2SCFGR_DATLEN_LSB;

	if (ckpol) {
		tmp |= SPI_I2SCFGR_CKPOL;
	}

	tmp |= (i2sstd & 0x3) << SPI_I2SCFGR_I2SSTD_LSB;

	if (pcmsync) {
		tmp |= SPI_I2SCFGR_PCMSYNC;
	}

	tmp |= (i2scfg & 0x3) << SPI_I2SCFGR_I2SCFG_LSB;

	if (i2smod) {
		tmp |= SPI_I2SCFGR_I2SMOD;
	}
	SPI_I2SCFGR(spi) = tmp;
}

static void i2s_setup(void)
{
	uint32_t cr_temp;
	const uint8_t i2sdiv = 187;
	const uint8_t odd = 1;
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
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);

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
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
	                        GPIO12 | GPIO13 | GPIO15);

	nvic_enable_irq(NVIC_SPI2_IRQ);
	rcc_periph_clock_enable(RCC_SPI2);

	i2s_set_prescaler(I2S2_EXT_BASE, i2sdiv, odd);
	i2s_set_config(I2S2_EXT_BASE, 1 /* i2smod */,
		       SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE, 0 /* pcmsync */,
		       SPI_I2SCFGR_I2SSTD_I2S_PHILIPS, 0 /* ckpol */,
		       SPI_I2SCFGR_DATLEN_16BIT, 0 /* chlen */);

	i2s_set_prescaler(SPI2, i2sdiv, odd);
	i2s_set_config(SPI2, 1 /* i2smod */,
		       SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT, 0 /* pcmsync */,
		       SPI_I2SCFGR_I2SSTD_I2S_PHILIPS, 0 /* ckpol */,
		       SPI_I2SCFGR_DATLEN_16BIT, 0 /* chlen */);
	spi_enable_tx_buffer_empty_interrupt(SPI2);
	spi_enable_error_interrupt(SPI2);
	spi_enable_rx_buffer_not_empty_interrupt(SPI2);

	i2s_enable(I2S2_EXT_BASE);
	i2s_enable(SPI2);
}

void spi2_isr(void)
{
	uint32_t spi_base = SPI2;
	uint32_t sr = SPI_SR(spi_base);
	uint16_t tx_data = 0;
	uint32_t rx_data;
	if (sr & SPI_SR_TXE) {
		if (!is_empty16(&i2s2_tx))
		{
			tx_data = pop16(&i2s2_tx);
		}
		SPI_DR(spi_base) = tx_data;
	}

	if (sr & SPI_SR_RXNE) {
		rx_data = SPI_DR(spi_base);
		dbg_set();
	}

	if (sr & SPI_SR_OVR) {
		dbg_set();
	}

	if (sr & SPI_SR_UDR) {
		dbg_set();
	}

	if (sr & SPI_SR_FRE) {
		dbg_set();
	}

	spi_base = I2S2_EXT_BASE;
	sr = SPI_SR(spi_base);
	if (sr & SPI_SR_RXNE) {
		rx_data = SPI_DR(spi_base);
		if (size_available16(&i2s2_rx) >= 2) {
			push16(&i2s2_rx, rx_data);
		}
	}
	rx_data = rx_data;
	dbg_clear();
}

static size_t i2s_write(fifo16_t *fifo_ptr, uint16_t *data, size_t size)
{
	size_t wr_size = 0;
	while (size > 0) {
		if (is_full16(fifo_ptr))
		{
			break;
		}
		push16(fifo_ptr, data[wr_size++]);
		size--;
	}

	return wr_size;
}

__attribute__((unused)) static size_t i2s_read(fifo16_t * fifo_ptr, uint16_t * data, size_t size)
{
	size_t rd_size = 0;
	while (size > 0) {
		if (is_empty16(fifo_ptr)) {
			break;
		}
		data[rd_size++] = pop16(fifo_ptr);
		size--;
	}

	return rd_size;
}

int main(void)
{
	uint16_t tx_data[2*I2S_BUF_SIZE];
	uint16_t rx_data[2*I2S_BUF_SIZE];
	size_t tx_size;
	size_t rx_size;
	size_t rx_received_cnt = 0;
	size_t cur_idx = 0;
	char dbg_buf[128];
	uint32_t cur_time;
	uint32_t last_time;


	for (int i = 0; i < 2*I2S_BUF_SIZE; ++i) {
		tx_data[i] = 0xF000 + i;
	}

	gpio_clock_setup();
	gpio_setup();
	clock_setup();
	console_setup(115200);

	console_puts("Setup i2s\n");
	i2s_setup();
	console_puts("start main loop\n");

	init_fifo16(&i2s2_rx, i2s2_data_rx, I2S_BUF_SIZE);
	init_fifo16(&i2s2_tx, i2s2_data_tx, I2S_BUF_SIZE);

	last_time = mtime();
	while (1) {
		gpio_toggle(GPIOA, GPIO5);	/* LED on/off */

		tx_size = i2s_write(&i2s2_tx, &tx_data[cur_idx],
		                    sizeof(tx_data)/sizeof(tx_data[0]) - cur_idx);
		cur_idx += tx_size;
		if (cur_idx == (sizeof(tx_data)/sizeof(tx_data[0])))
		{
			cur_idx = 0;
		}

		rx_size = i2s_read(&i2s2_rx, rx_data, I2S_BUF_SIZE);
		rx_received_cnt += rx_size;
		cur_time = mtime();
		if ((cur_time - last_time) >= 5*1000) {
			snprintf(dbg_buf, sizeof(dbg_buf),
			         "received %d samples in 5 seconds\n", rx_received_cnt);
			console_puts(dbg_buf);
			last_time = cur_time;
			rx_received_cnt = 0;
		}
		//snprintf(dbg_buf, sizeof(dbg_buf), "written %d bytes into tx fifo\n", tx_size);
		//console_puts(dbg_buf);
	}

	return 0;
}

