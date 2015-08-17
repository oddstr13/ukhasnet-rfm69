/**
 * spi_conf.h
 *
 * This file is part of the UKHASNet (ukhas.net) maintained RFM69 library for
 * use with all UKHASnet nodes, including Arduino, AVR and ARM.
 *
 * Ported to Arduino 2014 James Coxon
 * Ported, tidied and hardware abstracted by Jon Sowman, 2015
 *
 * Copyright (C) 2014 Phil Crump
 * Copyright (C) 2015 Jon Sowman <jon@jonsowman.com>
 *
 * Based on RF22 Copyright (C) 2011 Mike McCauley
 * Ported to mbed by Karl Zweimueller
 *
 * Based on RFM69 LowPowerLabs (https://github.com/LowPowerLab/RFM69/)
 */

#ifndef __SPI_CONF_H__
#define __SPI_CONF_H__

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * These functions are all required and are called by the library. For
 * documentation on what you need to put in them, see spi_conf.c in the same
 * directory as this file.
 */
bool rfm69_init(void);
uint8_t spi_exchange_single(uint8_t out);
void spi_ss_assert(void);
void spi_ss_deassert(void);

#endif /* __SPI_CONF_H__ */