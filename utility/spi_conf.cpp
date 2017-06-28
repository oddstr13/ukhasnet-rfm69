/**
 * spi_conf.c
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

#include <Arduino.h>
#include "UKHASnetRFM69.h"
#include "spi_conf.h"

#include <SPI.h>

/**
 * User SPI setup function. Use this function to set up the SPI peripheral
 * on the microcontroller, such as to setup the IO, set the mode (0,0) for the
 * RFM69, and become a master.
 * @returns True on success, false on failure.
 */

int spi_ss;
rfm_status_t spi_set_ss(int pin)
{
    spi_ss = pin;
    return RFM_OK;
}

rfm_status_t spi_init(void)
{
    Serial.println("START spi_init");
    SPI.begin();
    pinMode(spi_ss, OUTPUT);
    digitalWrite(spi_ss, HIGH);

    Serial.println("END spi_init");
    /* Return RFM_OK if everything went ok, otherwise RFM_FAIL */
    return RFM_OK;
}

/**
 * User function to exchange a single byte over the SPI interface
 * @warn This does not handle SS, since higher level functions might want to do
 * burst read and writes
 * @param out The byte to be sent
 * @returns The byte received
 */
rfm_status_t spi_exchange_single(const rfm_reg_t out, rfm_reg_t* in)
{
    *in = SPI.transfer(out);
    return RFM_OK;
}

/**
 * User function to assert the slave select pin
 */
rfm_status_t spi_ss_assert(void)
{
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(spi_ss, LOW);
    return RFM_OK;
}

/**
 * User function to deassert the slave select pin
 */
rfm_status_t spi_ss_deassert(void)
{
    digitalWrite(spi_ss, HIGH);
    SPI.endTransaction();
    return RFM_OK;
}

