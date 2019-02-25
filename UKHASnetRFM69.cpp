/**
 * \mainpage
 * 
 * \section About
 * This file is part of the UKHASNet (ukhas.net) maintained RFM69 library for
 * use with all UKHASnet nodes, including Arduino, AVR and ARM.
 *
 * \section Authorship
 * Ported to Arduino 2014 James Coxon
 *
 * Ported, tidied and hardware abstracted by Jon Sowman, 2015
 *
 * Copyright (C) 2014 Phil Crump
 *
 * Copyright (C) 2015 Jon Sowman <jon@jonsowman.com>
 *
 * Based on RF22 Copyright (C) 2011 Mike McCauley
 *
 * Ported to mbed by Karl Zweimueller
 *
 * Based on RFM69 LowPowerLabs (https://github.com/LowPowerLab/RFM69/)
 *
 * @file ukhasnet-rfm69.c
 * @addtogroup ukhasnet-rfm69
 * @{
 */

//#define RFM_DEBUG

//#include <avr/io.h>
//#include <util/delay.h>
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <iso646.h>

#include "UKHASnetRFM69.h"
#include "UKHASnetRFM69-config.h"

const unsigned long MAXULONG = 0xffffffff;
unsigned long now;
unsigned long getTimeSince(unsigned long ___start) {
    unsigned long interval;
    now = millis();
    if (___start > now) {
        interval = MAXULONG - ___start + now;
    } else {
        interval = now - ___start;
    }
    return interval;
}

/** Track the current mode of the radio */
static rfm_reg_t _mode;

/* Private functions */
static rfm_status_t _rf69_fifo_write(const rfm_reg_t* src, uint8_t len);
static rfm_status_t _rf69_clear_fifo(void);

/**
 * Initialise the RFM69 device and set into SLEEP mode (0.1uA)
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
 
rfm_status_t rf69_init(int spi_ss)
{
    spi_set_ss(spi_ss);
    return rf69_init();
}

rfm_status_t rf69_init(void)
{
    uint8_t i;
    rfm_reg_t res;

    /* Call the user setup function to configure the SPI peripheral */
    if (spi_init() != RFM_OK)
        return RFM_FAIL;

    /* Zero version number, RFM probably not connected/functioning */
    rf69_read(RFM69_REG_10_VERSION, &res);
    if (!res)
        return RFM_FAIL;

    /* Set up device */
    for (i = 0; CONFIG[i][0] != 255; i++)
        rf69_write(CONFIG[i][0], CONFIG[i][1]);
    
    /* Set initial mode */
    rf69_set_mode(RFM69_MODE_SLEEP);

    return RFM_OK;
}

/**
 * Read a single byte from a register in the RFM69. Transmit the (one byte)
 * address of the register to be read, then read the (one byte) response.
 * @param reg The register address to be read
 * @param result A pointer to where to put the result
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_read(const rfm_reg_t reg, rfm_reg_t* result)
{
#ifdef RFM_DEBUG
    Serial1.print("# rf69_read: 0x");
    Serial1.println(reg, HEX);
#endif
    rfm_reg_t data;

    spi_ss_assert();

    /* Transmit the reg we want to read from */
    spi_exchange_single(reg, &data);

    /* Read the data back */
    spi_exchange_single(0xFF, result);

    spi_ss_deassert();

    return RFM_OK;
}

/**
 * Write a single byte to a register in the RFM69. Transmit the register
 * address (one byte) with the write mask RFM_SPI_WRITE_MASK on, and then the
 * value of the register to be written.
 * @param reg The address of the register to write
 * @param val The value for the address
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_write(const rfm_reg_t reg, const rfm_reg_t val)
{
#ifdef RFM_DEBUG
    Serial1.print("# rf69_write: 0x");
    Serial1.println(reg, HEX);
#endif
    rfm_reg_t dummy;

    spi_ss_assert();

    /* Transmit the reg address */
    spi_exchange_single(reg | RFM69_SPI_WRITE_MASK, &dummy);

    /* Transmit the value for this address */
    spi_exchange_single(val, &dummy);

    spi_ss_deassert();

    return RFM_OK;
}

/**
 * Read a given number of bytes from the given register address into a 
 * provided buffer
 * @param reg The address of the register to start from
 * @param dest A pointer into the destination buffer
 * @param len The number of bytes to read
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_burst_read(const rfm_reg_t reg, rfm_reg_t* dest, 
        uint8_t len)
{
#ifdef RFM_DEBUG
    Serial1.print("# rf69_burst_read: 0x");
    Serial1.println(reg, HEX);
#endif
    rfm_reg_t dummy;

    spi_ss_assert();
    
    /* Send the start address with the write mask off */
    spi_exchange_single(reg & ~RFM69_SPI_WRITE_MASK, &dummy);
    
    while (len--) {
        spi_exchange_single(0xFF, dest);
        dest++;
    }

    spi_ss_deassert();

    return RFM_OK;
}
/**
 * Write a given number of bytes into the registers in the RFM69.
 * @param reg The first byte address into which to write
 * @param src A pointer into the source data buffer
 * @param len The number of bytes to write
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_burst_write(rfm_reg_t reg, const rfm_reg_t* src, 
        uint8_t len)
{
#ifdef RFM_DEBUG
    Serial1.print("# rf69_burst_write: 0x");
    Serial1.println(reg, HEX);
#endif
    rfm_reg_t dummy;

    spi_ss_assert();
    
    /* Send the start address with the write mask on */
    spi_exchange_single(reg | RFM69_SPI_WRITE_MASK, &dummy); 

    while (len--)
        spi_exchange_single(*src++, &dummy);

    spi_ss_deassert();

    return RFM_OK;
}

/**
 * Write data into the FIFO on the RFM69
 * @param src The source data comes from this buffer
 * @param len Write this number of bytes from the buffer into the FIFO
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
static rfm_status_t _rf69_fifo_write(const rfm_reg_t* src, uint8_t len)
{
#ifdef RFM_DEBUG
    Serial1.println("# _rf69_fifo_write");
#endif
    rfm_reg_t dummy;

    spi_ss_assert();
    
    /* Send the start address with the write mask on */
    spi_exchange_single(RFM69_REG_00_FIFO | RFM69_SPI_WRITE_MASK, &dummy);
    
    /* First byte is packet length */
    spi_exchange_single(len, &dummy);

    /* Then write the packet */
    while (len--)
        spi_exchange_single(*src++, &dummy);

    spi_ss_deassert();

    return RFM_OK;
}

/**
 * Change the RFM69 operating mode to a new one.
 * @param newMode The value representing the new mode (see datasheet for
 * further information). The MODE bits are masked in the register, i.e. only
 * bits 2-4 of newMode are ovewritten in the register.
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_set_mode(const rfm_reg_t newMode)
{
#ifdef RFM_DEBUG
    Serial1.print("# rf69_set_mode: 0x");
    Serial1.println(newMode, HEX);
#endif
    rfm_reg_t res;
    rf69_read(RFM69_REG_01_OPMODE, &res);
    // Just return if correct mode is already set
    if ((res & 0x1C) == newMode) {
        _mode = newMode;
        return RFM_OK;
    }

    rf69_write(RFM69_REG_01_OPMODE, (res & 0xE3) | newMode);
    
    rf69_read(RFM69_REG_01_OPMODE, &res);
    while (!((res & 0x1C) == newMode)) {
        yield();
        rf69_read(RFM69_REG_01_OPMODE, &res);
    }

    _mode = newMode;
    return RFM_OK;
}

/**
 * Get data from the RFM69 receive buffer.
 * @param buf A pointer into the local buffer in which we would like the data.
 * @param len The length of the data
 * @param lastrssi The RSSI of the packet we're getting
 * @param rfm_packet_waiting A boolean pointer which is true if a packet was
 * received and has been put into the buffer buf, false if there was no packet
 * to get from the RFM69.
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_receive(rfm_reg_t* buf, rfm_reg_t* len, int16_t* lastrssi,
        bool* rfm_packet_waiting)
{
#ifdef RFM_DEBUG
    Serial1.println("# rf69_receive");
#endif
    rfm_reg_t res;

    if(_mode != RFM69_MODE_RX)
    {
        rf69_set_mode(RFM69_MODE_RX);
    }

    /* Check IRQ register for payloadready flag
     * (indicates RXed packet waiting in FIFO) */
    rf69_read(RFM69_REG_28_IRQ_FLAGS2, &res);
    if (res & RF_IRQFLAGS2_PAYLOADREADY)
    {
        /* Get packet length from first byte of FIFO */
        rf69_read(RFM69_REG_00_FIFO, len);
        *len += 1;
        /* Read FIFO into our Buffer */
        rf69_burst_read(RFM69_REG_00_FIFO, buf, RFM69_FIFO_SIZE);
        /* Read RSSI register (should be of the packet? - TEST THIS) */
        rf69_read(RFM69_REG_24_RSSI_VALUE, &res);
        *lastrssi = -(res/2);
        /* Clear the radio FIFO (found in HopeRF demo code) */
        _rf69_clear_fifo();

        *rfm_packet_waiting = true;
        return RFM_OK;
    }

    *rfm_packet_waiting = false;
    return RFM_OK;
}


// Automatically generated CRC function
// polynomial: 0x11021
uint16_t crc16_ccit(uint8_t *data, int len, uint16_t crc) {
    static const uint16_t table[256] = {
        0x0000U,0x1021U,0x2042U,0x3063U,0x4084U,0x50A5U,0x60C6U,0x70E7U,
        0x8108U,0x9129U,0xA14AU,0xB16BU,0xC18CU,0xD1ADU,0xE1CEU,0xF1EFU,
        0x1231U,0x0210U,0x3273U,0x2252U,0x52B5U,0x4294U,0x72F7U,0x62D6U,
        0x9339U,0x8318U,0xB37BU,0xA35AU,0xD3BDU,0xC39CU,0xF3FFU,0xE3DEU,
        0x2462U,0x3443U,0x0420U,0x1401U,0x64E6U,0x74C7U,0x44A4U,0x5485U,
        0xA56AU,0xB54BU,0x8528U,0x9509U,0xE5EEU,0xF5CFU,0xC5ACU,0xD58DU,
        0x3653U,0x2672U,0x1611U,0x0630U,0x76D7U,0x66F6U,0x5695U,0x46B4U,
        0xB75BU,0xA77AU,0x9719U,0x8738U,0xF7DFU,0xE7FEU,0xD79DU,0xC7BCU,
        0x48C4U,0x58E5U,0x6886U,0x78A7U,0x0840U,0x1861U,0x2802U,0x3823U,
        0xC9CCU,0xD9EDU,0xE98EU,0xF9AFU,0x8948U,0x9969U,0xA90AU,0xB92BU,
        0x5AF5U,0x4AD4U,0x7AB7U,0x6A96U,0x1A71U,0x0A50U,0x3A33U,0x2A12U,
        0xDBFDU,0xCBDCU,0xFBBFU,0xEB9EU,0x9B79U,0x8B58U,0xBB3BU,0xAB1AU,
        0x6CA6U,0x7C87U,0x4CE4U,0x5CC5U,0x2C22U,0x3C03U,0x0C60U,0x1C41U,
        0xEDAEU,0xFD8FU,0xCDECU,0xDDCDU,0xAD2AU,0xBD0BU,0x8D68U,0x9D49U,
        0x7E97U,0x6EB6U,0x5ED5U,0x4EF4U,0x3E13U,0x2E32U,0x1E51U,0x0E70U,
        0xFF9FU,0xEFBEU,0xDFDDU,0xCFFCU,0xBF1BU,0xAF3AU,0x9F59U,0x8F78U,
        0x9188U,0x81A9U,0xB1CAU,0xA1EBU,0xD10CU,0xC12DU,0xF14EU,0xE16FU,
        0x1080U,0x00A1U,0x30C2U,0x20E3U,0x5004U,0x4025U,0x7046U,0x6067U,
        0x83B9U,0x9398U,0xA3FBU,0xB3DAU,0xC33DU,0xD31CU,0xE37FU,0xF35EU,
        0x02B1U,0x1290U,0x22F3U,0x32D2U,0x4235U,0x5214U,0x6277U,0x7256U,
        0xB5EAU,0xA5CBU,0x95A8U,0x8589U,0xF56EU,0xE54FU,0xD52CU,0xC50DU,
        0x34E2U,0x24C3U,0x14A0U,0x0481U,0x7466U,0x6447U,0x5424U,0x4405U,
        0xA7DBU,0xB7FAU,0x8799U,0x97B8U,0xE75FU,0xF77EU,0xC71DU,0xD73CU,
        0x26D3U,0x36F2U,0x0691U,0x16B0U,0x6657U,0x7676U,0x4615U,0x5634U,
        0xD94CU,0xC96DU,0xF90EU,0xE92FU,0x99C8U,0x89E9U,0xB98AU,0xA9ABU,
        0x5844U,0x4865U,0x7806U,0x6827U,0x18C0U,0x08E1U,0x3882U,0x28A3U,
        0xCB7DU,0xDB5CU,0xEB3FU,0xFB1EU,0x8BF9U,0x9BD8U,0xABBBU,0xBB9AU,
        0x4A75U,0x5A54U,0x6A37U,0x7A16U,0x0AF1U,0x1AD0U,0x2AB3U,0x3A92U,
        0xFD2EU,0xED0FU,0xDD6CU,0xCD4DU,0xBDAAU,0xAD8BU,0x9DE8U,0x8DC9U,
        0x7C26U,0x6C07U,0x5C64U,0x4C45U,0x3CA2U,0x2C83U,0x1CE0U,0x0CC1U,
        0xEF1FU,0xFF3EU,0xCF5DU,0xDF7CU,0xAF9BU,0xBFBAU,0x8FD9U,0x9FF8U,
        0x6E17U,0x7E36U,0x4E55U,0x5E74U,0x2E93U,0x3EB2U,0x0ED1U,0x1EF0U,
    };

    while (len > 0) {
        crc = table[*data ^ (uint8_t)(crc >> 8)] ^ (crc << 8);
        data++;
        len--;
    }
    return crc;
}

rfm_status_t rf69_receive_long(rfm_reg_t* buf, uint16_t* len, float* lastrssi, bool* rfm_packet_waiting, const uint16_t bufsize, const int DIO1_pin, const unsigned long timeout) {
    unsigned long start = millis();
#ifdef RFM_DEBUG
    Serial1.println("# rf69_receive_long");
#endif
    *len = 0;
    rfm_reg_t res;

    /***** Set up DIO1 **********************************************/
    pinMode(DIO1_pin, INPUT);
    rf69_read(RFM69_REG_25_DIO_MAPPING1, &res);
#ifdef RFM_DEBUG
    Serial1.print(">RFM69_REG_25_DIO_MAPPING1: ");
    Serial1.println(res, BIN);
#endif
    res &= ~RF_DIOMAPPING1_DIO1_11; // Clear current DIO1 value
    res |=  RF_DIOMAPPING1_DIO1_10; // Set DIO1 to 10 (FifoNotEmpty)
#ifdef RFM_DEBUG
    Serial1.print("<RFM69_REG_25_DIO_MAPPING1: ");
    Serial1.println(res, BIN);
#endif
    rf69_write(RFM69_REG_25_DIO_MAPPING1, res);
    /****************************************************************/

    /***** Set up RX Settings **************************************/
    // PacketFormat=0 + PayloadLength=0 == Unlimited Length Packet Format
    rf69_read(RFM69_REG_37_PACKET_CONFIG1, &res);
    // Disable CRC(not possible in infinite packet RX), set PacketFormat=0
    res &= ~(RF_PACKET1_CRC_ON | RF_PACKET1_FORMAT_VARIABLE);
    rf69_write(RFM69_REG_37_PACKET_CONFIG1, res);
    rf69_write(RFM69_REG_38_PAYLOAD_LENGTH, 0); // set PayloadLength=0
    /****************************************************************/

    if (_mode != RFM69_MODE_RX) {
        rf69_set_mode(RFM69_MODE_RX);
    }

#ifdef RFM_DEBUG
    Serial1.println("# rf69_receive_long % Waiting for FifoNotEmpty");
#endif
    // Wait for FifoNotEmpty flag
    while (!digitalRead(DIO1_pin)) { // while FIFO is empty
        if (timeout) { // Default is 0, or no timeout.
            if (getTimeSince(start) > timeout) {
                return RFM_TIMEOUT;
            }
        }
        yield();
    }
#ifdef RFM_DEBUG
    Serial1.println("# rf69_receive_long % FifoNotEmpty");
#endif

    /***** Receive packet *******************************************/
    spi_ss_assert();
    
    /* Send the start address with the write mask off */
    spi_exchange_single(RFM69_REG_00_FIFO & ~RFM69_SPI_WRITE_MASK, &res);

    rfm_status_t err = RFM_OK;
    long psize = -1;
    bool packet_complete = false;

#ifdef RFM_DEBUG
    Serial1.println("# rf69_receive_long % Starting FIFO read loop");
#endif
    while (true) {
        yield();
        start = millis();
        spi_exchange_single(0xFF, &res);
        buf[(*len)++] = res;

        if (psize > 0) {
            if (buf[0] == 0) {
                //* null (1) + size (2) + packet (n) + crc (2)
                if (*len >= (psize + 5)) {
                    packet_complete = true;
                    break; // Packet received
                }
            } else {
                //* size byte (1) + packet (n) + crc (2)
                if (*len >= (psize + 3)) {
                    packet_complete = true;
                    break; // Packet received
                }
            }
        }

        // Make sure we don't exceed the buffer size (might result in incomplete packets if bufsize is too small)
        if (*len >= bufsize) {
            err = RFM_BUFFER_OVERFLOW;
            break;
        }

        switch (*len) {
            case 1:
                if (buf[0] != 0) {
                    psize = buf[0];
                    if (bufsize < (psize + 3)) {
                        err = RFM_BUFFER_OVERFLOW;
                    }
                }
                break;
            case 5:
                //* If standard packet size is 0,
                //* interpet the next 2 bytes as a big-endian uint16_t size field ">H".
                if (buf[0] == 0) {
                    psize = (buf[1] << 8) | buf[2];
                    if (bufsize < (psize + 5)) {
                        err = RFM_BUFFER_OVERFLOW;
                    }
                }
                break;
            default:
                break;
        }
        
        if (err != RFM_OK) {
            break;
        }

        // while FIFO is empty
        while (!digitalRead(DIO1_pin)) { 
            if (getTimeSince(start) >= 10) { // Seems to be about 4ms between bytes
                err = RFM_TIMEOUT;
                break;
            }
            yield();
        }

        if (err != RFM_OK) {
            break;
        }
    }
#ifdef RFM_DEBUG
    Serial1.print("# rf69_receive_long % FIFO read loop done, received: ");
    Serial1.println(*len, DEC);
#endif
    spi_ss_deassert();
    /****************************************************************/

    
    //* Read RSSI register (should be of the packet? - TEST THIS)
    //? From SX1231 Datasheet 3.5.9 - RSSI (Page 29-30)
    //! "The RSSI sampling must occur during the reception of preamble..."
    rf69_read(RFM69_REG_24_RSSI_VALUE, &res);
    *lastrssi = -(((float)res)/2);
    /* Clear the radio FIFO (found in HopeRF demo code) */
    _rf69_clear_fifo();

    if (packet_complete) {
        // crc16-ccit initial value is 0x1D0F
        uint16_t packet_crc = (buf[(*len)-2] << 8) | buf[(*len)-1];
        
        //! CRC16-CCITT (polynomial: 0x11021), initial=0x1D0F
        //! Checksum is XOR 0xFFFF
        uint16_t crc = crc16_ccit(buf, (*len)-2, 0x1D0F) ^ 0xFFFF;

        if (crc == packet_crc) {
            // Remove length byte(s) and crc
            *len -= 2;
            uint16_t offset = buf[0] ? 1 : 3;
            for (uint16_t i=0; i < *len; i++) {
                buf[i] = buf[i+offset];
            }
            *len -= offset;

            *rfm_packet_waiting = true;
            return err;
        } else {
            err = RFM_CRC_ERROR;
        }
    } else {
        err = RFM_FAIL;
    }

    *rfm_packet_waiting = false;
    return err;
}

/**
 * Send a packet using the RFM69 radio.
 * @param data The data buffer that contains the string to transmit
 * @param len The number of bytes in the data packet (excluding preamble, sync
 * and checksum)
 * @param power The transmit power to be used in dBm
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_send(const rfm_reg_t* data, uint8_t len, 
        const uint8_t power)
{
#ifdef RFM_DEBUG
    Serial1.println("# rf69_send");
#endif
    rfm_reg_t oldMode, res;
    uint8_t paLevel;

    /* power is TX Power in dBmW (valid values are 2dBmW-20dBmW) */
    if (power < 2 || power > 20)
    {
        /* Could be dangerous, so let's check this */
        return RFM_FAIL;
    }

    oldMode = _mode;
    
    /* Start transmitter */
    rf69_set_mode(RFM69_MODE_TX);

    /* Set up PA */
    if (power <= 17) {
        /* Set PA Level */
        paLevel = power + 28;
        rf69_write(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | paLevel);        
    } else {
        /* Disable Over Current Protection */
        rf69_write(RFM69_REG_13_OCP, RF_OCP_OFF);
        /* Enable High Power Registers */
        rf69_write(RFM69_REG_5A_TEST_PA1, 0x5D);
        rf69_write(RFM69_REG_5C_TEST_PA2, 0x7C);
        /* Set PA Level */
        paLevel = power + 11;
        rf69_write(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }

    /* Wait for PA ramp-up */
    res = 0;
    while (!(res & RF_IRQFLAGS1_TXREADY))
        rf69_read(RFM69_REG_27_IRQ_FLAGS1, &res);

    /* Throw Buffer into FIFO, packet transmission will start 
     * automatically */
    _rf69_fifo_write(data, len);

    /* Wait for packet to be sent */
    res = 0;
    while (!(res & RF_IRQFLAGS2_PACKETSENT))
        rf69_read(RFM69_REG_28_IRQ_FLAGS2, &res);

    /* Return Transceiver to original mode */
    rf69_set_mode(oldMode);

    /* If we were in high power, switch off High Power Registers */
    if (power > 17) {
        /* Disable High Power Registers */
        rf69_write(RFM69_REG_5A_TEST_PA1, 0x55);
        rf69_write(RFM69_REG_5C_TEST_PA2, 0x70);
        /* Enable Over Current Protection */
        rf69_write(RFM69_REG_13_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
    }

    return RFM_OK;
}

rfm_status_t rf69_send_long(const rfm_reg_t* data, uint16_t len, const uint8_t power, const int DIO1_pin) {
#ifdef RFM_DEBUG
    Serial1.println("# rf69_send_long");
#endif
    rfm_reg_t oldMode, res;
    uint8_t paLevel;

    /* power is TX Power in dBmW (valid values are 2dBmW-20dBmW) */
    if (power < 2 || power > 20) {
        /* Could be dangerous, so let's check this */
        return RFM_FAIL;
    }

    /***** Set up DIO1 **********************************************/
    pinMode(DIO1_pin, INPUT);

    rf69_read(RFM69_REG_25_DIO_MAPPING1, &res);
    res &= ~RF_DIOMAPPING1_DIO1_11; // Clear current DIO1 value
    res |=  RF_DIOMAPPING1_DIO1_01; // Set DIO1 to 01 (FifoFull)
    rf69_write(RFM69_REG_25_DIO_MAPPING1, res);
    /****************************************************************/

    /***** Set up TX Settings **************************************/
    // PacketFormat=0 + PayloadLength=0 == Unlimited Length Packet Format
    rf69_read(RFM69_REG_37_PACKET_CONFIG1, &res);
    res &= ~(RF_PACKET1_FORMAT_VARIABLE);       //* set PacketFormat=0
    res |= RF_PACKET1_CRC_ON;                   //* Enable CRC generation
    rf69_write(RFM69_REG_37_PACKET_CONFIG1, res);

    rf69_write(RFM69_REG_38_PAYLOAD_LENGTH, 0); //* set PayloadLength=0

    
    if (len < 5) {
        //! Start transmitting on complete packet if packet is shorter than 5 (length byte(1) + packet length(len))
        rf69_write(RFM69_REG_3C_FIFO_THRESHOLD, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | len + 1);
    } else if (len > 0x1F) {
        //! Start transmitting on 31 packets in buffer if packet length is larger than that
        rf69_write(RFM69_REG_3C_FIFO_THRESHOLD, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x1F);
    } else {
        //! Start TX on: short style length(1 byte) + minimal packet (0a[Q])
        rf69_write(RFM69_REG_3C_FIFO_THRESHOLD, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x06);
    }
    
    /****************************************************************/

    oldMode = _mode;
    
    /* Start transmitter */
    rf69_set_mode(RFM69_MODE_TX);

    /* Set up PA */
    if (power <= 17) {
        /* Set PA Level */
        paLevel = power + 28;
        rf69_write(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | paLevel);        
    } else {
        /* Disable Over Current Protection */
        rf69_write(RFM69_REG_13_OCP, RF_OCP_OFF);
        /* Enable High Power Registers */
        rf69_write(RFM69_REG_5A_TEST_PA1, 0x5D);
        rf69_write(RFM69_REG_5C_TEST_PA2, 0x7C);
        /* Set PA Level */
        paLevel = power + 11;
        rf69_write(RFM69_REG_11_PA_LEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | paLevel);
    }

    /* Wait for PA ramp-up */
    rf69_read(RFM69_REG_27_IRQ_FLAGS1, &res);
    while (!(res & RF_IRQFLAGS1_TXREADY)) {
        yield();
        rf69_read(RFM69_REG_27_IRQ_FLAGS1, &res);
    }

    spi_ss_assert();
    
    /* Send the start address with the write mask on */
    spi_exchange_single(RFM69_REG_00_FIFO | RFM69_SPI_WRITE_MASK, &res);
    
    if (len <= 0xff) {
        /* First byte is packet length */
        spi_exchange_single((uint8_t)len, &res);
    } else {
        spi_exchange_single(0x00, &res);
        spi_exchange_single((len >> 8) & 0xFF, &res); //! MSB first
        spi_exchange_single(len & 0xFF, &res);
    }

    /* Throw Buffer into FIFO, packet transmission will start automatically */
    for (uint16_t i=0; i<len; i++) {
        // While FIFO is full, do other things
        while (digitalRead(DIO1_pin)) {
            yield();
        }

        // Put a byte of data into the FIFO
        spi_exchange_single(data[i], &res);
    }

    spi_ss_deassert();

    /* Wait for packet to be sent */
    rf69_read(RFM69_REG_28_IRQ_FLAGS2, &res);
    while (!(res & RF_IRQFLAGS2_PACKETSENT)) {
        yield();
        rf69_read(RFM69_REG_28_IRQ_FLAGS2, &res);
    }
        

    /* Return Transceiver to original mode */
    rf69_set_mode(oldMode);

    /* If we were in high power, switch off High Power Registers */
    if (power > 17) {
        /* Disable High Power Registers */
        rf69_write(RFM69_REG_5A_TEST_PA1, 0x55);
        rf69_write(RFM69_REG_5C_TEST_PA2, 0x70);
        /* Enable Over Current Protection */
        rf69_write(RFM69_REG_13_OCP, RF_OCP_ON | RF_OCP_TRIM_95);
    }

    return RFM_OK;
}

/**
 * Clear the FIFO in the RFM69. We do this by entering STBY mode and then
 * returing to RX mode.
 * @warning Must only be called in RX Mode
 * @note Apparently this works... found in HopeRF demo code
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
static rfm_status_t _rf69_clear_fifo(void)
{
    rf69_set_mode(RFM69_MODE_STDBY);
    rf69_set_mode(RFM69_MODE_RX);
    return RFM_OK;
}

/**
 * The RFM69 has an onboard temperature sensor, read its value
 * @warning RFM69 must be in one of the active modes for temp sensor to work.
 * @param temperature A pointer to the variable into which the temperature will
 * be read by this method.
 * @returns RFM_OK for success, RFM_FAIL for failure, RFM_TIMEOUT if there is a
 * timeout due to the sensor on the RFM not starting and/or finishing a
 * conversion.
 */
rfm_status_t rf69_read_temp(int8_t* temperature)
{
    /* Store current transceiver mode */
    rfm_reg_t oldMode, temp;
    uint8_t timeout;
    
    oldMode = _mode;
    /* Set mode into Standby (required for temperature measurement) */
    rf69_set_mode(RFM69_MODE_STDBY);

    /* Trigger Temperature Measurement */
    rf69_write(RFM69_REG_4E_TEMP1, RF_TEMP1_MEAS_START);

    /* Check Temperature Measurement has started */
    timeout = 0;
    temp = 0;
    while (!(RF_TEMP1_MEAS_RUNNING & temp)) {
        rf69_read(RFM69_REG_4E_TEMP1, &temp);
        delay(1);
        if(++timeout > 50)
        {
            *temperature = -127.0;
            return RFM_TIMEOUT;
        }
        rf69_write(RFM69_REG_4E_TEMP1, RF_TEMP1_MEAS_START);
    }

    /* Wait for Measurement to complete */
    timeout = 0;
    temp = 0;
    while (RF_TEMP1_MEAS_RUNNING & temp) {
        rf69_read(RFM69_REG_4E_TEMP1, &temp);
        delay(1);
        if(++timeout > 10)
        {
            *temperature = -127.0;
            return RFM_TIMEOUT;
        }
    }

    /* Read raw ADC value */
    temp = 0;
    rf69_read(RFM69_REG_4F_TEMP2, &temp);
	
    /* Set transceiver back to original mode */
    rf69_set_mode(oldMode);

    /* Return processed temperature value */
    *temperature = 161 - (int8_t)temp;

    return RFM_OK;
}

/**
 * Get the last RSSI value from the RFM69
 * @warning Must only be called when the RFM69 is in rx mode
 * @param rssi A pointer to an int16_t where we will place the RSSI value
 * @returns RFM_OK for success, RFM_FAIL for failure.
 */
rfm_status_t rf69_sample_rssi(int16_t* rssi)
{
    rfm_reg_t res;

    /* Must only be called in RX mode */
    if (_mode != RFM69_MODE_RX)
        return RFM_FAIL;

    /* Trigger RSSI Measurement */
    rf69_write(RFM69_REG_23_RSSI_CONFIG, RF_RSSI_START);

    /* Wait for Measurement to complete */
    while (!(RF_RSSI_DONE & res))
        rf69_read(RFM69_REG_23_RSSI_CONFIG, &res);

    /* Read, store in _lastRssi and return RSSI Value */
    res = 0;
    rf69_read(RFM69_REG_24_RSSI_VALUE, &res);
    *rssi = -(res/2);

    return RFM_OK;
}

/**
 * @}
 */
