/*!
 *  @file Adafruit_TLC5947.h
 *
 * 	Adafruit 24-channel PWM/LED driver
 *
 * 	This is a library for the Adafruit 24-channel PWM/LED driver:
 * 	http://www.adafruit.com/products/1429
 *
 *  These drivers uses SPI to communicate, 3 pins are required to
 *  interface: Data, Clock and Latch. The boards are chainable
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_TLC5947_H
#define _ADAFRUIT_TLC5947_H

#include <Arduino.h>
#include <SPI.h>

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            TLC5947 24-channel PWM/LED driver
 */
class Adafruit_TLC5947 {
public:
  Adafruit_TLC5947(uint16_t n, int8_t c, int8_t d, int8_t l);
  Adafruit_TLC5947(uint16_t n, int8_t l, SPIClass *theSPI = &SPI);

  boolean begin(void);

  void setPWM(uint16_t chan, uint16_t pwm);
  void setLED(uint16_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void write();

private:
  uint16_t *pwmbuffer;
  uint8_t *spibuffer;

  uint16_t numdrivers;
  int8_t _clk, _dat, _lat;
  SPIClass *_spi;
};

#endif
