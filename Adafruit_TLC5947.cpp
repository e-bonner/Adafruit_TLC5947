/*!
 *  @file Adafruit_TLC5947.cpp
 *
 *  @mainpage Adafruit 24-channel PWM/LED driver
 *
 *  @section intro_sec Introduction
 *
 * 	Driver for Microchip's TLC5947
 *
 * 	This is a library for the Adafruit TLC5947
 * 	http://www.adafruit.com/products/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include <Adafruit_TLC5947.h>
#include <SPI.h>

// TLC5947 datasheet says can go up to 30 MHz for single board, see page 3
SPISettings SPI_SETTINGS(5000000, MSBFIRST, SPI_MODE0);

/*!
 *    @brief  Instantiates a new TLC5947 class
 *    @param  n
 *            num of drivers (boards)
 *    @param  c
 *            Arduino pin connected to TLC5947 clock pin
 *    @param  d
 *            Arduino pin connected to TLC5947 data pin
 *    @param  l
 *            Arduino pin connected to TLC5947 latch pin
 */
Adafruit_TLC5947::Adafruit_TLC5947(uint16_t n, int8_t c, int8_t d,
                                   int8_t l) {
  numdrivers = n;
  _clk = c;
  _dat = d;
  _lat = l;

  pwmbuffer = (uint16_t *)malloc(2 * 24 * n);
  memset(pwmbuffer, 0, 2 * 24 * n);
}

/*!
 *    @brief  Instantiates a new TLC5947 class
 *    @param  n
 *            num of drivers (boards)
 *    @param  l
 *            Arduino pin connected to TLC5947 latch pin
 *    @param  *theSPI
 *            spi object
 */
Adafruit_TLC5947::Adafruit_TLC5947(uint16_t n, int8_t l, SPIClass *theSPI = &SPI) {
  numdrivers = n;
  _clk = -1;
  _dat = -1;
  _lat = l;
  _spi = theSPI;

  pwmbuffer = (uint16_t *)malloc(2 * 24 * n);
  memset(pwmbuffer, 0, 2 * 24 * n);
  // pwmbuffer array has 2 bytes per channel, need to send 12 bits over SPI
  spibuffer = (uint8_t *)malloc(2 * 24 * n / 4 * 3);
}


/*!
 *    @brief  Writes PWM data to the all connected TLC5947 boards
 */
void Adafruit_TLC5947::write() {
  uint16_t bit; // Used to iterate through spibuffer bits, starts high
  if (_clk < 0) {
    memset(spibuffer, 0, 2 * 24 * numdrivers / 4 * 3);
    bit = 0;
  }

  digitalWrite(_lat, LOW);
  // 24 channels per TLC5974
  for (int16_t c = 24 * numdrivers - 1; c >= 0; c--) {
    if (_clk >= 0) {
      // 12 bits per channel, send MSB first
      for (int8_t b = 11; b >= 0; b--) {
        digitalWrite(_clk, LOW);

        if (pwmbuffer[c] & (1 << b)) {
          digitalWrite(_dat, HIGH);
        } else {
          digitalWrite(_dat, LOW);
        }
          
        digitalWrite(_clk, HIGH);
      }
    } else {
      // 12 bits per channel, write MSB first into spibuffer
      for (int8_t b = 11; b >= 0; b--) {
        if (pwmbuffer[c] & (1 << b))
          spibuffer[bit / 8] |= (0x80 >> (bit % 8));
          
        bit++;
      }
    }
  }
  if (_clk >= 0) {
    digitalWrite(_clk, LOW);
  } else {
    _spi->beginTransaction(SPI_SETTINGS);
    _spi->transfer(spibuffer, 2 * 24 * numdrivers / 4 * 3);
    _spi->endTransaction();
  }

  digitalWrite(_lat, HIGH);
  digitalWrite(_lat, LOW);
}

/*!
 *    @brief  Set the PWM channel / value
 *    @param  chan
 *            channel number ([0 - 23] on each board, so chanel 2 for second board will be 25)
 *    @param  pwm
 *            pwm value [0-4095]
 */
void Adafruit_TLC5947::setPWM(uint16_t chan, uint16_t pwm) {
  if (pwm > 4095)
    pwm = 4095;
  if (chan > 24 * numdrivers)
    return;
  pwmbuffer[chan] = pwm;
}

/*!
 *    @brief  Set LED
 *    @param  lednum
 *            led number
 *    @param  r
 *            red value [0-255]
 *    @param  g
 *            green value [0-255]
 *    @param  b
 *            blue value [0-255]
 */
void Adafruit_TLC5947::setLED(uint16_t lednum, uint16_t r, uint16_t g,
                              uint16_t b) {
  setPWM(lednum * 3, r);
  setPWM(lednum * 3 + 1, g);
  setPWM(lednum * 3 + 2, b);
}

/*!
 *    @brief  Setups the HW
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_TLC5947::begin() {
  if (!pwmbuffer)
    return false;

  if (_clk >= 0) {
    pinMode(_clk, OUTPUT);
    pinMode(_dat, OUTPUT);
  } else {
    _spi->begin();
  }
  pinMode(_lat, OUTPUT);
  digitalWrite(_lat, LOW);

  return true;
}
