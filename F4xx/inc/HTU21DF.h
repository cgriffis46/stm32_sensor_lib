/*
 * HTU21DF.h
 *
 *  Created on: Mar 20, 2024
 *      Author: coryg
 *      @version 1.0.0
 *
 *      This is a redistribution of Adafruit's fine HTU21DF library, ported for STM32
 */

/*!
 * @file Adafruit_HTU21DF.cpp
 *
 * @mainpage Adafruit HTU21DF Sensor
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HTU21DF Humidity & Temp Sensor
 *
 * Designed specifically to work with the HTU21DF sensor from Adafruit
 * ----> https://www.adafruit.com/products/1899
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#ifndef HTU21DF_HTU21DF_H_
#define HTU21DF_HTU21DF_H_

extern "C"{

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/** Default I2C address for the HTU21D. */
static const uint16_t HTU21DF_I2CADDR = (0x40);

/** Read temperature register. */
#define HTU21DF_READTEMP (0xE3)

/** Read humidity register. */
#define HTU21DF_READHUM (0xE5)

/** Write register command. */
#define HTU21DF_WRITEREG (0xE6)

/** Read register command. */
#define HTU21DF_READREG (0xE7)

/** Reset command. */
#define HTU21DF_RESET (0xFE)

/**
 * Driver for the HTU21DF sensor
 */
class HTU21DF {
public:
  HTU21DF(I2C_HandleTypeDef* _hi2c);

  bool begin(uint8_t devAddress);
  float readTemperature(void);
  float readHumidity(void);
  void reset(void);

private:
  float _last_humidity, _last_temp;
protected:
  uint8_t devAddress = HTU21DF_I2CADDR;
  I2C_HandleTypeDef * hi2c;
};

}

#endif /* HTU21DF_HTU21DF_H_ */
