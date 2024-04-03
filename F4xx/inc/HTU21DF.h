/*
 * HTU21DF.h
 *
 *  Created on: Mar 20, 2024
 *      Author: coryg
 */

#ifndef HTU21DF_HTU21DF_H_
#define HTU21DF_HTU21DF_H_

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
 * Driver for the Adafruit HTU21DF breakout board.
 */
class Adafruit_HTU21DF {
public:
  Adafruit_HTU21DF(I2C_HandleTypeDef* _hi2c);

  bool begin(uint8_t devAddress);
  float readTemperature(void);
  float readHumidity(void);
  void reset(void);

private:
  //Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  float _last_humidity, _last_temp;
protected:
  //i2c_inst_t i2c;
  uint8_t devAddress = HTU21DF_I2CADDR;
  I2C_HandleTypeDef * hi2c;
};



#endif /* HTU21DF_HTU21DF_H_ */
