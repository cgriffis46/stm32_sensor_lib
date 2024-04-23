/*
 * lsm303dlhc.h
 *
 *  Created on: Apr 19, 2024
 *      Author: coryg
 *      @brief Adafruit's LSM303 driver ported to STM32 development boards, under STM32CubeIDE
 *
 */

/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef LSM303DLHC_H_
#define LSM303DLHC_H_

#include "stm32l4xx_hal.h"

/*
 * I2C Device Address
 */

#define ACCELEROMETER_ADR 0b00110010
#define MAGNETOMETER_ADR 0b00111100

/*
 * Register Address Definitions
 */
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define FIFO_CTRL_REG_A 0x2E
#define FIFO_SRC_REG_A 0x2F
#define INT1_CFG_A 0x30
#define INT1_SRC_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define INT2_CFG_A 0x34
#define INT2_SRC_A 0x35
#define INT2_THS_A 0x36
#define INT2_DURATION_A 0x37
#define CLICK_CFG_A 0x38
#define CLICK_SRC_A 0x39
#define CLICK_THS_A 0x3A
#define TIME_LIMIT_A 0x3B
#define TIME_LATENCY_A 0x3C
#define TIME_WINDOW_A 0x3D

#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08
#define SR_REG_M 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C
#define TEMP_OUT_H_M 0x31
#define TEMP_OUT_L_M 0x32

/*
 * CTRL_REG1_A Register Definitions
 */

#define ODRMask  0b11110000
#define LPenMask 0b00001000
#define ZenMask  0b00000100
#define YenMask  0b00000010
#define XenMask  0b00000001

#define ODRPowerDown 0b00000000
#define ODR1hz       0b00010000
#define ODR10hz      0b00100000
#define ODR25hz      0b00110000
#define ODR50hz      0b01000000
#define ODR100hz     0b01010000
#define ODR200hz     0b01100000
#define ODR400hz     0b01110000
#define ODR1620khz   0b10000000
#define ODR1344khz   0b10010000

/*
 * CTRL_REG2_A Register Definitions
 */

/*
 * CTRL_REG3_A
 */

/*
 * CTRL_REG4_A
 */

/*
 * CTRL_REG5_A
 */

/*
 * CTRL_REG6_A
 */

    typedef struct lsm303AccelData_r
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } lsm303AccelData_raw_t;

    typedef struct lsm303MagData_r
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } lsm303MagData_raw_t;

    typedef struct lsm303AccelData_f
    {
      float x;
      float y;
      float z;
    } lsm303AccelData;

    typedef struct lsm303MagData_f
    {
        float x;
        float y;
        float  z;
    } lsm303MagData;

    typedef enum
    {
      LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
      LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
      LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
      LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
      LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
      LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
      LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
      LSM303_MAGRATE_220                        = 0x07   // 200 Hz
    } lsm303MagRate;

    typedef enum
    {
      LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } lsm303MagGain;

#define SENSORS_GAUSS_TO_MICROTESLA (100) /**< Gauss to micro-Tesla multiplier */
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)

class lsm303dlhc{
public:
	lsm303dlhc(I2C_HandleTypeDef * _hi2c);
	bool begin();
	bool readAccelerometer(lsm303AccelData* _lsm303AccelData);
	bool readMagnetometer(lsm303MagData*_lsm303MagData);
	void setMagGain(lsm303MagGain gain);
	void setMagRate(lsm303MagRate rate);
private:
	bool LowPowerModeEnable;
protected:
	uint8_t MagnetometerAddress;
	uint8_t AccelerometerAddress;
	I2C_HandleTypeDef * hi2c;
	uint8_t devAddress;
	lsm303AccelData_raw_t rawAccelData;
	lsm303MagData_raw_t rawMagData;
	lsm303AccelData accelData;
	lsm303MagData magData;
	lsm303MagGain   magGain;
};

#endif /* LSM303DLHC_H_ */
