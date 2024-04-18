/*
 * xPM25.h
 *
 *  Created on: Apr 16, 2024
 *      Author: coryg
 *
 *      @brief STM32 driver for PM25 Air Quality sensor. FreeRTOS enabled.
 *      @prerequisites This driver requires a dedicated UART channel. UART callbacks are
 *      	used.
 */

#ifndef INC_XPM25_H_
#define INC_XPM25_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include "cmsis_os.h"

#if (USE_HAL_UART_REGISTER_CALLBACKS!=1U)
	#error "USE_HAL_UART_REGISTER_CALLBACKS Must be set to 1U in stm32lxx_hal_conf.h"
#endif

#define DEFAULT_TIMEOUT osWaitForever

extern "C" {

/**! Structure holding Plantower's standard packet **/
typedef struct PMSAQIdata {
	uint16_t framelen;       ///< How long this data chunk is
	uint16_t pm10_standard,  ///< Standard PM1.0
			pm25_standard,       ///< Standard PM2.5
			pm100_standard;      ///< Standard PM10.0
	uint16_t pm10_env,       ///< Environmental PM1.0
			pm25_env,            ///< Environmental PM2.5
			pm100_env;           ///< Environmental PM10.0
	uint16_t particles_03um, ///< 0.3um Particle Count
			particles_05um,      ///< 0.5um Particle Count
			particles_10um,      ///< 1.0um Particle Count
			particles_25um,      ///< 2.5um Particle Count
			particles_50um,      ///< 5.0um Particle Count
			particles_100um;     ///< 10.0um Particle Count
	uint16_t unused;         ///< Unused
	uint16_t checksum;       ///< Packet checksum
} PM25_AQI_Data;

/*
 * Class definition for pms5003 PM25 Air Quality Sensor/Dust Particle Counter
 */
class PM25 {
public:
	PM25(UART_HandleTypeDef *_huart); //constructor
	bool begin(); // Starts driver.
	bool newData(); // Check if new data is available
	bool getData(PM25_AQI_Data *_data); // copy PM25 data to user structure
	bool crc8(); // perform CRC8 check
	void setDataReadyCallback(void (*_funcptr)(void));
	void setTimeout(uint32_t _timeout);
	UART_HandleTypeDef* getPm25Uart(); // function to get the UART definition
	osMessageQueueId_t getPm25DataQueue(); // function to get OS Message Queue
	osThreadId_t getPM25TaskHandle();
	PM25_AQI_Data data, *dataPtr; // AQI data
	void (*funcPtr)(void);
	uint32_t timeout;
protected:
private:
	osMessageQueueId_t dataQueue; // OS Message Queue to buffer PM25 data
	osThreadId_t PM25_Task_HandlHandle; // OS Task handle
	UART_HandleTypeDef *huart; // pointer to the UART
	bool _newdata = false; // Flag for new data.
}; // end class definition
} // end extern "C" {}
#endif /* INC_XPM25_H_ */
