/*
 * xPM25.cpp
 * @version 1.0.2
 *
 *  Created on: Apr 16, 2024
 *      Author: coryg
 */

#include "xPM25.h"
#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern "C"{

static PM25 *pm25SelfPointer;

uint8_t rxBuffer[32];// buffer for the UART.
uint8_t tempData[32]; // buffer to read the PM25 data from an OS Message Queue

/* Definitions for PM25_Task_Handl */
const osThreadAttr_t PM25_Task_Handl_attributes = {
  .name = "PM25_Task_Handl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityISR,
};

/* Definitions for pm25_rx_q_handle */
const osMessageQueueAttr_t pm25_rx_q_handle_attributes = {
  .name = "pm25_rx_q_handle"
};

PM25::PM25(UART_HandleTypeDef *_huart) {
	huart=_huart;
	pm25SelfPointer = this;
	_newdata=false;
	dataPtr=&data;
	funcPtr=NULL;}

bool PM25::getData(PM25_AQI_Data *_data){
		memcpy((void*)_data,dataPtr, sizeof(data));
		_newdata=false;
		return true;
}

UART_HandleTypeDef* PM25::getPm25Uart(){
	return this->huart;
}

bool PM25::newData(){
	return _newdata;
}

bool PM25::crc8(){
	  uint16_t sum = 0;
	  uint16_t csum = 0;
	  // check CRC
	  for (uint8_t i = 0; i < 30; i++) {
	        csum += tempData[i];
	      }
	  // The data comes in endian'd, this solves it so it works on all platforms
	      uint16_t buffer_u16[15];
	      for (uint8_t i = 0; i < 15; i++) {
	        buffer_u16[i] = tempData[2 + i * 2 + 1];
	        buffer_u16[i] += (tempData[2 + i * 2] << 8);
	      }
	      // put it into a nice struct :)
	      memcpy((void *)this->dataPtr, (void *)buffer_u16, 32);

		 if(csum==buffer_u16[14]) {
			 _newdata=true;
			 return true;
		 }
		 return false;
}

/*
 * @brief Used by ISR to get message queue
 * @return returns the handle of the OS Message Queue
 */
osMessageQueueId_t PM25::getPm25DataQueue(){
	return this->dataQueue;
}

/* USER CODE BEGIN Header_PM25_Task_Func */
/**
* @brief Function Reads the PM25 message queue and performs CRC check.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PM25_Task_Func */

PM25_AQI_Data _tempData;

void PM25_Task_Func(void *argument)
{
  /* USER CODE BEGIN PM25_Task_Func */

  /* Infinite loop */
  for(;;)
  {
	if(osMessageQueueGet(pm25SelfPointer->getPm25DataQueue(),&tempData,0U,0U)==osOK){ // Wait indefinitely for a message from PM25
		if(tempData[0]==0x42&&tempData[1]==0x4d){ // PM25 sensor starts each transmission with 0x42 0x4d
			if(pm25SelfPointer->crc8()){// Perform CRC8 check
				if(pm25SelfPointer->funcPtr!=NULL){ // execute callback if necessary
					pm25SelfPointer->funcPtr();
				}
			}
		}

	}else{
		osEventFlagsWait(pm25SelfPointer->ef_id, 1U, osFlagsWaitAny, osWaitForever);
	}
  }
  /* USER CODE END PM25_Task_Func */
}

void xPM25_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  if(huart==pm25SelfPointer->getPm25Uart()){osMessageQueuePut(pm25SelfPointer->getPm25DataQueue(),rxBuffer, 0U, 0U);}
  //osThreadSetPriority (pm25SelfPointer->getPM25TaskHandle(),osPriorityISR); } // Put the data in a message queue for a task to process
  osEventFlagsSet(pm25SelfPointer->ef_id, 1U);
  HAL_UART_Receive_IT(pm25SelfPointer->getPm25Uart(),rxBuffer, sizeof(rxBuffer)); // Get ready for next message
}

bool PM25::begin(){
	_newdata=false; // initialize _newdata flag
	HAL_UART_RegisterCallback(this->huart,HAL_UART_RX_COMPLETE_CB_ID, xPM25_UART_RxCpltCallback); // setup UART callback
	this->dataQueue = osMessageQueueNew(4,32, &pm25_rx_q_handle_attributes); // create message queue to receive PM25 UART messages
	this->PM25_Task_HandlHandle = osThreadNew(PM25_Task_Func, NULL, &PM25_Task_Handl_attributes); // create task to process PM25 messages
	ef_id=osEventFlagsNew(NULL);
	if(HAL_UART_Receive_IT(this->huart,rxBuffer, sizeof(rxBuffer))!=HAL_OK) return false; // tell uart to expect message from PM25
	return true;
}

/*
 * @brief Set the data received callback.
 */
void PM25::setDataReadyCallback(void (*_funcptr)(void)){
	funcPtr=_funcptr;
}

/*
 * @brief returns the thread handle
 */
osThreadId_t PM25::getPM25TaskHandle(){
	return this->PM25_Task_HandlHandle;
}

}


