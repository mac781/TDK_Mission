/*
 * communication.h
 *
 *  Created on: Aug 8, 2024
 *      Author: macub
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stm32f4xx_hal.h"

void UART_setup();
void UART_TransmitData_Status_IT(UART_HandleTypeDef *huart, float data1, float data2, float data3,float data4);
void UART_ReceiveData_arrriveDestination_IT(UART_HandleTypeDef *huart, bool *arrive);//put in uart rx IT
void UART_TRANSMIT();
#endif /* INC_COMMUNICATION_H_ */
