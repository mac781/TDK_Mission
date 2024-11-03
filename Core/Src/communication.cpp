/*
 * communication.cpp
 *
 *  Created on: Aug 8, 2024
 *      Author: macub
 */
#include "communication.h"
#include "stdlib.h"
#include "string.h"

uint8_t buffer_TX[4 * sizeof(float)];
uint8_t buffer_RX[sizeof(bool)];
//uint8_t buffer_RX[2 * sizeof(float)];
bool arrive = 0;

int rx,tx;
extern float statusData[4];
extern UART_HandleTypeDef huart3;

void UART_setup(){
    HAL_UART_Receive_DMA(&huart3, buffer_RX, sizeof(buffer_RX));
}
//tx
void UART_TransmitData_Status_IT(UART_HandleTypeDef *huart, float data1, float data2, float data3,float data4) {
    memcpy(buffer_TX, &data1, sizeof(float));
    memcpy(buffer_TX + sizeof(float), &data2, sizeof(float));
    memcpy(buffer_TX + 2 * sizeof(float), &data3, sizeof(float));
    memcpy(buffer_TX + 3 * sizeof(float), &data4, sizeof(float));
    HAL_UART_Transmit_DMA(huart, buffer_TX, sizeof(buffer_TX));
}
//rx
void UART_ReceiveData_arrriveDestination_IT(UART_HandleTypeDef *huart, bool *arrive){//put in uart rx IT
	memcpy(arrive, buffer_RX, sizeof(bool));
    HAL_UART_Receive_DMA(huart, buffer_RX, sizeof(buffer_RX));
}
//傳訊時 call 這個function
void UART_TRANSMIT(){
	 UART_TransmitData_Status_IT(&huart3, statusData[0], statusData[1], statusData[2], statusData[3]);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart -> Instance == USART3){
		tx++;
		if (statusData[0] == 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}else if (statusData[0] > 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart){

	if(huart -> Instance == USART3){
		rx++;
		UART_ReceiveData_arrriveDestination_IT(&huart3, &arrive);

//		if (arrive == true){
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		}else {
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//		}

	}
}
