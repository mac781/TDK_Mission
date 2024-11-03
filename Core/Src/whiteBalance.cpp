/*
 * whiteBalance.cpp
 *
 *  Created on: 2024年9月19日
 *      Author: mac
 */
#include "colorSensor.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim7;
/*
 * S0		PC7
 * S1		PC4
 * S2		PC6
 * S3		PC8
 * OUTPUT	PC9
 */
int   w_count_f = 0;  // 頻率計算
int   w_array[3];     // 儲存 RGB 值
int   w_flag = 0;     // RGB 過濾順序
float w_SF[3];        // 儲存白平衡計算後之 RGB 補償係數
int   w_color[3];	  // 儲存白平衡後 RGB 值
// 選擇過濾顏色
void W_TSC_FilterColor(bool Level1, bool Level2){

	if(Level1)
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    else
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    if(Level2)
  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    else
  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
//White Balance
void W_TSC_WB(bool Level0, bool Level1){

	w_count_f = 0;
	w_flag++;
	W_TSC_FilterColor(Level0, Level1);
}
void W_TSC_Callback(){

    switch(w_flag){
    //red
    case 0:
    	W_TSC_WB(0, 0);
	    break;
    //green
    case 1:
    	w_array[0] = w_count_f;
        W_TSC_WB(1, 1);
        break;
    //blue
    case 2:
    	w_array[1] = w_count_f;
        W_TSC_WB(0, 1);
        break;
    //Clear(no filter)
    case 3:
    	w_array[2] = w_count_f;
        W_TSC_WB(1, 0);
        break;
     default:
    	 w_count_f = 0;
        break;
  }
}

void W_colorSensor_setup(){

	HAL_TIM_Base_Start_IT(&htim7);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    HAL_Delay(1000);

    for(int i = 0; i < 3; i++)
    	w_SF[i] = 255.0/ w_array[i];
}
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_15){

		w_count_f++;
	}
}
*/
void W_colorSensor(){

	w_flag = 0;

    HAL_Delay(1000);

    // 儲存補償計算後之 RGB
    for(int i = 0; i < 3; i++)
  	    w_color[i] = (int)(w_array[i] * w_SF[i]);
}



