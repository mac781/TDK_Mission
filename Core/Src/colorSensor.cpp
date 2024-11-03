///*
// * colorSensor.cpp
// *
// *  Created on: 2024年9月19日
// *      Author: mac
// */
//#include "colorSensor.h"
//#include "stm32f4xx_hal.h"
//#include "stdlib.h"
//
//extern TIM_HandleTypeDef htim7;
///*
// * S0		PC7
// * S1		PC4
// * S2		PC6
// * S3		PC8
// * OUTPUT	PC9
// */
//int   count_f = 0;  // 頻率計算
//int   array[3];     // 儲存 RGB 值
//int   flag = 0;     // RGB 過濾順序
//int   color[3];		// 儲存白平衡後 RGB 值
//// RGB 補償係數
////float SF[3] = {0.130102038, 0.139802635, 0.11681173};
//float SF[3] = {0.534591198, 0.57432431, 0.482041597};
//// 選擇過濾顏色
//void TSC_FilterColor(bool Level1, bool Level2){
//
//	count_f = 0;
//	flag++;
//
//    if(Level1)
//	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//    else
//  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//
//    if(Level2)
//  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//    else
//  	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
//}
//
//void TSC_Callback(){
//
//    switch(flag){
//    //red
//    case 0:
//    	TSC_FilterColor(0, 0);
//	    break;
//    //green
//    case 1:
//        array[0] = count_f;
//        TSC_FilterColor(1, 1);
//        break;
//    //blue
//    case 2:
//        array[1] = count_f;
//        TSC_FilterColor(0, 1);
//        break;
//    //Clear(no filter)
//    case 3:
//        array[2] = count_f;
//        TSC_FilterColor(1, 0);
//        break;
//     default:
//        count_f = 0;
//        break;
//  }
//}
//
//void colorSensor_setup(){
//
//	HAL_TIM_Base_Start_IT(&htim7);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//
//    HAL_Delay(1000);
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//
//	if(GPIO_Pin == GPIO_PIN_15){
//
//		count_f++;
//	}
//}
//
//int colorSensor(){
//
//    flag = 0;
//
//    HAL_Delay(1000);
//
//    // 儲存補償計算後之 RGB
//    for(int i = 0; i < 3; i++)
//  	    color[i] = (int)(array[i] * SF[i]);
//
//    //yellow
//    if(color[0] - color[1] >= 20 && color[1] - color[2] >= 20)
//    	return 1;
//    //red
//    else if(color[0] - color[1] >= 25 && color[0] - color[2] >= 25)
//        return 2;
//    //blue
//    else if	(std::abs(color[0] - color[1]) <=15 && std::abs(color[0] - color[2]) <=15 && std::abs(color[2] - color[1]) <=15)
//        return 3;
//    //pink
//    else
//    	return 4;
//
//}

