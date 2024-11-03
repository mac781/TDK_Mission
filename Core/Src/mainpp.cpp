/*
 * mainpp.cpp
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */
#include "mainpp.h"
#include "communication.h"
#include "servo.h"
#include "motor.h"
#include "colorSensor.h"
#include "whiteBalance.h"
#include "mission.h"
#include "UART_servo.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;

//test
extern int testAngle[4] ;
extern int joint1_test[2];
extern int joint2_test[2];
extern float sp_test[2];
extern bool arrive;
extern Arm Arms[2];
extern servo servoFR;
extern servo servoBR;
extern servo servoFM;
extern servo servoBM;
extern DC_motor lifters[2];

int delay_count = 0,divide_10_count = 0,divide_100_count;
int servoState[4] = {0};
int timer,k;
int e1,e2,e3,e4;
int test = 0;
int script = 0;
int script_angle[4];
int tim12 = 0;

void setup(){
	DCmotor_setup();
	servo_setup();
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_Base_Start_IT(&htim13);
	UART_setup();
}

int divide(int times,int *count){

	if (*count > times){
		*count = 0;
		return 1;

	}else{
		return 0;
	}
}

void wait(int time){//time單位為ms

	HAL_TIM_Base_Start_IT(&htim13);

	while(delay_count < time){
	}

	HAL_TIM_Base_Stop_IT(&htim13);

	delay_count = 0;
}
void main_function(){
	while(1){
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET){
		test = 1;
	}
	test = 0;
	Cascade( 6, 2);

	switch (script) {
		case(0)://一開始走右邊

			mission_1_R();
			break;
		case(1)://一開始走左邊

			mission_1_L();
			mission_2();
			mission_3();
			break;
		case(2)://夾玩球沒過重製，從起點直接往前走

			GoTo( 1, 0, 220, 0);

			//middle find line(f)
			GoTo( 4, 2, 1, 0);

			//front spin find line(t)
			GoTo( 4, 5, 1, 0);

			//find right line
			GoTo( 4, 5, 1, 0);

			GoTo( 5, 0, 0, 90);

			GoTo( 2, 0, 0, 0);

			GoTo( 1, 0, 290, 0);

			//middle find line(f)
			GoTo( 4, 2, 1, 0);

			//front spin find line(t)
			GoTo( 4, 8, 1, 0);

			//reset
			GoTo( 5, 0, 147.5, 90);

			mission_2();
			mission_3();
			break;

		case(3)://夾完球過重製，從第三關前重置點開始

			//find left line(f)
			GoTo( 4, 6, 1, 0);

			mission_2();
			mission_3();
			break;

		case(4)://第五關S型前重置開始
			mission_3();
			break;
		}
		//break;
	}
}

void TEST_MODE(){
	SERVO_TEST();
	DCMOTOR_TEST();
	if(divide(100,&divide_100_count)){
		ARM_RUN();
		}
	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM7){
		divide_10_count++;
		divide_100_count++;
		if(test == true){
			TEST_MODE();
		}else if(test != true){


			timer++;

			blockState();//
			DCmotor_run();
			//ARM_RUN();
			if(divide(10,&divide_10_count)){

			}
			if(divide(100,&divide_100_count)){
				UART_TRANSMIT();
				ARM_RUN();

			}
		}
	}

	if (htim -> Instance == TIM13){
		delay_count++;
	}
}

void reset(uint16_t GPIO_Pin){
	switch (GPIO_Pin) {
	case GPIO_PIN_13:
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
    	script = 1;
    	Arms[0].ARM_POS = 3;
    	Arms[1].ARM_POS = 3;
    	Arms[0].claw_state = true;
    	Arms[1].claw_state = true;;
    	lifters[0].goalLocation = 1;
    	lifters[1].goalLocation = 1;
    	servoFR.block_state = 0;
    	servoBR.block_state = 0;
    	servoFM.block_state = 0;
    	servoBM.block_state = 0;
	     }
    break;

    case GPIO_PIN_2:
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
    	script = 2;
    	Arms[0].ARM_POS = 3;
    	Arms[1].ARM_POS = 3;
    	Arms[0].claw_state = true;
    	Arms[1].claw_state = true;;
    	lifters[0].goalLocation = 1;
    	lifters[1].goalLocation = 1;
    	servoFR.block_state = 0;
    	servoBR.block_state = 0;
    	servoFM.block_state = 0;
    	servoBM.block_state = 0;
    		}
    break;

    case GPIO_PIN_3:
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
    	script = 3;
    	Arms[0].ARM_POS = 3;
    	Arms[1].ARM_POS = 3;
    	Arms[0].claw_state = true;
    	Arms[1].claw_state = true;;
    	lifters[0].goalLocation = 1;
    	lifters[1].goalLocation = 1;
    	servoFR.block_state = 0;
    	servoBR.block_state = 0;
    	servoFM.block_state = 0;
    	servoBM.block_state = 0;
         }
    break;
    case GPIO_PIN_8:
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
    	script = 4;
    	Arms[0].ARM_POS = 2;
    	Arms[1].ARM_POS = 2;
    	Arms[0].claw_state = true;
    	Arms[1].claw_state = true;;
    	lifters[0].goalLocation = 1;
    	lifters[1].goalLocation = 1;
    	servoFR.block_state = 0;
    	servoBR.block_state = 0;
    	servoFM.block_state = 0;
    	servoBM.block_state = 0;
    	 }
    break;
    default:
    	//script = 0;
    break;
	}
	}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	updateLocation(GPIO_Pin);
    reset(GPIO_Pin);
}

