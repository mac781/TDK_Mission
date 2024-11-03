/*
 * motor.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "motor.h"
#include "string.h"
#include "math.h"
#include "cmath"
//extern timer
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
//PID
const int resolution = 512;
const int reduction_ratio = 20.8;
float ki = 10;
float kp = 1;
int arr = 4199;
float span = 0.001;
int target_Location[2] = {0};
//lifter speed
float sp[2] = {0};
float lifter_speed = 7;
//float p = 3.5, i= 10.1,turns;
//test
float sp_test[2] = {0};
//declare struct
PID_controller PID_controllers[2] = {
    {kp, ki, 0, span, 0, arr,
    		GPIOB, GPIO_PIN_12, &htim12, TIM_CHANNEL_1,0},  // cascade_motor_a
    {kp, ki, 0, span, 0, arr,
		    GPIOB, GPIO_PIN_13, &htim12, TIM_CHANNEL_2,0}// cascade_motor_b
};
DC_motor lifters[2] = {
	{&PID_controllers[0],&htim3,0,1,1,reduction_ratio},
	{&PID_controllers[1],&htim5,0,1,1,reduction_ratio}
};
//declarefunction

//DC motor setup,put in setup
void DCmotor_setup(){
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);//motor[2]
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);//motor[1]
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}
//PID
void PI_control_run(DC_motor *motor,float sp) {
    float error, u_a = 0;
    motor->PID_Controllers->pul = 0;
    float bound = 1 / motor->PID_Controllers->ki;
    motor->PID_Controllers->setpoint = sp;
    error = motor->PID_Controllers->setpoint - motor->speed;
    motor->PID_Controllers->integral += error * motor->PID_Controllers->span;
    if (motor->PID_Controllers->integral
    		> bound) motor->PID_Controllers->integral = bound;
    else if (motor->PID_Controllers->integral
    		< -bound) motor->PID_Controllers->integral = -bound;
    u_a = motor->PID_Controllers->kp * error
    		+ motor->PID_Controllers->ki * motor->PID_Controllers->integral;
    if (u_a > 1) u_a = 1;
    else if (u_a < -1) u_a = -1;

    if (u_a > 0) {
    	motor->PID_Controllers->pul = (int)(u_a * motor->PID_Controllers->arr);
        HAL_GPIO_WritePin(motor->PID_Controllers->gpioPort, motor->PID_Controllers->gpioPin, GPIO_PIN_SET);
    } else if (u_a < 0) {
    	motor->PID_Controllers->pul = (int)(-u_a * motor->PID_Controllers->arr);
        HAL_GPIO_WritePin(motor->PID_Controllers->gpioPort, motor->PID_Controllers->gpioPin, GPIO_PIN_RESET);
    } else {
    	motor->PID_Controllers->pul = 0;
    }
    __HAL_TIM_SET_COMPARE(motor->PID_Controllers->htim, motor->PID_Controllers->TIM_CHANNEL, motor->PID_Controllers->pul);
}
//get speed of encoder
void getState(DC_motor *motor,int sign){
	int16_t enc ;
	enc = __HAL_TIM_GetCounter(motor->htim);
	motor->speed = sign*(float)enc /(4*resolution*motor->reduction_ratio*span);
	__HAL_TIM_SetCounter(motor->htim,0);
	//motor->currentLocation += motor->speed*span*sign;
}
//put in EXTI
void updateLocation(uint16_t GPIO_Pin){//get ball location = 0,put ball location = 1
	  switch (GPIO_Pin) {
	  case GPIO_PIN_4:// get ball
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_SET){
		  lifters[0].currentLocation = 0;
		  if(lifters[0].goalLocation == 0){
		  sp[0] = 0;
		  sp_test[0] = 0;
		  }
		  }
		  break;
	  case GPIO_PIN_5://put ball
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_SET){
	      lifters[0].currentLocation = 1;
	      if(lifters[0].goalLocation == 1){
	      sp[0] = 0;
	      sp_test[0] = 0;
	      }
		  }
	  	  break;
	  case GPIO_PIN_0://put ball
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_SET){
	 	  lifters[1].currentLocation = 1;
	 	  if(lifters[1].goalLocation == 1){
	 	  sp[1] = 0;
	 	  sp_test[1] = 0;
	 	  }
	 	  }
	  	  break;
	  case GPIO_PIN_1://get ball
		  if (HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_SET){
	  	  lifters[1].currentLocation = 0;
	      if(lifters[1].goalLocation == 0){
	   	  sp[1] = 0;
	      sp_test[1] = 0;
	      }
		  }
	  	  break;
	  default:
		  break;
	  }
}
bool reach_destination(DC_motor *lifter){
	if (lifter->currentLocation == lifter->goalLocation )
		return 1;
	else{
		return 0;
	}
}
//run all the DC motro function,put in timer IT
void DCmotor_run(){
	//get speed from encoder, and calculate height
	getState(&lifters[0],-1);//
	getState(&lifters[1],1);//
	//set goal height of lifter
	gotoTargetLocation(&lifters[0],&sp[0]);
	gotoTargetLocation(&lifters[1],&sp[1]);
    // PI control DCmotor with velocity set point
	PI_control_run(&lifters[0],-sp[0]);
	PI_control_run(&lifters[1],sp[1]);
	}
int y[3] = {0,0,0};
void gotoTargetLocation(DC_motor *lifter,float *velocity_sp){
	/*selct lifter,lifters[0] for the back arm,lifters[1] for the front arms,
	  give speed,sp[0] for the back arm,sp[1] for the back arm,
	  set targetLocation,1 for go up,0 for go down
	  基本上用的時候extern int target_Location[2] 就好,
	  target_Location[0]是前面手臂的,target_Location[1]是後面手臂的
	  要調速度的話改lifter_speed,預設是1RPS,然後目前兩隻lifter速度變數用都是同個 */
	if (lifter->currentLocation != lifter->goalLocation){
		if(lifter->currentLocation == 0){//get ball
			*velocity_sp = -lifter_speed;
			y[0]++;
		}else if(lifter->currentLocation == 1){//put ball
			*velocity_sp = lifter_speed;
			y[1]++;
		}
	}else{
		y[2]++;
		*velocity_sp = 0;
	}
}
//for script
void setTargetLocation(DC_motor *lifter,int targetLocation){
	//lifter[0]是前面的[1]是後面的,targetLocation 1是到上面放球0是到下面取球
	lifter->goalLocation = targetLocation;
	while(!reach_destination(lifter)){
	}

}
//for test
void DCMOTOR_TEST(){
	getState(&lifters[0],-1);//
    getState(&lifters[1],1);//
	PI_control_run(&lifters[0],-sp[0]);
	PI_control_run(&lifters[1],sp[1]);
	gotoTargetLocation(&lifters[0],&sp[0]);
	gotoTargetLocation(&lifters[1],&sp[1]);
}
