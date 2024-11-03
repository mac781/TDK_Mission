/*
 * motor.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "mainpp.h"

typedef struct {
    float kp;
    float ki;
    float integral;
    float span;
    float setpoint;
    int arr;
    GPIO_TypeDef *gpioPort;
    uint16_t gpioPin;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;
    int pul;
} PID_controller;

typedef struct {
	PID_controller *PID_Controllers;
	TIM_HandleTypeDef *htim;//encoder_timer
	float speed;
	int currentLocation;//get ball location = 0,put ball location = 1
	int goalLocation;
	float reduction_ratio;

} DC_motor;

void DCmotor_setup();
void getState(DC_motor *lifter,int sign);
void PI_control_run(DC_motor *motor,float sp) ;
void DCmotor_run();
void gotoTargetLocation(DC_motor *lifter,float *velocity_sp);
void encodersp(float *encsp);
void speedOutput(int m);
void updateLocation(uint16_t GPIO_Pin);
void DCMOTOR_TEST();
void setTargetLocation(DC_motor *lifter,int targetLocation);

#endif /* INC_MOTOR_H_ */
