
#pragma once

#include "stm32f4xx_hal.h"

typedef struct{
	uint16_t joint_1_motorId;
	uint16_t joint_2_motorId;
	uint16_t claw_motorId;
	int getBall_delay_count;
	int getBall_delay_count_state;
	int putBall_delay_count;
	int putBall_delay_count_state;
	bool claw_state;
    int ARM_POS;//if ARM_POS　== true , put ball ,else if ARM_POS == false, get ball
    int joint_1_Pos;
    int joint_2_Pos;
    int joint_1_initAngle;
    int joint_2_initAngle;
}Arm;
void UART_Send(uint8_t u8_data);
void UART_Send_SetMotorPosition(uint16_t motorId, uint16_t Position,uint16_t Time);
void claw(Arm *Arms);
void claw_Delay(Arm *Arms);
void arm_ReachDestination(uint16_t GPIO_Pin);//1,getball,0,putball
void arm_script();
void arm_move(Arm *Arms,int sign);
void ARM_RUN();
void CLAW_DELAY_COUNT();
void arm_test(Arm *Arms,int pos1,int pos2);
void ARM_TEST();
void pos_control(Arm *Arms);
void update_claw(Arm *Arms,bool claw_state);
void upadate_pos(Arm *Arms,int Arm_pos);
