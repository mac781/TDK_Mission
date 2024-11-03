
#include "UART_servo.h"
#include <math.h>
#include "mainpp.h"
//reflect time of servo,轉道指定角度的時間,預設2秒
uint16_t reflectime = 1000;
//extern UART
extern UART_HandleTypeDef huart5;
//servo control variale
int Checksum_Calc, count = 0;
bool claw_state[2] = {false};
int arm_pos[2] = {0};
//declare struct
Arm Arms[2] = {
	{5,6,7,0,0,0,0,true,3,0,0,1780,1780},
	{1,3,4,0,0,0,0,true,3,0,0,1290,1290}
};
//servo control function
void UART_Send(uint8_t u8_data) {
	uint8_t *u8_pointer = &u8_data;
	HAL_UART_Transmit(&huart5, u8_pointer, 1, 100);
	Checksum_Calc += u8_data;
}
void UART_Send_SetMotorPosition(uint16_t motorId, uint16_t Position, uint16_t Time) {
	Checksum_Calc = 0;
	UART_Send(0x80 + motorId);    //header mark & broadcast ID
	UART_Send(0x83);              //header mark & command code
	UART_Send(0x05);              //total data length
	UART_Send((Position / 256) & 0x7F);  //Servo Pos_H
	UART_Send(Position % 256);           //Servo Pos_L
	UART_Send((Time / 256) & 0x7F); //Servo Time_H
	UART_Send(Time % 256);          //Servo Time_L
	UART_Send(Checksum_Calc);     //data length (one servo with time and speed)
}
//declare function
void claw_Delay_Count(Arm *Arms){//put in timer IT
	if(Arms->getBall_delay_count_state == 1){
		Arms->getBall_delay_count++;
		Arms->claw_state = 0;
	}else if(Arms->getBall_delay_count_state == 1&&Arms->getBall_delay_count == 2500){//delay finish,do next mission
		Arms->claw_state = 1;
		Arms->getBall_delay_count = 0;
		Arms->getBall_delay_count_state =0;
	}
	if(Arms->putBall_delay_count_state == 1){
		Arms->putBall_delay_count++;
		Arms->claw_state = 1;
	}else if(Arms->putBall_delay_count_state == 1&&Arms->putBall_delay_count == 2500){//delay finish,do next mission
		Arms->claw_state = 0;
		Arms->putBall_delay_count = 0;
		Arms->putBall_delay_count_state =0;
	}
}
void arm_move(Arm *Arms,int sign){// put in timer IT
	//inverseKinematics(x,y,l1,l2,&pos1,&pos2,elbowUp);
	UART_Send_SetMotorPosition(Arms->joint_1_motorId,(uint16_t)(Arms->joint_1_initAngle+7*sign*Arms->joint_1_Pos),reflectime);
	UART_Send_SetMotorPosition(Arms->joint_2_motorId,(uint16_t)(Arms->joint_2_initAngle+7*sign*Arms->joint_2_Pos),reflectime);
	claw(Arms);
}
void ARM_RUN(){
	arm_move(&Arms[0],-1);
	arm_move(&Arms[1],1);
	pos_control(&Arms[0]);
	pos_control(&Arms[1]);
//	claw(&Arms[0]);
//	claw(&Arms[0]);

}

int getball_pos1 = -60 ,getball_pos2 = 40;//get ball from the ground
int putball_rail_pos1 =60 ,putball_rail_pos2 = 85;//get ball on the ground
int putball_TDKbox_pos1 = 50 ,putball_TDKbox_pos2 = -50;//get ball on the TDKbox
int vertical_pos1 = 75,vertical_pos2 = 5;

void pos_control(Arm *Arms){
	/*select arms,zero for the back arm,1 for the front arms,
	pos control,0 for get ball,1 for put ball on rail,2 for put ball on TDK box
	用的時候改bool claw_state[2],[0]前面,[1]後面*/
	if (Arms -> ARM_POS == 0){
		Arms->joint_1_Pos = getball_pos1;
		Arms->joint_2_Pos = getball_pos2;
	}else if(Arms -> ARM_POS == 1){
		Arms->joint_1_Pos = putball_rail_pos1;
		Arms->joint_2_Pos = putball_rail_pos2;
	}else if(Arms -> ARM_POS == 2){
		Arms->joint_1_Pos = putball_TDKbox_pos1;
		Arms->joint_2_Pos = putball_TDKbox_pos2;
	}else if(Arms -> ARM_POS == 3){
		Arms->joint_1_Pos = vertical_pos1 ;
		Arms->joint_2_Pos = vertical_pos2;
	}
}
uint16_t close_pos = 2200,open_angle =1200;
void claw(Arm *Arms){
	/*select arms,Arms[0] for the front arm,Arms[1] for the front arms,
	夾爪開關,false for open,true for close,
	用的時候改bool claw_state[2],[0]前面,[1]後面*/
	if (Arms->claw_state == 0){
	UART_Send_SetMotorPosition(Arms->claw_motorId,open_angle,500);
	}else{
	UART_Send_SetMotorPosition(Arms->claw_motorId,close_pos,500);
	}
}
//for script
void upadate_pos(Arm *Arms,int Arm_pos){
	/*ARM[0]是前面的手臂,ARM[1]是後面的;
	pos control,0 是取球姿態,1 是放球到軌道,2 是放球到TDK盒子*/
	Arms->ARM_POS = Arm_pos;
	wait(1000);
}
void update_claw(Arm *Arms,bool claw_state){
	/*ARM[0]是前面的手臂,ARM[1]是後面的;
	claw_state 為true時夾爪夾緊,false時夾爪打開*/
	Arms->claw_state = claw_state;
	wait(500);
}
//test
int joint1_test[2]= {0};
int joint2_test[2]= {0};
void arm_test(Arm *Arms,int pos1,int pos2){
	Arms->joint_1_Pos = pos1;
	Arms->joint_2_Pos = pos2;
	arm_move(Arms,reflectime);
}
void ARM_TEST(){
	arm_test(&Arms[0],joint1_test[0],joint2_test[0]);
	arm_test(&Arms[1],joint1_test[1],joint2_test[1]);
	claw(&Arms[0]);
	claw(&Arms[1]);
}


