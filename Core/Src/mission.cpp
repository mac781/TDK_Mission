/*
 * mission.cpp
 *
 *  Created on: 2024年9月24日
 *      Author: mac
 */
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

#define craw_to_middle 13
#define craw_to_front 18.1

extern bool arrive;
extern Arm Arms[2];
extern servo servoFR;
extern servo servoBR;
extern servo servoFM;
extern servo servoBM;
extern DC_motor lifters[2];

extern int target_Location[2];

float statusData[4] = { 0, 0, 0, 0};

//Mode:1:path/ 2:/integral/ 3::stop/ 4:find line/ 5:reset
void GoTo(float Mode, float x, float y, float ang){


	statusData[0] = Mode;
	statusData[1] = x;
	statusData[2] = y;
	statusData[3] = ang;
	UART_TRANSMIT();
	wait(100);
	arrive = 0;
	while(!arrive){}
}
//Mode:6:cascade / state_1 1:down 0:up
//Mode:7:casdcade servo  / state_1 1:open 0:close
void Cascade(float Mode, float state_1){

	statusData[0] = Mode;
	statusData[1] = state_1;
	statusData[2] = 0;
	statusData[3] = 0;

	UART_TRANSMIT();
	wait(100);
	arrive = 0;

	while(!arrive){}
}
void pose_reset(){

	GoTo( 5, 0, 0, 90);
	upadate_pos(&Arms[0], 1);
	upadate_pos(&Arms[1], 1);
	update_claw(&Arms[0], 1);
	update_claw(&Arms[1], 1);
	setTargetLocation(&lifters[0], 1);
	setTargetLocation(&lifters[1], 1);
	update_blockstate(&servoFR, 0);
	update_blockstate(&servoBR, 0);
	update_blockstate(&servoFM, 0);
	update_blockstate(&servoBM, 0);
}
void takeBall(int arm){

	int a = arm;

	//takeball
	upadate_pos(&Arms[a], 0);
	//open claw
	update_claw(&Arms[a], 0);
	//claw down
	setTargetLocation(&lifters[a], 0);
	//close claw
	update_claw(&Arms[a], 1);

	//claw up
	setTargetLocation(&lifters[a], 1);
	//place on robot
	upadate_pos(&Arms[a], 1);
	//open claw
	update_claw(&Arms[a], 0);
}
void placeBall(int arm){

	int a = arm;

	GoTo( 3, 0, 0, 0);

	upadate_pos(&Arms[a], 2);
	update_claw(&Arms[a], 0);
	upadate_pos(&Arms[a], 1);
}

void mission_1_R(){

	//find cross(f)
	GoTo( 4, 3, 1, 0);

	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, 0, 0);

	GoTo( 1, 0, 100.5, 0);

	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 91);

	//R-1
	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, craw_to_middle, 90);

	update_blockstate(&servoBR, 1);
	update_blockstate(&servoBM, 1);
	takeBall(1);
	wait(500);
	update_blockstate(&servoBM, 0);

	//R-2
	GoTo( 2, 0, 100 - craw_to_middle, 90);

	update_blockstate(&servoFR, 1);
	update_blockstate(&servoFM, 1);
	takeBall(0);
	wait(500);
	update_blockstate(&servoFM, 0);

	//R-3
	GoTo( 2, 0, 200 + craw_to_middle, 90);

	//takeball
	upadate_pos(&Arms[1], 0);
	//open claw
	update_claw(&Arms[1], 0);
	//claw down
	setTargetLocation(&lifters[1], 0);
	//close claw
	update_claw(&Arms[1], 1);

	GoTo( 5, 0, 0, 90);

	//claw up
	setTargetLocation(&lifters[1], 1);

	GoTo( 2, 0, 123, 1000);

	//open claw
	update_claw(&Arms[0], 0);
}
void mission_1_L(){

	//find cross(f)
	GoTo( 4, 3, 1, 0);

	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 91);

	GoTo( 1, 0, 100.5, 0);

	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 91);

	GoTo( 5, 0, 0, 90);

	//L-1
	GoTo( 2, 0, craw_to_middle, 90);

	update_blockstate(&servoBR, 1);
	takeBall(1);

	//L-2
	GoTo( 2, 0, -50 - craw_to_middle, 90);

	update_blockstate(&servoFR, 1);
	takeBall(0);

	//L-3
	GoTo( 2, 0, -100 + craw_to_middle, 90);

	update_blockstate(&servoBR, 0);
	takeBall(1);

	//L-4
	GoTo( 2, 0, -150 - craw_to_middle, 90);

	update_blockstate(&servoFR, 0);
	//takeball
	upadate_pos(&Arms[0], 0);
	//open claw
	update_claw(&Arms[0], 0);
	//claw down
	setTargetLocation(&lifters[0], 0);
	//close claw
	update_claw(&Arms[0], 1);
	//claw up
	setTargetLocation(&lifters[0], 1);

	//L-5
	GoTo( 2, 0, -200 + craw_to_middle, 90);

	//takeball
	upadate_pos(&Arms[1], 0);
	//open claw
	update_claw(&Arms[1], 0);
	//claw down
	setTargetLocation(&lifters[1], 0);
	//close claw
	update_claw(&Arms[1], 1);
	//claw up
	setTargetLocation(&lifters[1], 1);

	//middle find line(b)
	GoTo( 4, 2, 0, 0);

	//front spin find line(f)
	GoTo( 4, 7, 1, 0);

	//left find line(f)
	GoTo( 4, 6, 1, 0);

	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 92);

	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, 10, 90);

	//middle find line(f)
	GoTo( 4, 2, 1, 0);

	//front spin find line(t)
	GoTo( 4, 7, 1, 0);
}
void mission_2(){

	upadate_pos(&Arms[0], 3);
	upadate_pos(&Arms[1], 3);
	update_claw(&Arms[0], 1);
	update_claw(&Arms[0], 1);
/*
	//middle find red
	GoTo( 4, 8, 1, 0);

	GoTo( 1, 0, 43, 0);
*/
	GoTo( 1, 0, 180, 0);

	//reset
	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 90);

	GoTo( 2, 0, 40, 90);

	//cascade舉起
	Cascade( 6, 0);

	Cascade( 7, 0);
	Cascade( 7, 1);
	wait(200);
	Cascade( 7, 0);
	//cascade下降
	Cascade( 6, 1);

	update_blockstate(&servoFM, 1);
	update_blockstate(&servoBM, 1);

	GoTo( 2, 0, -40, 90);

	//cascade舉起
	Cascade( 6, 0);

	Cascade( 7, 0);
	Cascade( 7, 1);
	wait(200);
	Cascade( 7, 0);
	//cascade下降
	Cascade( 6, 1);

	//middle find line(f)
	GoTo( 4, 2, 1, 0);

	//front spin find line(c)
	GoTo( 4, 7, 0, 0);

	GoTo( 1, 0, 238, 0);

	//reset
	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, 0, 170);

	GoTo( 4, 4, 1, 0);
}
void mission_3(){

	GoTo( 1, 0, 180, 0);
	GoTo( 1, 0, 257, 0);
	GoTo( 1, 0, 147, 0);
	GoTo( 1, 0, 260, 0);

	upadate_pos(&Arms[0], 2);
	upadate_pos(&Arms[1], 2);

	GoTo( 1, 0, 135, 0);

	//reset
	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 92);

	//reset
	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, 70, 90);

	//reset
	GoTo( 5, 0, 0, 0);

	GoTo( 2, 0, 0, 90);

	//爪子上升到最高
	setTargetLocation(&lifters[0], 1);
	setTargetLocation(&lifters[1], 1);

	//reset
	GoTo( 5, 0, 0, 90);

	GoTo( 2, 0, 61.5 + craw_to_middle, 90);

	placeBall(1);

	GoTo( 2, 0, 208.5 - craw_to_middle, 90);

	placeBall(0);

	GoTo( 2, 0, 355.5 + craw_to_middle, 90);

	update_claw(&Arms[1], 0);
	upadate_pos(&Arms[1], 1);
	update_claw(&Arms[1], 1);
	upadate_pos(&Arms[1], 2);
	update_claw(&Arms[1], 0);

	GoTo( 3, 0, 0, 0);
}
