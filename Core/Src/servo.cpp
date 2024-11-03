/*
 * servo.cpp
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */
#include "servo.h"
extern TIM_HandleTypeDef htim1;

int goalAngle,t;
int responseTime = 250;
int servoAngle[3] = {50,160,0};
int testAngle[4] = {0};
//F front, B back, R right, M middle, L left
servo servoFR = {0, 0, 0, 2000, true, false, &htim1, TIM_CHANNEL_4};
servo servoBR = {0, 0, 0, 2000, true, false, &htim1, TIM_CHANNEL_2};
servo servoFM = {0, 0, 0, 2000, true, false, &htim1, TIM_CHANNEL_3};
servo servoBM = {0, 0, 0, 2000, true, false, &htim1, TIM_CHANNEL_1};

//servo timer setup,put in setup
void servo_setup(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
//servo control function
void servo_move(servo*servo,float goalAngle){
	//response time 為轉到指定角度時間,預設1秒
	servo -> goalAngle = goalAngle;
	servo -> responseTime = responseTime;
	servo -> move = true;
}
void servo_run(servo*servo ,int updateFreq){/*updateFreq = timer IT frequency (Hz)
	put in timer IT*/
	if (servo -> move == true){
		if ((int)servo -> pos == (int)servo -> goalAngle){
    	servo -> move = false;
    	servo -> lastAngle = servo -> goalAngle;
        }else{
         float distance = servo -> goalAngle - servo -> lastAngle;
         servo -> pos += distance/(servo -> responseTime * updateFreq / 1000);
         __HAL_TIM_SET_COMPARE(servo -> htim, servo -> TIM_CHANNEL,600+10*(int)servo -> pos);
         t++;
        }
	}
}
//put in timer IT
void blockState(){
	/*bloack_state_FR右前,bloack_state_BR右後,bloack_state_FM中前,bloack_state_BM中後
	true open ,false close
	用的時候extern直接改block_state[4]*/
	servo_run(&servoFR, 1000);
	servo_run(&servoBR, 1000);
	servo_run(&servoFM, 1000);
	servo_run(&servoBM, 1000);
	if (servoFR.block_state == true){
		servo_move(&servoFR, 170);
	}else{
		servo_move(&servoFR, 40);
	}
	if (servoBR.block_state == true){
		servo_move(&servoBR, 40);
	}else{
		servo_move(&servoBR, 170);
		}
	if (servoFM.block_state == true){
		servo_move(&servoFM, servoAngle[0]);
	}else{
		servo_move(&servoFM, servoAngle[1]);
	}
	if (servoBM.block_state == true){
		servo_move(&servoBM, servoAngle[1]);
	}else{
		servo_move(&servoBM, servoAngle[0]);
		}
}
//for script
void update_blockstate(servo *block,bool block_state){
	//先選哪一顆servo,然後block_state為true時擋板打開,false擋板關閉
	block->block_state = block_state;
	//wait(0);
}
//for test
void SERVO_TEST(){
	servo_run(&servoFR, 1000);
	servo_run(&servoBR, 1000);
	servo_run(&servoFM, 1000);
	servo_run(&servoBM, 1000);
	servo_move(&servoFR, testAngle[0]);
	servo_move(&servoBR, testAngle[1]);
	servo_move(&servoFM, testAngle[2]);
	servo_move(&servoBM, testAngle[3]);
}
