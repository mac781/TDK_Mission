/*
 * mission.h
 *
 *  Created on: 2024年9月24日
 *      Author: mac
 */

#ifndef INC_MISSION_H_
#define INC_MISSION_H_

void GoTo(float Mode, float x, float y, float ang);
void Cascade(float Mode, float state_1);
void takeBall(int arm);
void mission_1_L();
void mission_1_R();
void mission_2();
void mission_3();
void cascade_goback();

#endif /* INC_MISSION_H_ */
