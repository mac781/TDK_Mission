/*
 * mainpp.h
 *
 *  Created on: Jul 29, 2024
 *      Author: macub
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#ifdef __cplusplus
extern "C"{
#endif

void main_function();
void setup();
void TEST_MODE();
int divide(int times,int *count);
void wait(int time);//time單位為ms

#ifdef __cplusplus
}
#endif

#endif /* INC_MAINPP_H_ */
