/*
 * Servo_data.cpp
 *
 *  Created on: 2016Äê3ÔÂ23ÈÕ
 *      Author: Thinkpad
 */


#include "Servo_data.h"
#include "string.h"
void CHServo::display_data(){
	printf("******Servo_data display*******\n");
	float tpos = (float)sd.pos / 1000.0;
	float tvel = (float)sd.vel / 1000.0;
	printf("len=%d,status=%d,pos=%f,vel=%f\n",sd.len,sd.status,tpos,tvel);
}
void CHServo::init_data(unsigned char* p1){
	memcpy(&sd,p1,11);
}
