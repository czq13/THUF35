/*
 * Servo_data.cpp
 *
 *  Created on: 2016Äê3ÔÂ23ÈÕ
 *      Author: Thinkpad
 */


#include "AP_CHuart.h"

#include "stdio.h"

#include "string.h"
#include <AP_HAL/AP_HAL.h>


void AP_CHuart::display_data(uint8_t t){
	printf("******Servo_data display*******\n");
	float tpos = (float)(sd[t]).pos / 1000.0;
	float tvel = (float)sd[t].vel / 1000.0;
	printf("len=%d,status=%d,pos=%f,vel=%f\n",sd[t].len,sd[t].status,tpos,tvel);
}
void AP_CHuart::update_data(unsigned char* p1,uint8_t t){
	memcpy(&(sd[t]),p1,11);
}
void AP_CHuart::send_token(){
	token.num = servo_ToSend + 1;
	servo_ToSend = (servo_ToSend + 1) % servoN;
	tmpUartD->write((const uint8_t*)&token,3);
}
uint8_t AP_CHuart::readUart(){
	uint8_t count = tmpUartD->ch_read(buf,11);
	if (count > 0) {
		uint8_t tmp = buf[3];
		update_data(buf,tmp - 1);
		return 1;
	}
	return -1;
}
AP_CHuart chuart;
