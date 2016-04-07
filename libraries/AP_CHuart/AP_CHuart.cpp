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
extern const AP_HAL::HAL& hal;


void AP_CHuart::display_data(uint8_t t){
	//printf("******Servo_data display*******\n");
	double tpos = (double)(sd[t].pos) ;/// (double)(1000.0);
	tpos = tpos / 1000.0;
	double tvel = (double)(sd[t].vel) ;/// (double)(1000.0);
	tvel = tvel / 1000.0;
	printf("input=%x,len=%x,status=%x,pos=%f,vel=%f\n",sd[t].input,sd[t].pos,sd[t].vel,tpos,tvel);
}
void AP_CHuart::update_data(unsigned char* p1,uint8_t t){
	memcpy(&(sd[t]),p1,11);
	//if debug
	for (int i = 0;i < 11;i++)
		printf("%x ",p1[i]);
	printf("\n");
	display_data(num);
}
void AP_CHuart::send_token(){
	token.num = servo_ToSend + 1;
	token.parse = 0x00;
	servo_ToSend = (servo_ToSend + 1) % servoN;
	tmpUartD->write((const uint8_t*)&token,sizeof(token));
}
uint8_t AP_CHuart::readUart(){
	uint8_t count = tmpUartD->ch_read(buf,11);
	if (count > 0) {
		uint8_t tmp = buf[3];
		num = tmp - 1;
		if (num > 1)
			return -1;
		update_data(buf,num);
		return 1;
	}
	return -1;
}
AP_CHuart::AP_CHuart():servoN(2),servo_ToSend(0){
		tmpUartD = (PX4::PX4UARTDriver*)hal.uartD;
		token.head1 = 0x055;
		token.head2 = 0x0AA;
}
AP_CHuart chuart;
