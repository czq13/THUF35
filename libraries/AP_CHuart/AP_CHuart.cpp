/*
 * Servo_data.cpp
 *
 *  Created on: 2016��3��23��
 *      Author: THU czq
 *      ���������ļ�ʵ����AP_CHuart�����еĺ���
 *      ��ʷ��
 *      	THU czq 2016/3/23  ����
 */


#include "AP_CHuart.h"

#include "stdio.h"

#include "string.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

/*****************************************
 * function : Servo_data
 * ���ߣ�THU czq
 * ���������ڵ��ԣ���������Ϣ
 * ���ڣ�2016/5/5
 * ���룺���Զ����ţ��ܹ����������
 * �������
 ***************************************** */
void AP_CHuart::display_data(uint8_t t){
	//printf("******Servo_data display*******\n");
	double tpos = (double)(sd[t].pos) ;/// (double)(1000.0);
	tpos = tpos / 1000.0;
	double tvel = (double)(sd[t].vel) ;/// (double)(1000.0);
	tvel = tvel / 1000.0;
	printf("input=%x,len=%x,status=%x,pos=%f,vel=%f\n",sd[t].input,sd[t].pos,sd[t].vel,tpos,tvel);
}
/*****************************************
 * function : Servo_data
 * ���ߣ�THU czq
 * �������ڽ��������ݺ󣬽����ܵ����ݸ��µ���ؽṹ�С�
 * ���ڣ�2016/5/5
 * ���룺���յ��������Լ������ţ��ܹ����������
 * �������
 ***************************************** */
void AP_CHuart::update_data(unsigned char* p1,uint8_t t){
	memcpy(&(sd[t]),p1,11);
	//if debug
	for (int i = 0;i < 11;i++)
		printf("%x ",p1[i]);
	printf("\n");
	display_data(num);
}
/*****************************************
 * function : send_token
 * ���ߣ�THU czq
 * ���������ڷ������ƻ�
 * ���ڣ�2016/5/5
 * ���룺��
 * �������
 ***************************************** */
void AP_CHuart::send_token(){
	token.num = servo_ToSend + 1;
	token.parse = 0x00;
	servo_ToSend = (servo_ToSend + 1) % servoN;
	tmpUartD->write((const uint8_t*)&token,sizeof(token));
}
/*****************************************
 * function : readUart
 * ���ߣ�THU czq
 * ���������ڶ�ȡ�������ݣ������µ�������
 * ���ڣ�2016/5/5
 * ���룺��
 * ������ɹ����
 ***************************************** */
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
