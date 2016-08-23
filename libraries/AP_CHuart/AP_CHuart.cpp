/*
 * Servo_data.cpp
 *
 *  Created on: 2016年3月23日
 *      Author: THU czq
 *      描述：该文件实现了AP_CHuart这类中的函数
 *      历史：
 *      	THU czq 2016/3/23  创建
 */


#include "AP_CHuart.h"

#include "stdio.h"

#include "string.h"
#include "math.h"
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

/*****************************************
 * function : Servo_data
 * 作者：THU czq
 * 描述：用于调试，输出相关信息
 * 日期：2016/5/5
 * 输入：调试舵机序号（总共两个舵机）
 * 输出：无
 ***************************************** */
void AP_CHuart::display_data(uint8_t t){
	//printf("******Servo_data display*******\n");
	double tpos = (double)(sd[t].pos) ;/// (double)(1000.0);
	tpos = tpos / 1000.0;
	//double tvel = (double)(sd[t].vel) ;/// (double)(1000.0);
	//tvel = tvel / 1000.0;
	//printf("input=%x,len=%x,status=%x,pos=%f,vel=%f\n",sd[t].input,sd[t].pos,sd[t].vel,tpos,tvel);
}
/*****************************************
 * function : Servo_data
 * 作者：THU czq
 * 描述：在接受完数据后，将接受的数据更新到相关结构中。
 * 日期：2016/5/5
 * 输入：接收到的数据以及舵机序号（总共两个舵机）
 * 输出：无
 ***************************************** */
void AP_CHuart::update_data(unsigned char* p1,uint8_t t){
	memcpy(&(sd[t]),p1,sizeof(Servo_data));
	//printf("receive:%d\n",sd[t].pos);
	//if debug
	//for (unsigned int i = 0;i < sizeof(Servo_data);i++)
	//	printf("%x ",p1[i]);
	//printf("\n");
	//display_data(num);
}
/*****************************************
 * function : send_token
 * 作者：THU czq
 * 描述：用于发送令牌环
 * 日期：2016/5/5
 * 输入：无
 * 输出：无
 ***************************************** */
void AP_CHuart::send_token(){
	token.servo_token.num = servo_ToSend + 1;

	servo_ToSend = (servo_ToSend + 1) % servoN;
	if (token.servo_token.num == 1) {
		token.servo_token.timer = ctimer1++;
		token.servo_token.input = (int16_t) (floor(sInput1 / 0.15));
	}
	else {
		token.servo_token.timer = ctimer2++;
		token.servo_token.input = (int16_t) (floor(sInput2 / 0.15));
	}
	token.servo_token.checkSum = 0;
	for (int8_t i = 2;i < 7;i++)
		token.servo_token.checkSum += token.data[i];
	tmpUartD->write((const uint8_t*)&token,sizeof(token));
}
/*****************************************
 * function : readUart
 * 作者：THU czq
 * 描述：用于读取串口数据，并更新到内润中
 * 日期：2016/5/5
 * 输入：无
 * 输出：成功与否
 * 修改：加入了chesum校验，为了配合收发多帧，改进了收取方式
 ***************************************** */
int8_t AP_CHuart::readUart(){
	int16_t count = tmpUartD->ch_read(buf,8);
	if (count > 0) {
		for (uint16_t i = 0;i < count;i++) {
			if ((uint8_t)buf[i] == 0x55 && (uint8_t)buf[i+1] == 0xAA) {
				//checksum
				uint8_t csum = 0;
				for (int8_t j = 2;j < 7;j++)
					csum += (uint8_t) buf[i+j];
				if (csum != (uint8_t) buf[i+7]) {
					printf("failed,count=%d\n",count);
					continue;
				}
				uint8_t tmp = buf[i+6];
				num = tmp -1;
				if (num > 1) {
					printf("failed @!\n");
					continue;
				}
				update_data(buf+i,num);
			}
		}
		return 1;
	}
	return -1;
}
void AP_CHuart::setServoCtrl(float c1,float c2) {
	sInput1 = c1;
	sInput2 = c2;
	//int16_t t1 = (int16_t) (floor(sInput1 / 0.15));
	//int16_t t2 = (int16_t) (floor(sInput2 / 0.15));
	//printf("t1=%d,t2=%d\n",t1,t2);

}
AP_CHuart::AP_CHuart():servoN(2),servo_ToSend(0),ctimer1(0),ctimer2(0),num(0),sInput1(0),sInput2(0){
		tmpUartD = (PX4::PX4UARTDriver*)hal.uartD;
		tmpUartB = (PX4::PX4UARTDriver*)hal.uartB;
		tmpUartE = (PX4::PX4UARTDriver*)hal.uartE;
		token.servo_token.head1 = 0x055;
		token.servo_token.head2 = 0x0AA;
		token.servo_token.sparse = 0x00;
}
AP_CHuart chuart;
