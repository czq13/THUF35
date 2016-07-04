/*
 * Servo_data.h
 *
 *  Created on: 2016年3月23日
 *      Author: THU czq
 *      描述：AP_CHuart主要实现了和舵机通信的功能。
 *      历史：
 *      	THU czq 2016/3/23  创建
 */
#ifndef SERVO_DATA
#define SERVO_DATA 1
#include <stdint.h>
#include <string.h>

#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include "AP_HAL_PX4/UARTDriver.h"

/*****************************************
 * struct : Servo_data
 * 作者：THU czq
 * 描述：用于记录舵机数据。与舵机发送相关数据相一致。
 * 日期：2016/5/5
 ***************************************** */
struct Servo_data {
	uint8_t head1;
	uint8_t head2;
	//uint8_t len;
	//int16_t input;
	int16_t pos;
	//int16_t vel;
	int16_t timer;
	//int8_t parse1;
	//int16_t parse2;
	uint8_t status;
	uint8_t checkSum;
};
/*****************************************
 * struct : Servo_token
 * 作者：THU czq
 * 描述：用于向舵机发送令牌
 * 日期：2016/5/5
 ***************************************** */
union Servo_token{
	struct {
		uint8_t head1;
		uint8_t head2;
		int16_t input;
		int16_t timer;
		uint8_t num;
		uint8_t checkSum;
		uint8_t sparse;
		uint8_t sparse1;
		uint8_t sparse2;
		uint8_t sparse3;
	}servo_token;
	uint8_t data[12];
};
/*****************************************
 * class : AP_CHuart
 * 作者：THU czq
 * 描述：用于处理串口信息，完成与舵机的通信功能
 * 日期：2016/5/5
 ***************************************** */
class AP_CHuart {
private:
	//this is the head
	Servo_token token;
	//How many actuator we have
	uint8_t servoN;
	//this is used to decide which uart we should send data
	uint8_t servo_ToSend;
	PX4::PX4UARTDriver* tmpUartD;
	unsigned char buf[256];
	int16_t ctimer1;
	int16_t ctimer2;
	float sInput1;
	float sInput2;
public:
	uint8_t num;
	Servo_data sd[2];
	AP_CHuart();
	void update_data(unsigned char* p1,uint8_t t);
	void display_data(uint8_t t);
	void send_token();
	void setServoCtrl(float c1,float c2);
	int8_t readUart();
};
extern AP_CHuart chuart;

#endif
