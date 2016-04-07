#ifndef SERVO_DATA
#define SERVO_DATA 1
#include <stdint.h>
#include <string.h>

#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include "AP_HAL_PX4/UARTDriver.h"


struct Servo_data {
	uint8_t head1;
	uint8_t head2;
	uint8_t len;
	uint8_t status;
	int16_t input;
	int16_t pos;
	int16_t vel;
	uint8_t checkSum;
};
struct Servo_token {
	uint8_t head1;
	uint8_t head2;
	uint8_t num;
	uint8_t parse;
};
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
public:
	uint8_t num;
	Servo_data sd[2];
	AP_CHuart();
	void update_data(unsigned char* p1,uint8_t t);
	void display_data(uint8_t t);
	void send_token();
	uint8_t readUart();
};
extern AP_CHuart chuart;

#endif
