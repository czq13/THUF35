#ifndef SERVO_DATA
#define SERVO_DATA 1
#include <stdint.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include "AP_HAL_PX4/UARTDriver.h"

extern const AP_HAL::HAL& hal;
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
};
class AP_CHuart {
private:
	Servo_data sd[2];
	Servo_token token;
	uint8_t servoN;
	uint8_t servo_ToSend;
	PX4::PX4UARTDriver* tmpUartD;
	unsigned char buf[256];
public:
	AP_CHuart():servoN(2),servo_ToSend(0){
		tmpUartD = (PX4::PX4UARTDriver*)hal.uartD;
		token.head1 = 0x055;
		token.head2 = 0x0AA;
	}
	void update_data(unsigned char* p1,uint8_t t);
	void display_data(uint8_t t);
	void send_token();
	uint8_t readUart();
};
extern AP_CHuart chuart;

#endif
