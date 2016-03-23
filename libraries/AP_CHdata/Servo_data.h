#ifndef SERVO_DATA
#define SERVO_DATA 1
#include <stdint.h>
#include <string.h>
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
class CHServo {
private:
	Servo_data sd;
public:
	void init_data(unsigned char* p1);
	void display_data();
};
//void CHServo::display_data()
//void CHServo::init_data(unsigned char* p1)

#endif
