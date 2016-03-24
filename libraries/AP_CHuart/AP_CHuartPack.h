/*
 * AP_CHuartPack.h
 *
 *  Created on: 2016Äê3ÔÂ24ÈÕ
 *      Author: Thinkpad
 */

#ifndef LIBRARIES_AP_CHUART_AP_CHUARTPACK_H_
#define LIBRARIES_AP_CHUART_AP_CHUARTPACK_H_
#include "string.h"
#include <stdint.h>
class AP_CHuartPack{
private:
	uint8_t * bits;
	uint16_t maxLen;
	uint8_t po;
public:
	AP_CHuartPack(int tmaxLen) {
		//TODO:
		this->maxLen = tmaxLen;
		this->bits = new uint8_t[tmaxLen];
		this->po = 0;
	}
	AP_CHuartPack(uint8_t* tbits,int tmaxLen) {
		this->maxLen = tmaxLen;
		this->bits = new uint8_t[tmaxLen];
		memcpy(this->bits,tbits,tmaxLen);
		this->po = 0;
	}
	~AP_CHuartPack() {
		delete bits;
	}
	uint8_t setValue(uint32_t v,uint8_t len);
	//uint8_t getValue(uint32_t& v,uint8_t len);
	//uint8_t getStr(uint8_t* dst,uint8_t len);
};



#endif /* LIBRARIES_AP_CHUART_AP_CHUARTPACK_H_ */
