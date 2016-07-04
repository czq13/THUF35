/*
 * AP_CHuartPack.cpp
 *
 *  Created on: 2016年3月24日
 *      Author: Thinkpad
 *      描述：暂时没有用该函数和该类
 */

#include "AP_CHuartPack.h"
uint8_t AP_CHuartPack::setValue(uint32_t v,uint8_t len){
	if (this->po + len > this-> maxLen) {
		return -1;//not enough room!
	}
	while(len > 0) {
		this->bits[this->po++] = (uint8_t)(0x0ff & v);
		v = v >> 8;
		len--;
	}
	return 1;
}

