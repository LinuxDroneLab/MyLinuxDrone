/*
 * RangeInt.h
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#ifndef COMMONS_RANGEINT16_H_
#define COMMONS_RANGEINT16_H_
#include <stdint.h>

class RangeInt16 {
public:
	RangeInt16(int16_t min, int16_t max);
	virtual ~RangeInt16();
public:
	int16_t constraint(int16_t v);
	int16_t convert(int16_t v, RangeInt16& range);
	bool contains(int16_t v);
	float getCenter();
	int16_t getLength();
private:
	int16_t min;
	int16_t max;
};

#endif /* COMMONS_RANGEINT16_H_ */
