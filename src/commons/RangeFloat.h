/*
 * RangeFloat.h
 *
 *  Created on: 17 ago 2017
 *      Author: andrea
 */

#ifndef COMMONS_RANGEFLOAT_H_
#define COMMONS_RANGEFLOAT_H_
#include <stdint.h>

class RangeFloat {
public:
	RangeFloat(float min, float max);
	virtual ~RangeFloat();
public:
	float constraint(float v);
	float convert(float v, RangeFloat& range);
	bool contains(float v);
	float getCenter();
	float getLength();
private:
	float min;
	float max;
};

#endif /* COMMONS_RANGEFLOAT_H_ */
