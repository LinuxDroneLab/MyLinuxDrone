/*
 * ValueFloat.h
 *
 *  Created on: 17 ago 2017
 *      Author: andrea
 */

#ifndef COMMONS_VALUEFLOAT_H_
#define COMMONS_VALUEFLOAT_H_
#include <commons/RangeFloat.h>

class ValueFloat {
public:
	ValueFloat(ValueFloat& v, RangeFloat& range);
	ValueFloat(float v, RangeFloat& range);
	virtual ~ValueFloat();
	float getValue();
	float getValueAsPercent();
	void setValue(float v);
	void setValue(ValueFloat& v);
	void setPercentValue(float v);
	RangeFloat& getRange();
private:
	float value;
	RangeFloat& range;
};

#endif /* COMMONS_VALUEFLOAT_H_ */
