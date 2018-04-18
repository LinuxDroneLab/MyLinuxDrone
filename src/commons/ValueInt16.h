/*
 * ValueInt.h
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#ifndef COMMONS_VALUEINT16_H_
#define COMMONS_VALUEINT16_H_
#include <commons/RangeInt16.h>
class ValueInt16 {
public:
	ValueInt16(ValueInt16& v, RangeInt16& range);
	ValueInt16(int16_t v, RangeInt16& range);
	virtual ~ValueInt16();
	int16_t getValue();
	float getValueAsPercent();
	void setValue(int16_t v);
	void setValue(ValueInt16& v);
	void setPercentValue(float v);
	RangeInt16& getRange();
private:
	int16_t value;
	RangeInt16& range;
};

#endif /* COMMONS_VALUEINT16_H_ */
