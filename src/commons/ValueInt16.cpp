/*
 * ValueInt.cpp
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#include <commons/ValueInt16.h>
#include <cmath>

ValueInt16::ValueInt16(ValueInt16& v, RangeInt16& range) : range(range) {
	this->value = range.convert(v.getValue(), v.getRange());
}

ValueInt16::ValueInt16(int16_t v, RangeInt16& range) : range(range) {
	this->value = range.constraint(v);
}
ValueInt16::~ValueInt16() {
	// TODO Auto-generated destructor stub
}

// get value as [-100%, 100%]
float ValueInt16::getValueAsPercent() {
	return (float(this->value) - this->getRange().getCenter())/(float(this->getRange().getLength())/2.0f);
}

int16_t ValueInt16::getValue() {
	return this->value;
}
void ValueInt16::setValue(int16_t v) {
	this->value = this->range.constraint(v);
}

void ValueInt16::setValue(ValueInt16& v) {
	this->value = this->range.convert(v.getValue(), v.getRange());
}
void ValueInt16::setPercentValue(float v) {
	this->value = this->range.constraint(rint(this->getRange().getCenter() + v*float(this->getRange().getLength())/2.0f));
}

RangeInt16& ValueInt16::getRange() {
	return range;
}
