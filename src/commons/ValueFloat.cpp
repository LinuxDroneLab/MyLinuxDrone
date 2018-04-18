/*
 * ValueFloat.cpp
 *
 *  Created on: 17 ago 2017
 *      Author: andrea
 */

#include <commons/ValueFloat.h>

#include <cmath>

ValueFloat::ValueFloat(ValueFloat& v, RangeFloat& range) : range(range) {
	this->value = range.convert(v.getValue(), v.getRange());
}

ValueFloat::ValueFloat(float v, RangeFloat& range) : range(range) {
	this->value = range.constraint(v);
}
ValueFloat::~ValueFloat() {
	// TODO Auto-generated destructor stub
}

// get value as [-100%, 100%]
float ValueFloat::getValueAsPercent() {
	return (this->value - this->getRange().getCenter())/(this->getRange().getLength()/2.0f);
}

float ValueFloat::getValue() {
	return this->value;
}
void ValueFloat::setValue(float v) {
	this->value = this->range.constraint(v);
}

void ValueFloat::setValue(ValueFloat& v) {
	this->value = this->range.convert(v.getValue(), v.getRange());
}
void ValueFloat::setPercentValue(float v) {
	this->value = this->range.constraint(this->getRange().getCenter() + v*this->getRange().getLength()/2.0f);
}

RangeFloat& ValueFloat::getRange() {
	return range;
}
