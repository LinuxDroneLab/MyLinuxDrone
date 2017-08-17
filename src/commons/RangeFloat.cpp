/*
 * RangeFloat.cpp
 *
 *  Created on: 17 ago 2017
 *      Author: andrea
 */

#include <commons/RangeFloat.h>

#include <cmath>
#include <iostream>

RangeFloat::RangeFloat(float min, float max) : min(min), max(max) {

}

RangeFloat::~RangeFloat() {
	// TODO Auto-generated destructor stub
}
float RangeFloat::getCenter() {
    return (max + min)/2.0f;
}
float RangeFloat::getLength() {
	return max - min;
}

float RangeFloat::constraint(float v) {
	return v < min ? min : (v > max ? max : v);
}
float RangeFloat::convert(float v, RangeFloat& range) {
	float vf = float(range.constraint(v));
	float result =  (vf - range.getCenter())/range.getLength() * this->getLength() + this->getCenter();
	return result;
}
bool RangeFloat::contains(float v) {
	return v >= min && v <= max;
}
