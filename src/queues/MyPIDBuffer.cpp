/*
 * PIDBuffer.cpp
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#include <queues/MyPIDBuffer.h>

MyPIDBuffer::MyPIDBuffer() : m_period(0.01f), m_misureBuff(20), m_meanBuff(10), m_derivateBuff(4) {
}
MyPIDBuffer::MyPIDBuffer(float period, size_type meanDim, size_type integralDim, size_type derivateDim) : m_period(period), m_misureBuff(meanDim), m_meanBuff(integralDim), m_derivateBuff(derivateDim) {
}

MyPIDBuffer::~MyPIDBuffer() {
}

void MyPIDBuffer::push(param_type item) {
	value_type meanPrev = this->getMean();

	this->m_misureBuff.push_front(item);
	value_type meanCurr = this->getMean();

	this->m_meanBuff.push_front(meanCurr);

	this->m_derivateBuff.push_front(meanCurr - meanPrev);
}

float MyPIDBuffer::getMean() {
	if(!this->m_misureBuff.empty()) {
		value_type result = 0.0f;
		size_type size = this->m_misureBuff.size();
		for(value_type val : this->m_misureBuff) {
			result += val;
		}
		return result/size;
	} else {
		return 0.0;
	}
}

float MyPIDBuffer::getIntegral() {
	value_type result = 0.0f;
	for(value_type val : this->m_meanBuff) {
		result += val;
	}
	return result*this->m_period;
}

float MyPIDBuffer::getDerivate() {
	if(!this->m_derivateBuff.empty()) {
		value_type result = 0.0f;
		size_type size = this->m_derivateBuff.size();
		for(value_type val : this->m_derivateBuff) {
			result += val;
		}
		return (result/this->m_period)/size;
	} else {
		return 0.0;
	}
}

