/*
 * PIDBuffer.cpp
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#include <queues/MyPIDBuffer.h>

MyPIDBuffer::MyPIDBuffer() : m_period(0.01f), m_misureBuff(20), m_meanBuff(10), m_derivateBuff(4), m_integral(0.0f) {
}
MyPIDBuffer::MyPIDBuffer(float period, size_type meanDim, size_type integralDim, size_type derivateDim) : m_period(period), m_misureBuff(meanDim), m_meanBuff(integralDim), m_derivateBuff(derivateDim), m_integral(0.0f) {
}

MyPIDBuffer::~MyPIDBuffer() {
}
void MyPIDBuffer::clean() {
	this->m_misureBuff.clear();
	this->m_meanBuff.clear();
	this->m_derivateBuff.clear();
	this->m_integral = 0.0f;
}
void MyPIDBuffer::push(param_type item) {
	value_type meanPrev = this->getMean();

	this->m_misureBuff.push_front(item);
	value_type meanCurr = this->getMean();

	this->m_meanBuff.push_front(meanCurr);
	m_integral+= meanCurr*this->m_period;
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
//	value_type result = 0.0f;
//	for(value_type val : this->m_meanBuff) {
		// per integrale considero solo errori grandi
		// gli errori minori verranno corretti direttamente
		//if(val > 5.0f || val < -5.0f) {
//			result += val;
		//}
//	}
//	return result*this->m_period;
	return m_integral;
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

