/*
 * PIDBuffer.cpp
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#include <math.h>
#include <queues/MyPIDBuffer.h>

RangeFloat MyPIDBuffer::DEFAULT_INTEGRAL_RANGE = RangeFloat(-1.0f, 1.0f);

MyPIDBuffer::MyPIDBuffer() : m_period(1.0f), m_misureBuff(20), m_meanBuff(10), m_squareBuff(10), m_rmsBuff(10), m_derivateBuff(4), m_integral(0.0f, DEFAULT_INTEGRAL_RANGE) {
}
MyPIDBuffer::MyPIDBuffer(float period, size_type meanDim, size_type integralDim, size_type derivateDim, RangeFloat &integralRange) : m_period(period), m_misureBuff(meanDim), m_meanBuff(integralDim), m_squareBuff(meanDim), m_rmsBuff(meanDim), m_derivateBuff(derivateDim), m_integral(0.0f, integralRange) {

}

MyPIDBuffer::~MyPIDBuffer() {
}
void MyPIDBuffer::clean() {
	this->m_misureBuff.clear();
	this->m_meanBuff.clear();
    this->m_squareBuff.clear();
    this->m_rmsBuff.clear();
	this->m_derivateBuff.clear();
	this->m_integral.setValue(0.0f);
}
void MyPIDBuffer::push(param_type item) {
	value_type meanPrev = this->getMean();

	this->m_misureBuff.push_front(item);
    this->m_squareBuff.push_front(item*item);

	value_type meanCurr = this->getMean();
    value_type rmsCurr = this->getRMS();
	this->m_meanBuff.push_front(meanCurr);
    this->m_rmsBuff.push_front(rmsCurr);

	m_integral.setValue(m_integral.getValue() + meanCurr*this->m_period);
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

float MyPIDBuffer::getRMS() {
    if(!this->m_squareBuff.empty()) {
        value_type result = 0.0f;
        size_type size = this->m_squareBuff.size();
        for(value_type val : this->m_squareBuff) {
            result += val;
        }
        return sqrt(result/size);
    } else {
        return 0.0;
    }
}

float MyPIDBuffer::getMinRMS() {
    if(!this->m_rmsBuff.empty()) {
        value_type result = 99999999.0f;
        for(value_type val : this->m_rmsBuff) {
            result = result <= val ? result : val;
        }
        return result;
    } else {
        return 0.0;
    }
}
float MyPIDBuffer::getMaxRMS() {
    if(!this->m_rmsBuff.empty()) {
        value_type result = 0.0f;
        for(value_type val : this->m_rmsBuff) {
            result = result >= val ? result : val;
        }
        return result;
    } else {
        return 0.0;
    }
}

float MyPIDBuffer::getCenterRMS() {
    return sqrt((this->getMinRMS()*this->getMinRMS() + this->getMaxRMS()*this->getMaxRMS())/2.0f);
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
	return m_integral.getValue();
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

