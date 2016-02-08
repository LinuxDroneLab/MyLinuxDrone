/*
 * PIDBuffer.h
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#ifndef QUEUES_MYPIDBUFFER_H_
#define QUEUES_MYPIDBUFFER_H_
#include <boost/circular_buffer.hpp>
#include <boost/call_traits.hpp>

class MyPIDBuffer {
public:
	typedef boost::circular_buffer<float> container_type;
	typedef typename container_type::size_type size_type;
	typedef typename container_type::value_type value_type;
	typedef typename boost::call_traits<value_type>::param_type param_type;

	MyPIDBuffer();
	MyPIDBuffer(float period, size_type meanDim, size_type integralDim, size_type derivateDim);
	virtual ~MyPIDBuffer();

	void push(param_type item);
	float getMean();
	float getIntegral();
	float getDerivate();
private:
	float m_period;
	container_type m_misureBuff;
	container_type m_meanBuff;
	container_type m_derivateBuff;
};

#endif /* QUEUES_MYPIDBUFFER_H_ */
