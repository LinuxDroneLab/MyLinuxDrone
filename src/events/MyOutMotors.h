/*
 * MyOutMotors.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYOUTMOTORS_H_
#define EVENTS_MYOUTMOTORS_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyOutMotors: public MyEvent {
public:
	MyOutMotors(boost::uuids::uuid origin, int32_t front, int32_t rear, int32_t left, int32_t right);
	virtual ~MyOutMotors();
	virtual MyEvent::EventType getType() const;
	int32_t getFront() const;
	int32_t getRear() const;
	int32_t getLeft() const;
	int32_t getRight() const;
private:
	const int32_t front;
	const int32_t rear;
	const int32_t left;
	const int32_t right;
};

#endif /* EVENTS_MYOUTMOTORS_H_ */
