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
	MyOutMotors(boost::uuids::uuid origin, uint16_t front, uint16_t rear, uint16_t left, uint16_t right);
	virtual ~MyOutMotors();
	virtual MyEvent::EventType getType() const;
	uint16_t getFront() const;
	uint16_t getRear() const;
	uint16_t getLeft() const;
	uint16_t getRight() const;
private:
	const uint16_t front;
	const uint16_t rear;
	const uint16_t left;
	const uint16_t right;
};

#endif /* EVENTS_MYOUTMOTORS_H_ */
