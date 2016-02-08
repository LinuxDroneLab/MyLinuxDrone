/*
 * MyTick.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYTICK_H_
#define EVENTS_MYTICK_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyTick: public MyEvent {
public:
	explicit MyTick(boost::uuids::uuid origin);
	virtual ~MyTick();
	virtual MyEvent::EventType getType() const;

};

#endif /* EVENTS_MYTICK_H_ */
