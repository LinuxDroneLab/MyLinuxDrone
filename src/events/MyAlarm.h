/*
 * MyAlarm.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYALARM_H_
#define EVENTS_MYALARM_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyAlarm: public MyEvent {
public:
	explicit MyAlarm(boost::uuids::uuid origin);
	virtual ~MyAlarm();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYALARM_H_ */
