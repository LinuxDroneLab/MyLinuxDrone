/*
 * MyMotorsArmed.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYMOTORSARMED_H_
#define EVENTS_MYMOTORSARMED_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyMotorsArmed: public MyEvent {
public:
	MyMotorsArmed(boost::uuids::uuid origin);
	virtual ~MyMotorsArmed();
	virtual MyEvent::EventType getType() const;

};

#endif /* EVENTS_MYMOTORSARMED_H_ */
