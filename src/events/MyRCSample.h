/*
 * MyRCSample.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYRCSAMPLE_H_
#define EVENTS_MYRCSAMPLE_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyRCSample: public MyEvent {
public:
	explicit MyRCSample(boost::uuids::uuid origin);
	virtual ~MyRCSample();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYRCSAMPLE_H_ */
