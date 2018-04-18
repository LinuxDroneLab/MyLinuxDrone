/*
 * MinThrustMinPitch.h
 *
 *  Created on: 18 ago 2017
 *      Author: andrea
 */

#ifndef EVENTS_MINTHRUSTMINPITCH_H_
#define EVENTS_MINTHRUSTMINPITCH_H_
#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>


class MinThrustMinPitch: public MyEvent {
public:
	MinThrustMinPitch(boost::uuids::uuid origin);
	virtual ~MinThrustMinPitch();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MINTHRUSTMINPITCH_H_ */
