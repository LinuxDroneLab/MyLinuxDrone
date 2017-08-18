/*
 * MinThrustMaxPitch.h
 *
 *  Created on: 18 ago 2017
 *      Author: andrea
 */

#ifndef EVENTS_MINTHRUSTMAXPITCH_H_
#define EVENTS_MINTHRUSTMAXPITCH_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MinThrustMaxPitch: public MyEvent {
public:
	MinThrustMaxPitch(boost::uuids::uuid origin);
	virtual ~MinThrustMaxPitch();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MINTHRUSTMAXPITCH_H_ */
