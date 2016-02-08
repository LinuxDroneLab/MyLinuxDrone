/*
 * MyTargetSample.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYTARGETSAMPLE_H_
#define EVENTS_MYTARGETSAMPLE_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyTargetSample: public MyEvent {
public:
	explicit MyTargetSample(boost::uuids::uuid origin);
	virtual ~MyTargetSample();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYTARGETSAMPLE_H_ */
