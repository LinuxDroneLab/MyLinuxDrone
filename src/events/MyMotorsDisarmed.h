/*
 * MyMotorsDisarmed.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYMOTORSDISARMED_H_
#define EVENTS_MYMOTORSDISARMED_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyMotorsDisarmed: public MyEvent {
public:
	MyMotorsDisarmed(boost::uuids::uuid origin);
	virtual ~MyMotorsDisarmed();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYMOTORSDISARMED_H_ */
