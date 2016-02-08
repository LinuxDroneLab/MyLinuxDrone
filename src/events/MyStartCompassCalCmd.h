/*
 * MyStartCompassCal.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTARTCOMPASSCALCMD_H_
#define EVENTS_MYSTARTCOMPASSCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStartCompassCalCmd: public MyCmd {
public:
	explicit MyStartCompassCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStartCompassCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTARTCOMPASSCALCMD_H_ */
