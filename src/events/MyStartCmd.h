/*
 * MyStartCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTARTCMD_H_
#define EVENTS_MYSTARTCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStartCmd: public MyCmd {
public:
	MyStartCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStartCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTARTCMD_H_ */
