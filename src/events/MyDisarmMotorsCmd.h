/*
 * MyDisarmMotorsCmd.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYDISARMMOTORSCMD_H_
#define EVENTS_MYDISARMMOTORSCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyDisarmMotorsCmd: public MyCmd {
public:
	MyDisarmMotorsCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyDisarmMotorsCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYDISARMMOTORSCMD_H_ */
