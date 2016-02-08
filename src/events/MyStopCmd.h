/*
 * MyStopCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTOPCMD_H_
#define EVENTS_MYSTOPCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStopCmd: public MyCmd {
public:
	MyStopCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStopCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTOPCMD_H_ */
