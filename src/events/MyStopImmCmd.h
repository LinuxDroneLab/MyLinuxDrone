/*
 * MyStopImmCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTOPIMMCMD_H_
#define EVENTS_MYSTOPIMMCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStopImmCmd: public MyCmd {
public:
	explicit MyStopImmCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStopImmCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTOPIMMCMD_H_ */
