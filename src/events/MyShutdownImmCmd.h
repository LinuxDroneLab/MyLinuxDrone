/*
 * MyShutdownImmCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSHUTDOWNIMMCMD_H_
#define EVENTS_MYSHUTDOWNIMMCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyShutdownImmCmd: public MyCmd {
public:
	explicit MyShutdownImmCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyShutdownImmCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSHUTDOWNIMMCMD_H_ */
