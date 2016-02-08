/*
 * MyStartBaroCalCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTARTBAROCALCMD_H_
#define EVENTS_MYSTARTBAROCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStartBaroCalCmd: public MyCmd {
public:
	explicit MyStartBaroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStartBaroCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTARTBAROCALCMD_H_ */
