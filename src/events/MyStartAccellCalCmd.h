/*
 * MyAccelCalCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTARTACCELLCALCMD_H_
#define EVENTS_MYSTARTACCELLCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyStartAccellCalCmd: public MyCmd {
public:
	explicit MyStartAccellCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStartAccellCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTARTACCELLCALCMD_H_ */
