/*
 * MyEndCompassCalCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYENDCOMPASSCALCMD_H_
#define EVENTS_MYENDCOMPASSCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyEndCompassCalCmd: public MyCmd {
public:
	explicit MyEndCompassCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyEndCompassCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYENDCOMPASSCALCMD_H_ */
