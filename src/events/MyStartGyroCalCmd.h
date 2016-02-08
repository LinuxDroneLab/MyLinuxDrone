/*
 * StartGyroCalCmd.h
 *
 *  Created on: 21 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYSTARTGYROCALCMD_H_
#define EVENTS_MYSTARTGYROCALCMD_H_

#include <events/MyCmd.h>

class MyStartGyroCalCmd: public MyCmd {
public:
	explicit MyStartGyroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyStartGyroCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYSTARTGYROCALCMD_H_ */
