/*
 * EndGyriCalCmd.h
 *
 *  Created on: 21 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYENDGYROCALCMD_H_
#define EVENTS_MYENDGYROCALCMD_H_

#include <events/MyCmd.h>

class MyEndGyroCalCmd : MyCmd {
public:
	explicit MyEndGyroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyEndGyroCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYENDGYROCALCMD_H_ */
