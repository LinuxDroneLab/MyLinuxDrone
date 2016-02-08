/*
 * MyEndBaroCalCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYENDBAROCALCMD_H_
#define EVENTS_MYENDBAROCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyEndBaroCalCmd: public MyCmd {
public:
	explicit MyEndBaroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyEndBaroCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYENDBAROCALCMD_H_ */
