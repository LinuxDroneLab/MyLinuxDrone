/*
 * MyEndAccellCalCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYENDACCELLCALCMD_H_
#define EVENTS_MYENDACCELLCALCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyEndAccellCalCmd: public MyCmd {
public:
	explicit MyEndAccellCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyEndAccellCalCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYENDACCELLCALCMD_H_ */
