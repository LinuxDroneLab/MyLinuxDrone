/*
 * MyArmMotorsCmd.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYARMMOTORSCMD_H_
#define EVENTS_MYARMMOTORSCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyCmd.h>

class MyArmMotorsCmd: public MyCmd {
public:
	MyArmMotorsCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyArmMotorsCmd();
	virtual MyEvent::EventType getType() const;
};

#endif /* EVENTS_MYARMMOTORSCMD_H_ */
