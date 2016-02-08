/*
 * MyCmd.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYCMD_H_
#define EVENTS_MYCMD_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyCmd: public MyEvent {
public:
	MyCmd(boost::uuids::uuid origin, boost::uuids::uuid destination);
	virtual ~MyCmd();
	virtual bool isCommand() const;
	boost::uuids::uuid getDestination() const;
private:
	const boost::uuids::uuid destination;
};

#endif /* EVENTS_MYCMD_H_ */
