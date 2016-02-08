/*
 * MyDisarmMotorsCmd.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyDisarmMotorsCmd.h>

MyDisarmMotorsCmd::MyDisarmMotorsCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::DISARM_MOTORS_PRIORITY);
}

MyDisarmMotorsCmd::~MyDisarmMotorsCmd() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyDisarmMotorsCmd::getType() const {
	return MyEvent::EventType::DisarmMotorsCmd;
}

