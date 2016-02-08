/*
 * MyStopCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStopCmd.h>

MyStopCmd::MyStopCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::STOP_PRIORITY);
}

MyStopCmd::~MyStopCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyStopCmd::getType() const {
	return MyEvent::EventType::StopCmd;
}
