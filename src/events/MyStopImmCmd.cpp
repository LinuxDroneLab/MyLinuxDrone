/*
 * MyStopImmCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStopImmCmd.h>

MyStopImmCmd::MyStopImmCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::STOP_IMMEDIATELY_PRIORITY);
}

MyStopImmCmd::~MyStopImmCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyStopImmCmd::getType() const {
	return MyEvent::EventType::StopImmCmd;
}
