/*
 * MyShutdownImmCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyShutdownImmCmd.h>

MyShutdownImmCmd::MyShutdownImmCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::SHUTDOWN_IMMEDIATELY_PRIORITY);
}

MyShutdownImmCmd::~MyShutdownImmCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyShutdownImmCmd::getType() const {
	return MyEvent::EventType::ShutdownImmCmd;
}
