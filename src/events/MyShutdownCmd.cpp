/*
 * MyShutdownCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyShutdownCmd.h>

MyShutdownCmd::MyShutdownCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::SHUTDOWN_PRIORITY);
}

MyShutdownCmd::~MyShutdownCmd() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyShutdownCmd::getType() const {
	return MyEvent::EventType::ShutdownCmd;
}

