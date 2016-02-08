/*
 * MyStartCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStartCmd.h>

MyStartCmd::MyStartCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination){
	this->setPriority(MyPriority::START_PRIORITY);
}

MyStartCmd::~MyStartCmd() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyStartCmd::getType() const {
	return MyEvent::EventType::StartCmd;
}
