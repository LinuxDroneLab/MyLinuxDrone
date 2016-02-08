/*
 * MyAccelCalCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStartAccellCalCmd.h>

MyStartAccellCalCmd::MyStartAccellCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::START_ACCEL_CALIBRATION_PRIORITY);
}

MyStartAccellCalCmd::~MyStartAccellCalCmd() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyStartAccellCalCmd::getType() const {
	return MyEvent::EventType::StartAccellCalCmd;
}

