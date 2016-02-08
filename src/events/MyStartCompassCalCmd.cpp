/*
 * MyStartCompassCal.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStartCompassCalCmd.h>

MyStartCompassCalCmd::MyStartCompassCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::START_COMPASS_CALIBRATION_PRIORITY);
}

MyStartCompassCalCmd::~MyStartCompassCalCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyStartCompassCalCmd::getType() const {
	return MyEvent::EventType::StartCompassCalCmd;
}
