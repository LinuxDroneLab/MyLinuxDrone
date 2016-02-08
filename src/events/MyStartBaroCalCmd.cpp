/*
 * MyStartBaroCalCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStartBaroCalCmd.h>

MyStartBaroCalCmd::MyStartBaroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination)  {
	this->setPriority(MyPriority::START_BARO_CALIBRATION_PRIORITY);
}

MyStartBaroCalCmd::~MyStartBaroCalCmd() {
}

MyEvent::EventType MyStartBaroCalCmd::getType() const {
	return MyEvent::EventType::StartBaroCalCmd;
}
