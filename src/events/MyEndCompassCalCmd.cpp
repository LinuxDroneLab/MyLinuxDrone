/*
 * MyEndCompassCalCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyEndCompassCalCmd.h>

MyEndCompassCalCmd::MyEndCompassCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::END_COMPASS_CALIBRATION_PRIORITY);
}

MyEndCompassCalCmd::~MyEndCompassCalCmd() {
}
MyEvent::EventType MyEndCompassCalCmd::getType() const {
	return MyEvent::EventType::EndCompassCalCmd;
}

