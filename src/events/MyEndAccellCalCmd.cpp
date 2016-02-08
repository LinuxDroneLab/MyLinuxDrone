/*
 * MyEndAccellCalCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyEndAccellCalCmd.h>

MyEndAccellCalCmd::MyEndAccellCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::END_GYRO_CALIBRATION_PRIORITY);
}

MyEndAccellCalCmd::~MyEndAccellCalCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyEndAccellCalCmd::getType() const {
	return MyEvent::EventType::EndAccellCalCmd;
}
