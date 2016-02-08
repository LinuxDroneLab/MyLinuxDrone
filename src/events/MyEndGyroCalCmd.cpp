/*
 * EndGyriCalCmd.cpp
 *
 *  Created on: 21 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyEndGyroCalCmd.h>

MyEndGyroCalCmd::MyEndGyroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::END_GYRO_CALIBRATION_PRIORITY);

}

MyEndGyroCalCmd::~MyEndGyroCalCmd() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyEndGyroCalCmd::getType() const {
	return MyEvent::EventType::EndGyroCalCmd;
}

