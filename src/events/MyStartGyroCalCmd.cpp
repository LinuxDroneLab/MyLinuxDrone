/*
 * StartGyroCalCmd.cpp
 *
 *  Created on: 21 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyStartGyroCalCmd.h>

MyStartGyroCalCmd::MyStartGyroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::START_GYRO_CALIBRATION_PRIORITY);

}

MyStartGyroCalCmd::~MyStartGyroCalCmd() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyStartGyroCalCmd::getType() const {
	return MyEvent::EventType::StartGyroCalCmd;
}

