/*
 * MyEndBaroCalCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyEndBaroCalCmd.h>

MyEndBaroCalCmd::MyEndBaroCalCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::END_BARO_CALIBRATION_PRIORITY);
}

MyEndBaroCalCmd::~MyEndBaroCalCmd() {

}
MyEvent::EventType MyEndBaroCalCmd::getType() const {
	return MyEvent::EventType::EndBaroCalCmd;
}
