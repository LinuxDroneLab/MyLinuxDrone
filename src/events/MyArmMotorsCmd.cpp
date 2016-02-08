/*
 * MyArmMotorsCmd.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyArmMotorsCmd.h>

MyArmMotorsCmd::MyArmMotorsCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyCmd(origin, destination) {
	this->setPriority(MyPriority::ARM_MOTORS_PRIORITY);

}

MyArmMotorsCmd::~MyArmMotorsCmd() {
}

MyEvent::EventType MyArmMotorsCmd::getType() const {
	return MyEvent::EventType::ArmMotorsCmd;
}
