/*
 * MinThrustMinPitch.cpp
 *
 *  Created on: 18 ago 2017
 *      Author: andrea
 */

#include <events/MinThrustMinPitch.h>
#include <commons/MyPriority.h>

MinThrustMinPitch::MinThrustMinPitch(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::DISARM_MOTORS_PRIORITY);
}

MinThrustMinPitch::~MinThrustMinPitch() {
}

MyEvent::EventType MinThrustMinPitch::getType() const {
	return MyEvent::EventType::MinThrustMinPitch;
}

