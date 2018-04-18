/*
 * MinThrustMaxPitch.cpp
 *
 *  Created on: 18 ago 2017
 *      Author: andrea
 */

#include <events/MinThrustMaxPitch.h>
#include <commons/MyPriority.h>

MinThrustMaxPitch::MinThrustMaxPitch(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::ARM_MOTORS_PRIORITY);

}

MinThrustMaxPitch::~MinThrustMaxPitch() {
}

MyEvent::EventType MinThrustMaxPitch::getType() const {
	return MyEvent::EventType::MinThrustMaxPitch;
}

