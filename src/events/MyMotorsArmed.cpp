/*
 * MyMotorsArmed.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyMotorsArmed.h>

MyMotorsArmed::MyMotorsArmed(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::MOTORS_ARMED_PRIORITY);

}

MyMotorsArmed::~MyMotorsArmed() {
}

MyEvent::EventType MyMotorsArmed::getType() const {
	return MyEvent::EventType::MotorsArmed;
}
