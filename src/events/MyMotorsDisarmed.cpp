/*
 * MyMotorsDisarmed.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyMotorsDisarmed.h>

MyMotorsDisarmed::MyMotorsDisarmed(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::MOTORS_DISARMED_PRIORITY);
}

MyMotorsDisarmed::~MyMotorsDisarmed() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyMotorsDisarmed::getType() const {
	return MyEvent::EventType::MotorsDisarmed;
}

