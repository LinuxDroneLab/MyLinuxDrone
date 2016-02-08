/*
 * MyAlarm.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyAlarm.h>

MyAlarm::MyAlarm(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::ALARM_PRIORITY);
}

MyAlarm::~MyAlarm() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyAlarm::getType() const {
	return MyEvent::EventType::Alarm;
}
