/*
 * MyTick.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyTick.h>

MyTick::MyTick(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::TICK_PRIORITY);
}

MyTick::~MyTick() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyTick::getType() const {
	return MyEvent::EventType::Tick;
}
