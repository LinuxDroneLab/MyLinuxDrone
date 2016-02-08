/*
 * MyRCSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyRCSample.h>

MyRCSample::MyRCSample(boost::uuids::uuid origin) : MyEvent(origin){
	this->setPriority(MyPriority::RC_SAMPLE_PRIORITY);
}

MyRCSample::~MyRCSample() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyRCSample::getType() const {
	return MyEvent::EventType::RCSample;
}
