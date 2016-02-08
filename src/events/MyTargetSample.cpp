/*
 * MyTargetSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyTargetSample.h>

MyTargetSample::MyTargetSample(boost::uuids::uuid origin) : MyEvent(origin) {
	this->setPriority(MyPriority::TARGET_SAMPLE_PRIORITY);
}

MyTargetSample::~MyTargetSample() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyTargetSample::getType() const {
	return MyEvent::EventType::TargetSample;
}
