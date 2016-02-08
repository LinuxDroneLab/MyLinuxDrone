/*
 * MyDBusEmitterAgent.cpp
 *
 *  Created on: 08 gen 2016
 *      Author: andrea
 */

#include <agents/MyDBusEmitterAgent.h>
#include <dbus/MyPIDControllerServiceWrap.h>
#include <events/MyYPRError.h>
#include <iostream>

MyDBusEmitterAgent::MyDBusEmitterAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false) {

}

MyDBusEmitterAgent::~MyDBusEmitterAgent() {

}

void MyDBusEmitterAgent::initialize() {
//	std::cout<< "MyDBusEmitterAgent initializing dbus" << std::endl;
//	MyPIDControllerServiceWrap::initialize();
//	std::cout<< "MyDBusEmitterAgent initialized dbus" << std::endl;
	initialized = true;
}
void MyDBusEmitterAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if (!initialized) {
		initialize();
	}
	if (this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::YPRError) {
			boost::shared_ptr<MyYPRError> outState =
									boost::static_pointer_cast<MyYPRError>(event);

			MyPIDControllerServiceWrap::emitStateChangedSignal(outState->getTimestampMillis(),
					outState->getYawCurr(), outState->getPitchCurr(), outState->getRollCurr(),
					outState->getYawTrg(), outState->getPitchTrg(), outState->getRollTrg(),
					outState->getERoll(),outState->getEIRoll(),outState->getEDRoll(),
					outState->getEPitch(),outState->getEIPitch(),outState->getEDPitch(),
					outState->getEYaw(),outState->getEIYaw(),outState->getEDYaw()
					);
		}
	}
}
