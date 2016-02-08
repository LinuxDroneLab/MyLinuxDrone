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
#include "dbus/MyPIDState.h"

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

			MyPIDState pidState;
			pidState.timestampMillis = outState->getTimestampMillis();
			pidState.yawSample.current_value = outState->getYawCurr();
			pidState.yawSample.target_value = outState->getYawTrg();
			pidState.yawSample.error_ = outState->getEYaw();
			pidState.yawSample.error_i = outState->getEIYaw();
			pidState.yawSample.error_d = outState->getEDYaw();

			pidState.pitchSample.current_value = outState->getPitchCurr();
			pidState.pitchSample.target_value = outState->getPitchTrg();
			pidState.pitchSample.error_ = outState->getEPitch();
			pidState.pitchSample.error_i = outState->getEIPitch();
			pidState.pitchSample.error_d = outState->getEDPitch();

			pidState.rollSample.current_value = outState->getRollCurr();
			pidState.rollSample.target_value = outState->getRollTrg();
			pidState.rollSample.error_ = outState->getERoll();
			pidState.rollSample.error_i = outState->getEIRoll();
			pidState.rollSample.error_d = outState->getEDRoll();

			MyPIDControllerServiceWrap::emitStateChangedSignal(pidState);

		}
	}
}
