/*
 * MyAgent.cpp
 *
 *  Created on: 12 dic 2015
 *      Author: andrea
 */

#include "MyAgent.h"
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/trivial.hpp>
#include "commons/MyUuid.h"
#include <boost/uuid/uuid_io.hpp>
#include <events/MyAgentChangeState.h>
#include <events/MyCmd.h>

//MyAgent::MyAgent() :
//		uuid(MyUuid::generateUuid()), currentState(
//				MyAgentState::Active) {
//	// TODO: fare un costruttore con capacity come argomento
//	// oppure definire parametro di configurazione
//}
MyAgent::MyAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		uuid(MyUuid::generateUuid()), bus(bus), acceptedEventTypes(
				acceptedEventTypes), currentState(MyAgentState::Active) {
	// TODO: fare un costruttore con capacity come argomento
	// oppure definire parametro di configurazione

	// connect Bus as Observer
	busConnection = m_signal.connect(
			boost::bind(&MyEventBus::doEvent, bus, _1)); // bus observe me
	thisConnection = bus->connect(boost::bind(&MyAgent::doEvent, this, _1)); // I observe the bus
}

MyAgent::~MyAgent() {
	this->disconnectAll();
}
void MyAgent::disconnectAll() {
	if (thisConnection.connected()) {
		thisConnection.disconnect();
	}
	if (busConnection.connected()) {
		busConnection.disconnect();
	}

}
boost::signals2::connection MyAgent::connect(
		const SignalType::slot_type &subscriber) {
	return m_signal.connect(subscriber);
}

MyAgentState MyAgent::getState() const {
	return currentState;
}
void MyAgent::setState(const MyAgentState state) {
	this->currentState = state;
	boost::shared_ptr<MyAgentChangeState> evOut(
			boost::make_shared<MyAgentChangeState>(uuid, this->getRole(),
					this->getState()));
	m_signal(evOut);
}

MyAgentRole MyAgent::getRole() const {
	return MyAgentRole::Generic;
}

void MyAgent::operator()() {
	bool done = false;
	// ciclo di lettura eventi
	while (!done) {
		MyAgentState state = this->currentState;
		switch (state) {
		case MyAgentState::Active: {
			value_type ev = received.pop();
			this->processEvent(ev);
			break;
		}
		case MyAgentState::ShuttingDown: {
			if (received.isNotEmpty()) {
				value_type ev = received.pop();
				this->processEvent(ev);
			} else {
				this->setState(MyAgentState::ShutDownComplete);
			}
			break;
		}
		case MyAgentState::Stopping: {
			if (received.isNotEmpty()) {
				value_type ev = received.pop();
				this->processEvent(ev);
			} else {
				this->setState(MyAgentState::Stopped);
			}
			break;
		}
		case MyAgentState::Stopped: {
			value_type ev = received.pop();
			this->processEvent(ev);
			break;
		}
		case MyAgentState::ShutDownComplete: {
			done = true;
			break;
		}
		}
	}
	this->disconnectAll();
}
void MyAgent::doEvent(boost::shared_ptr<MyEvent> event) {
	if (!(this->acceptEvent(event) && this->checkState(event))) {
		return;
	}
	received.push(event);
}
void MyAgent::processEvent(boost::shared_ptr<MyEvent> event) {
}
bool MyAgent::acceptEvent(boost::shared_ptr<MyEvent> event) {
	bool result = false;
	// skip my events
	if (event->getOrigin() == this->uuid) {
		result = false;
	} else if (event->isExpired()) { // skip expired
		result = false;
	} else {
		for (MyEvent::EventType t : this->acceptedEventTypes) { // check if accepted events
			if (event->getType() == t) {
				if(event->isCommand()) { // check destination for command
						boost::shared_ptr<MyCmd> cmd = boost::static_pointer_cast<MyCmd>(event);
						result = this->uuid == cmd->getDestination();
				} else {
					result = true;
				}
				break;
			}
		}
	}
	return result;
}

bool MyAgent::checkState(boost::shared_ptr<MyEvent> event) {
	bool result = false;
	// check events/state consistency
	MyAgentState state = currentState;
	if ((event->getType() == MyEvent::EventType::StartCmd)
			&& ((state == MyAgentState::Stopping)
					|| (state == MyAgentState::Stopped))) {
		result = true;
	} else if (state == MyAgentState::Active) {
		result = true;
	}
	return result;
}

boost::uuids::uuid MyAgent::getUuid() const {
	return uuid;
}
