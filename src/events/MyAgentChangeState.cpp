/*
 * MyChangeState.cpp
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyAgentChangeState.h>

MyAgentChangeState::MyAgentChangeState(boost::uuids::uuid origin, MyAgentRole role, MyAgentState state) : MyEvent(origin), role(role), state(state) {
	this->setPriority(MyPriority::CHANGE_STATE_PRIORITY);
}

MyAgentChangeState::~MyAgentChangeState() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyAgentChangeState::getType() const {
	return MyEvent::EventType::ChangeState;
}
