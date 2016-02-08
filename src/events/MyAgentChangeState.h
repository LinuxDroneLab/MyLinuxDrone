/*
 * MyChangeState.h
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYAGENTCHANGESTATE_H_
#define EVENTS_MYAGENTCHANGESTATE_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>
#include <agents/MyAgentRole.h>
#include <agents/MyAgentState.h>

class MyAgentChangeState: public MyEvent {
public:
	MyAgentChangeState(boost::uuids::uuid origin, MyAgentRole role, MyAgentState state);
	virtual ~MyAgentChangeState();
	virtual MyEvent::EventType getType() const;
	MyAgentRole getAgentRole() const;
	MyAgentState getAgentState() const;
private:
	const MyAgentRole role;
	const MyAgentState state;
};

#endif /* EVENTS_MYAGENTCHANGESTATE_H_ */
