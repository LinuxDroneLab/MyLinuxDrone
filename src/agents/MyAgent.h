/*
 * MyAgent.h
 *
 *  Created on: 12 dic 2015
 *      Author: andrea
 */

#ifndef MYAGENT_H_
#define MYAGENT_H_
#include <memory>
#include <boost/uuid/uuid.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/shared_ptr.hpp>
#include <events/MyEvent.h>
#include <queues/MyEventQueue.h>
#include <agents/MyAgentRole.h>
#include <agents/MyAgentState.h>
#include <bus/MyEventBus.h>

using namespace std;

class MyAgent {
public:
//	MyAgent();
	MyAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyAgent();

	typedef boost::signals2::signal<void (boost::shared_ptr<MyEvent>)> SignalType;
	boost::signals2::connection connect(const SignalType::slot_type &subscriber);
	void doEvent(boost::shared_ptr<MyEvent> event);
	virtual void operator()();
	MyAgentState getState() const;
	virtual MyAgentRole getRole() const;
	boost::uuids::uuid getUuid() const;
protected:
	virtual bool acceptEvent(boost::shared_ptr<MyEvent> event);
	virtual bool checkState(boost::shared_ptr<MyEvent> event);
	virtual void processEvent(boost::shared_ptr<MyEvent> event);
	typedef typename MyEventQueue::value_type value_type;
	SignalType m_signal;
	MyEventQueue received;
	const boost::uuids::uuid uuid;
	const boost::shared_ptr<MyEventBus> bus;
	boost::signals2::connection busConnection;
	boost::signals2::connection thisConnection;
	void disconnectAll();
private:
	vector<MyEvent::EventType> acceptedEventTypes;
	MyAgentState currentState;
	void setState(const MyAgentState state);
};

#endif /* MYAGENT_H_ */
