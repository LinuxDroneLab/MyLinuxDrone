/*
 * MyDBusEmitterAgent.h
 *
 *  Created on: 08 gen 2016
 *      Author: andrea
 */

#ifndef AGENTS_MYDBUSEMITTERAGENT_H_
#define AGENTS_MYDBUSEMITTERAGENT_H_

#include <agents/MyAgent.h>

class MyDBusEmitterAgent: public MyAgent {
public:
	MyDBusEmitterAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyDBusEmitterAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	bool initialized;
	void initialize();

};

#endif /* AGENTS_MYDBUSEMITTERAGENT_H_ */
