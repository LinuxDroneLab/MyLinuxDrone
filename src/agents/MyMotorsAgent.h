/*
 * MyMotorsAgent.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYMOTORSAGENT_H_
#define AGENTS_MYMOTORSAGENT_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <events/MyOutMotors.h>
#include <agents/MyAgent.h>
#include <motors/MotorDriver.h>

class MyMotorsAgent: public MyAgent {
public:
	MyMotorsAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyMotorsAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	bool initialized;
	void initialize();
	bool armed;
	void writeMotors(boost::shared_ptr<MyOutMotors> event) ;
	void disarmMotors();
	void armMotors();

	MotorDriver front;
	MotorDriver rear;
};

#endif /* AGENTS_MYMOTORSAGENT_H_ */
