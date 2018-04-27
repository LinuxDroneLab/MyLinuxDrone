/*
 * MyRCAgent.h
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#ifndef AGENTS_MYRCAGENT_H_
#define AGENTS_MYRCAGENT_H_

#include <agents/MyAgent.h>
#include <boost/thread.hpp>
#include <commons/ValueInt16.h>

class MyRCAgent: public MyAgent {
public:
	MyRCAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyRCAgent();
	void setRCSample(float thrust, float roll, float pitch, float yaw, float aux1, float aux2);
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
    static RangeInt16 PRU_RANGES[];
    static RangeInt16 CHAN_RANGES[];
    static ValueInt16 PRU_VALUES[];
    static ValueInt16 CHAN_VALUES[];
    static unsigned char readBuf[];

	bool initialized;
	void initialize();
	float thrust;
	float roll;
	float pitch;
	float yaw;
	float aux1;
	float aux2;
};

#endif /* AGENTS_MYRCAGENT_H_ */
