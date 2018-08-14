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
#include <sys/poll.h>

class MyMotorsAgent: public MyAgent {
public:
	MyMotorsAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyMotorsAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
    static unsigned char readBuf[];
	bool initialized;
	void initialize();
	bool armed;
	void writeMotors(boost::shared_ptr<MyOutMotors> event) ;
	void setThrottleRange();
	void disarmMotors();
	void armMotors();
    bool sendData();
    bool sendStart();
    bool sendStop();
    bool receiveData();
    bool sendDuty(uint16_t du1A, uint16_t du1B, uint16_t du2A, uint16_t du2B);
    void cleanBuffer();
    struct pollfd pruDevice;
};

#endif /* AGENTS_MYMOTORSAGENT_H_ */
