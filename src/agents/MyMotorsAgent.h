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
#include <sys/poll.h>

#define MYMOTORSAGENT_MAX_BUFFER_SIZE         64

class MyMotorsAgent {
public:
	MyMotorsAgent();
	virtual ~MyMotorsAgent();
    bool initialize();
    void disarmMotors();
    void armMotors();
    bool isArmed();
    void writeMotors(uint16_t rear, uint16_t front, uint16_t left, uint16_t right) ;
protected:

private:
    static unsigned char readBuf[MYMOTORSAGENT_MAX_BUFFER_SIZE];
	bool initialized;
	bool armed;
	void setThrottleRange();
    bool sendData();
    bool sendStart();
    bool sendStop();
    bool receiveData();
    bool sendDuty(uint16_t du1A, uint16_t du1B, uint16_t du2A, uint16_t du2B);
    void cleanBuffer();
    struct pollfd pruDevice;
};

#endif /* AGENTS_MYMOTORSAGENT_H_ */
