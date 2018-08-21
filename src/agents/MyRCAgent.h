/*
 * MyRCAgent.h
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#ifndef AGENTS_MYRCAGENT_H_
#define AGENTS_MYRCAGENT_H_

#define MYRCAGENT_STATUS_REQUIRE_MODE 1
#define MYRCAGENT_STATUS_RECEIVE_MODE 2
#define MYRCAGENT_STATUS_WAIT_MODE    3


#include <boost/thread.hpp>
#include <commons/ValueInt16.h>
#include <sys/poll.h>

class MyRCAgent {
public:
    typedef struct {
        int16_t thrust;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
        int16_t aux1;
        int16_t aux2;
    } RCSample;
	MyRCAgent();
	virtual ~MyRCAgent();
    RCSample& getRCSample();
    bool loadData();
    bool initialize();
    bool isMinThrustMaxPitch();
    bool isMinThrustMinPitch();
protected:

private:
    static RangeInt16 PRU_RANGES[];
    static RangeInt16 CHAN_RANGES[];
    static ValueInt16 PRU_VALUES[];
    static ValueInt16 CHAN_VALUES[];
    static unsigned char readBuf[];

    struct EcapData
    {
        uint8_t cmd[2];
        uint32_t chanValue[8];
    };

	bool initialized;
    bool updateTickTimestamp();
	bool sendDataRequest();
	bool receiveData();
    void setRCSample();

	uint8_t status;
    struct pollfd pruDevice;
    RCSample rcSample;

	uint32_t lastTickMicros;
    uint16_t lastDTimeMicros;
    uint32_t tickDTimeSum;
    uint32_t tickDTimeWaitMicros;

    bool minThrustMaxPitch;
    bool minThrustMinPitch;

};

#endif /* AGENTS_MYRCAGENT_H_ */
