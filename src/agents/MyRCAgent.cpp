/*
 * MyRCAgent.cpp
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#include <agents/MyRCAgent.h>
#include <events/MyRCSample.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/log/trivial.hpp>
#include <pthread.h>
#include <events/MinThrustMinPitch.h>
#include <events/MinThrustMaxPitch.h>
#include <syslog.h>
#include <fcntl.h>
#include <pru_rc_lib.h>


#define CHAN_THRUST 1
#define CHAN_ROLL 0
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_AUX1 4
#define CHAN_AUX2 5

#define MYRCAGENT_MAX_BUFFER_SIZE         512
unsigned char MyRCAgent::readBuf[MYRCAGENT_MAX_BUFFER_SIZE] = {};
#define MYRCAGENT_DEVICE_NAME             "/dev/rpmsg_pru31"

/*
 * TODO: Change static values to dynamic configuration
 */
RangeInt16 MyRCAgent::PRU_RANGES[] = { RangeInt16(628, 1586), RangeInt16(604,
        1583), RangeInt16(665, 1531), RangeInt16(604, 1581), RangeInt16(595,
        1588), RangeInt16(596, 1587) };
RangeInt16 MyRCAgent::CHAN_RANGES[] = { RangeInt16(-500, 500), RangeInt16(-500,
        500), RangeInt16(-500, 500), RangeInt16(-500, 500), RangeInt16(-500,
        500), RangeInt16(-500, 500) };
ValueInt16 MyRCAgent::PRU_VALUES[] = { ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_ROLL]), ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_THRUST]), ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_PITCH]), ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_YAW]), ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_AUX1]), ValueInt16(0,
        MyRCAgent::PRU_RANGES[CHAN_AUX2]) };
ValueInt16 MyRCAgent::CHAN_VALUES[] = { ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_ROLL]), ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_THRUST]), ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_PITCH]), ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_YAW]), ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_AUX1]), ValueInt16(0,
        MyRCAgent::CHAN_RANGES[CHAN_AUX2]) };

MyRCAgent::MyRCAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes)  : MyAgent(bus, acceptedEventTypes), initialized(false) {
	this->thrust=0;
	this->roll=0;
	this->pitch=0;
	this->yaw=0;
	this->aux1=0;
	this->aux2=0;
	this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
	this->tickDTimeSum = 0;
	this->tickDTimeWaitMicros = 100000; // with tick at 200Hz => dTimeMicros = 5000 => get data for each 100000/5000 = 20 cycles (10Hz)
}

MyRCAgent::~MyRCAgent() {
}

void MyRCAgent::initialize() {
	if(!initialized) {
		syslog(LOG_INFO, "mydrone: MyRCAgent: initializing ...");

	    /* Open the rpmsg_pru character device file */
	    pruDevice.fd = open(MYRCAGENT_DEVICE_NAME, O_RDWR);

	    if (pruDevice.fd < 0) {
	            syslog(LOG_ERR, "mydrone: MyRCAgent: Failed to open [%s] ", MYRCAGENT_DEVICE_NAME);
	            initialized = false;
	            return;
	    }

		/*
		 * TODO:
		 * open device /dev/rpmsg_31 in R/W mode. Set fd to private attribute
		 * close device when change state from active to not active
		 */
        initialized = true;
	}
}

void MyRCAgent::setRCSample(float thrust, float roll, float pitch, float yaw, float aux1, float aux2) {
	this->thrust=thrust;
	this->roll=roll;
	this->pitch=pitch;
	this->yaw=yaw;
	this->aux1=aux1;
	this->aux2=aux2;

}

bool MyRCAgent::sendDataRequest() {
    for(int i = 0; i < MYRCAGENT_MAX_BUFFER_SIZE; i++) {
        readBuf[i] = 0;
    }
    readBuf[0] = PRU_RC_LIB_CMD_ID;
    readBuf[1] = PRU_RC_LIB_CMD_GET_DATA;
    int result = write(pruDevice.fd, readBuf, 2);
//    if (result > 0) {
//        printf("Message GET_DATA sent to PRU\n");
//    }
    return (result > 0);
}
bool MyRCAgent::receiveData() {
    bool result = false;
    uint32_t* data = (uint32_t*)(readBuf+2);
    int bytes = read(pruDevice.fd, readBuf, sizeof(struct EcapData));
    if (bytes > 0) {
//        printf("Message received from PRU:%d:%d:%d 1\n",readBuf[0], readBuf[1], bytes);
        if((readBuf[0] == PRU_RC_LIB_CMD_ID)
                && (readBuf[1] == PRU_RC_LIB_CMD_GET_DATA_RSP)) {
            for(int j = 0; j < 6; j++) {
                MyRCAgent::PRU_VALUES[j].setValue(uint16_t(data[j]/200));
                MyRCAgent::CHAN_VALUES[j].setValue(
                        MyRCAgent::PRU_VALUES[j]);
            }
            this->setRCSample(
                    MyRCAgent::CHAN_VALUES[CHAN_THRUST].getValueAsPercent(),
                    MyRCAgent::CHAN_VALUES[CHAN_ROLL].getValueAsPercent(),
                    MyRCAgent::CHAN_VALUES[CHAN_PITCH].getValueAsPercent(),
                    MyRCAgent::CHAN_VALUES[CHAN_YAW].getValueAsPercent(),
                    MyRCAgent::CHAN_VALUES[CHAN_AUX1].getValueAsPercent(),
                    MyRCAgent::CHAN_VALUES[CHAN_AUX2].getValueAsPercent());
        }
        result = true;
    }

    return result;
}

bool MyRCAgent::updateTickTimestamp() {
    uint32_t now = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
    lastDTimeMicros = uint32_t(now - lastTickMicros);
    lastTickMicros = now;
    this->tickDTimeSum += this->lastDTimeMicros;
    if(this->tickDTimeSum > this->tickDTimeWaitMicros) {
        this->tickDTimeSum -= this->tickDTimeWaitMicros;
        return true;
    }
    return false;
}



/*
 * Count tick events. If (num ticks % divider) == 0, then go to RequireMode State
 * States:
 * - Idle: nothing to do
 * - RequireMode: send 'require data' message to PRU.
 * - Transition: RequireMode -> ReceiveMode. Immediate on RequireMode activity done.
 * - ReceiveMode: non blocking read data from /dev/rpmsg_31 (PRU1).
 * - Transition: ReceiveMode -> Idle if ((num ticks % divider) != 0 and data received)
 * - Transition: ReceiveMode -> RequireMode if (num tiks % divider) == 0
 * - Transition: Idle -> RequireMode if (num ticks % divider) == 0
 */
void MyRCAgent::pulse() {
    bool updatedRc = this->updateTickTimestamp();
    switch(this->status) {
    case (MYRCAGENT_STATUS_REQUIRE_MODE): {
        if(this->sendDataRequest()) {
            this->status = MYRCAGENT_STATUS_RECEIVE_MODE;
        }
        break;
    }
    case (MYRCAGENT_STATUS_RECEIVE_MODE): {
        if(this->receiveData()) {
            if(thrust <= -0.98f && pitch >= 0.98f) {
                boost::shared_ptr<MinThrustMaxPitch> armMotors(boost::make_shared<MinThrustMaxPitch>(uuid));
                m_signal(armMotors);
            } else if(thrust <= -0.98f && pitch <= -0.98f) {
                boost::shared_ptr<MinThrustMinPitch> disarmMotors(boost::make_shared<MinThrustMinPitch>(uuid));
                m_signal(disarmMotors);
            } else {
                boost::shared_ptr<MyEvent> evOut(boost::make_shared<MyRCSample>(this->getUuid(), thrust, roll, pitch, yaw, aux1, aux2));
                m_signal(evOut);
            }
            this->status = MYRCAGENT_STATUS_WAIT_MODE;
        } else if(updatedRc) {
            this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
        }
        break;
    }
    case (MYRCAGENT_STATUS_WAIT_MODE): {
        if(updatedRc) {
                    this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
        }
        break;
    }
    }

}
void MyRCAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if(!initialized) {
		initialize();
	}
	if(this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::Tick) {
		    this->pulse();
		}
	}
}
