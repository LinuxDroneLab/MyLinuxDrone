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
#include <sys/poll.h>


#define CHAN_THRUST 1
#define CHAN_ROLL 0
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_AUX1 4
#define CHAN_AUX2 5

#define MAX_BUFFER_SIZE         512
unsigned char MyRCAgent::readBuf[MAX_BUFFER_SIZE] = {};
#define DEVICE_NAME             "/dev/rpmsg_pru31"

/*
 * TODO: Change static values to dynamic configuration
 */
RangeInt16 MyRCAgent::PRU_RANGES[] = { RangeInt16(602, 1584), RangeInt16(611,
        1578), RangeInt16(665, 1535), RangeInt16(612, 1572), RangeInt16(597,
        1589), RangeInt16(597, 1588) };
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
}

MyRCAgent::~MyRCAgent() {
}

void MyRCAgent::initialize() {
	if(!initialized) {
		syslog(LOG_INFO, "mydrone: MyRCAgent: initializing ...");

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

void MyRCAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if(!initialized) {
		initialize();
	}
	/*
	 * Count tick events. If (num tiks % divider) == 0, then go to RequireMode State
	 * States:
	 * - Idle: nothing to do
	 * - RequireMode: send 'require data' message to PRU.
	 * - Transition: RequireMode -> ReceiveMode. Immediate on RequireMode activity done.
	 * - ReceiveMode: non blocking read data from /dev/rpmsg_31 (PRU1).
	 * - Transition: ReceiveMode -> Idle if ((num tiks % divider) != 0 and data received)
     * - Transition: ReceiveMode -> RequireMode if (num tiks % divider) == 0
     * - Transition: Idle -> RequireMode if (num tiks % divider) == 0
	 */
	if(this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::Tick) {
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
		}
	}
}
