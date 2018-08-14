/*
 * MyMotorsAgent.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <agents/MyMotorsAgent.h>
#include <events/MyOutMotors.h>
#include <events/MyMotorsDisarmed.h>
#include <events/MyMotorsArmed.h>
#include <events/MinThrustMaxPitch.h>
#include <events/MinThrustMinPitch.h>
#include <syslog.h>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <pru_pwmss_driver.h>

#define MYMOTORSAGENT_MAX_BUFFER_SIZE         64
unsigned char MyMotorsAgent::readBuf[MYMOTORSAGENT_MAX_BUFFER_SIZE] = {0};
#define MYMOTORSAGENT_DEVICE_NAME             "/dev/rpmsg_pru30"
#define MYMOTORSAGENT_MIN_DUTY 3125
#define MYMOTORSAGENT_MAX_DUTY 6250
#define MYMOTORSAGENT_PERIOD 62499

MyMotorsAgent::MyMotorsAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), armed(false)
//		front("/sys/devices/platform/ocp/ocp:P9_22_pinmux", "/sys/class/pwm/pwmchip1", 1),
//		rear("/sys/devices/platform/ocp/ocp:P9_21_pinmux", "/sys/class/pwm/pwmchip1", 0),
//		left("/sys/devices/platform/ocp/ocp:P8_19_pinmux", "/sys/class/pwm/pwmchip5", 0),
//		right("/sys/devices/platform/ocp/ocp:P8_13_pinmux", "/sys/class/pwm/pwmchip5", 1)
{
}

MyMotorsAgent::~MyMotorsAgent() {
    close(pruDevice.fd);
}

void MyMotorsAgent::initialize() {
	syslog(LOG_INFO, "mydrone: MyMotorsAgent: initializing motor drivers");
	bool result = true;
    /* Open the rpmsg_pru character device file */
    pruDevice.fd = open(MYMOTORSAGENT_DEVICE_NAME, O_RDWR);

    if (pruDevice.fd < 0) {
            syslog(LOG_ERR, "mydrone: MyMotorsAgent: Failed to open [%s] ", MYMOTORSAGENT_DEVICE_NAME);
            initialized = false;
            return;
    }

//	result &= front.initialize();
//	result &= rear.initialize();
//	result &= left.initialize();
//	result &= right.initialize();

	if(result) {
		this->setThrottleRange();
	    this->disarmMotors();
		syslog(LOG_INFO, "mydrone: MotorsAgent: initialization OK");
	} else {
		syslog(LOG_ERR, "mydrone: MotorsAgent: initialization failed");
	}

	// TODO: inviare pwm min
	initialized = result;
}
void MyMotorsAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if (!initialized) {
		initialize();
	}
	if (this->getState() == MyAgentState::Active) {
		if (this->armed) {
			if (event->getType() == MyEvent::EventType::OutMotors) {
				boost::shared_ptr<MyOutMotors> outMotors =
						boost::static_pointer_cast<MyOutMotors>(event);
//				syslog(LOG_INFO, "mydrone: motors: f=%d, rr=%d, l=%d, r=%d", outMotors->getFront(), outMotors->getRear(), outMotors->getLeft(), outMotors->getRight());
//				printf("Front:%d, Rear=%d, Left=%d, Right=%d \n", outMotors->getFront(), outMotors->getRear(), outMotors->getLeft(), outMotors->getRight());
				this->writeMotors(outMotors);
			} else if (event->getType()
					== MyEvent::EventType::MinThrustMinPitch) {
				disarmMotors();
			}
		} else {
			if (event->getType() == MyEvent::EventType::MinThrustMaxPitch) {
				armMotors();
			} else if (event->getType() == MyEvent::EventType::OutMotors) {
				// skip event
			}
		}
	}
}

void MyMotorsAgent::writeMotors(boost::shared_ptr<MyOutMotors> event) {
    this->sendDuty(event->getRear(), event->getFront(), event->getLeft(), event->getRight());

	// cout << "Armed: MF: " << event->getFront() << ", MR: " << event->getRear() << ", ML: " << event->getLeft() << ", MR: " << event->getRight() << endl;
//	front.setDutyCycleNanos(event->getFront());
//	rear.setDutyCycleNanos(event->getRear());
//	left.setDutyCycleNanos(event->getLeft());
//	right.setDutyCycleNanos(event->getRight());
}
void MyMotorsAgent::setThrottleRange() {
    this->sendDuty(MYMOTORSAGENT_MAX_DUTY, MYMOTORSAGENT_MAX_DUTY, MYMOTORSAGENT_MAX_DUTY, MYMOTORSAGENT_MAX_DUTY);
//	front.setDutyCycleNanos(2000000);
//	rear.setDutyCycleNanos(2000000);
//	left.setDutyCycleNanos(2000000);
//	right.setDutyCycleNanos(2000000);
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);
//	front.setDutyCycleNanos(1000000);
//	rear.setDutyCycleNanos(1000000);
//	left.setDutyCycleNanos(1000000);
//	right.setDutyCycleNanos(1000000);
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MyMotorsAgent::disarmMotors() {
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);
//	front.setDutyCycleNanos(1000000);
//	rear.setDutyCycleNanos(1000000);
//	left.setDutyCycleNanos(1000000);
//	right.setDutyCycleNanos(1000000);

	// TODO: disarm motors
	armed = false;
	boost::shared_ptr<MyMotorsDisarmed> evOut(
			boost::make_shared<MyMotorsDisarmed>(this->getUuid()));
	m_signal(evOut);
}
void MyMotorsAgent::armMotors() {
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);
//	front.setDutyCycleNanos(1000000);
//	rear.setDutyCycleNanos(1000000);
//	left.setDutyCycleNanos(1000000);
//	right.setDutyCycleNanos(1000000);

	armed = true;
	boost::shared_ptr<MyMotorsArmed> evOut(
			boost::make_shared<MyMotorsArmed>(this->getUuid()));
	m_signal(evOut);
}
bool MyMotorsAgent::sendStart() {
    for(int i = 0; i < MYMOTORSAGENT_MAX_BUFFER_SIZE; i++) {
        readBuf[i] = 0;
    }
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_START;
    readBuf[2] = 5; // enable pwmss 0 and 2

    return (write(pruDevice.fd, readBuf, 3) > 0 ) && this->receiveData();
}
void MyMotorsAgent::cleanBuffer() {
    for(int i = 0; i < MYMOTORSAGENT_MAX_BUFFER_SIZE; i++) {
        readBuf[i] = 0;
    }
}

bool MyMotorsAgent::sendStop() {
    this->cleanBuffer();
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_STOP;
    readBuf[2] = 5; // disable pwmss 0 and 2
    return (write(pruDevice.fd, readBuf, 3) > 0 ) && this->receiveData();
}

bool MyMotorsAgent::sendDuty(uint16_t du1A, uint16_t du1B, uint16_t du2A, uint16_t du2B) {
    this->cleanBuffer();
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_SET_DATA;
    readBuf[2] = 0; // pwmss 0
    readBuf[9] = 2; // pwmss 2
    uint16_t* data1 = (uint16_t*)(readBuf+3);
    data1[0]=MYMOTORSAGENT_PERIOD;
    data1[1]=du1A;
    data1[2]=du1B;
    uint16_t* data2 = (uint16_t*)(readBuf+10);
    data2[0]=MYMOTORSAGENT_PERIOD;
    data2[1]=du2A;
    data2[2]=du2B;
    return (write(pruDevice.fd, readBuf, 16) > 0 ) && this->receiveData();
}

bool MyMotorsAgent::receiveData() {
    this->cleanBuffer();
    bool result = (read(pruDevice.fd, readBuf, 4) > 0);
    if (!result)
    {
        syslog(LOG_INFO, "mydrone: MyMotorsAgent: cannot receive data from PWMSS");
    } else {
        if((uint8_t)readBuf[3] != 1) {
            syslog(LOG_INFO, "mydrone: MyMotorsAgent ERROR: PWMSSs response=%d:%d:%d:%d\n", (uint8_t) readBuf[0],
                   (uint8_t) readBuf[1], (uint8_t) readBuf[2],
                   (uint8_t) readBuf[3]);
            result = false;
        }
    }
    return result;
}
