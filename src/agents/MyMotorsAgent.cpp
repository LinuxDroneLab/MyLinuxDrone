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

MyMotorsAgent::MyMotorsAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), armed(false),
		front("/sys/devices/platform/ocp/ocp:P9_22_pinmux", "/sys/class/pwm/pwmchip0", 0),
		rear("/sys/devices/platform/ocp/ocp:P9_21_pinmux", "/sys/class/pwm/pwmchip0", 1),
		left("/sys/devices/platform/ocp/ocp:P8_19_pinmux", "/sys/class/pwm/pwmchip4", 0),
		right("/sys/devices/platform/ocp/ocp:P8_13_pinmux", "/sys/class/pwm/pwmchip4", 1)
{
}

MyMotorsAgent::~MyMotorsAgent() {
}

void MyMotorsAgent::initialize() {
	front.initialize();
	rear.initialize();
	left.initialize();
	right.initialize();
    this->disarmMotors();

	// TODO: inviare pwm min
	initialized = true;
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
	// cout << "Armed: MF: " << event->getFront() << ", MR: " << event->getRear() << ", ML: " << event->getLeft() << ", MR: " << event->getRight() << endl;
	front.setDutyCycleNanos(event->getFront());
	rear.setDutyCycleNanos(event->getRear());
	left.setDutyCycleNanos(event->getLeft());
	right.setDutyCycleNanos(event->getRight());
}
void MyMotorsAgent::disarmMotors() {
	front.setDutyCycleNanos(1000000);
	rear.setDutyCycleNanos(1000000);
	left.setDutyCycleNanos(1000000);
	right.setDutyCycleNanos(1000000);

	// TODO: disarm motors
	armed = false;
	boost::shared_ptr<MyMotorsDisarmed> evOut(
			boost::make_shared<MyMotorsDisarmed>(this->getUuid()));
	m_signal(evOut);
}
void MyMotorsAgent::armMotors() {
	front.setDutyCycleNanos(1000000);
	rear.setDutyCycleNanos(1000000);
	left.setDutyCycleNanos(1000000);
	right.setDutyCycleNanos(1000000);

	armed = true;
	boost::shared_ptr<MyMotorsArmed> evOut(
			boost::make_shared<MyMotorsArmed>(this->getUuid()));
	m_signal(evOut);
}
