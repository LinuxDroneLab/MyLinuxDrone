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

MyMotorsAgent::MyMotorsAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), armed(false),
		front("/sys/devices/platform/ocp/ocp:P9_22_pinmux", "/sys/class/pwm/pwmchip0", 0),
		rear("/sys/devices/platform/ocp/ocp:P9_21_pinmux", "/sys/class/pwm/pwmchip0", 1)
{
}

MyMotorsAgent::~MyMotorsAgent() {
}

void MyMotorsAgent::initialize() {
	front.initialize();
	rear.initialize();
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
				this->writeMotors(outMotors);
			} else if (event->getType()
					== MyEvent::EventType::DisarmMotorsCmd) {
				disarmMotors();
			}
		} else {
			if (event->getType() == MyEvent::EventType::ArmMotorsCmd) {
				armMotors();
			} else 			if (event->getType() == MyEvent::EventType::OutMotors) {
				boost::shared_ptr<MyOutMotors> outMotors =
						boost::static_pointer_cast<MyOutMotors>(event);
//				cout << "Disarmed: MF: " << outMotors->getFront() << ", MR: " << outMotors->getRear() << ", ML: " << outMotors->getLeft() << ", MR: " << outMotors->getRight() << endl;
			}
		}
	}
}

void MyMotorsAgent::writeMotors(boost::shared_ptr<MyOutMotors> event) {
//	cout << "Armed: MF: " << event->getFront() << ", MR: " << event->getRear() << ", ML: " << event->getLeft() << ", MR: " << event->getRight() << endl;
	front.setDutyCycleNanos(event->getFront());
	rear.setDutyCycleNanos(event->getRear());
}
void MyMotorsAgent::disarmMotors() {
	front.setDutyCycleNanos(1000000);
	rear.setDutyCycleNanos(1000000);

	// TODO: disarm motors
	armed = false;
	boost::shared_ptr<MyMotorsDisarmed> evOut(
			boost::make_shared<MyMotorsDisarmed>(this->getUuid()));
	m_signal(evOut);
}
void MyMotorsAgent::armMotors() {
	front.setDutyCycleNanos(1000000);
	rear.setDutyCycleNanos(1000000);

	armed = true;
	boost::shared_ptr<MyMotorsArmed> evOut(
			boost::make_shared<MyMotorsArmed>(this->getUuid()));
	m_signal(evOut);
}
