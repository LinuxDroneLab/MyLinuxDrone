/*
 * MyEventBus.cpp
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#include <bus/MyEventBus.h>

MyEventBus::MyEventBus() {
}

MyEventBus::~MyEventBus() {

}

boost::signals2::connection MyEventBus::connect(
		const SignalType::slot_type &subscriber) {
	return m_signal.connect(subscriber);
}

void MyEventBus::doEvent(boost::shared_ptr<MyEvent> event) {
	m_signal(event); // echo input event
}
