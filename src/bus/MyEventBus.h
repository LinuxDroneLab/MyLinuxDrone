/*
 * MyEventBus.h
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#ifndef BUS_MYEVENTBUS_H_
#define BUS_MYEVENTBUS_H_
#include <boost/signals2/signal.hpp>
#include <boost/shared_ptr.hpp>
#include <events/MyEvent.h>

class MyEventBus {
public:
	MyEventBus();
	virtual ~MyEventBus();
	typedef boost::signals2::signal<void (boost::shared_ptr<MyEvent>)> SignalType;
	boost::signals2::connection connect(const SignalType::slot_type &subscriber);
	void doEvent(boost::shared_ptr<MyEvent> event);
protected:
	SignalType m_signal;

};

#endif /* BUS_MYEVENTBUS_H_ */
