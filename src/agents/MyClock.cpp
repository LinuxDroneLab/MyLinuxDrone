/*
 * MyClock.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <agents/MyClock.h>
#include <events/MyTick.h>

MyClock::MyClock(boost::shared_ptr<MyEventBus> bus, uint m_waitMillis) :
		MyAgent(bus, {}), m_shutdown(false), m_waitMillis(m_waitMillis) {
}

MyClock::~MyClock() {
	// TODO Auto-generated destructor stub
}

void MyClock::shutdown() {
	m_shutdown = true;
}

void MyClock::operator()() {
	while (!m_shutdown) {
		boost::this_thread::sleep(boost::posix_time::microseconds(m_waitMillis*1000L));
    	boost::shared_ptr<MyEvent> evOut(boost::make_shared <MyTick>(uuid));
    	m_signal(evOut);
	}
}
