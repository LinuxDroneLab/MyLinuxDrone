/*
 * MyClock.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYCLOCK_H_
#define AGENTS_MYCLOCK_H_

#include <agents/MyAgent.h>
#include <events/MyEvent.h>
#include <bus/MyEventBus.h>

class MyClock : public MyAgent {
public:
	explicit MyClock(boost::shared_ptr<MyEventBus> bus, uint m_waitMillis);
	virtual ~MyClock();
	virtual void operator()();
	virtual void shutdown();
protected:
private:
	bool m_shutdown;
	uint m_waitMillis;
};

#endif /* AGENTS_MYCLOCK_H_ */
