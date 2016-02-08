/*
 * MyEventQueue.h
 *
 *  Created on: 14 dic 2015
 *      Author: andrea
 */

#ifndef MYEVENTQUEUE_H_
#define MYEVENTQUEUE_H_
#include <boost/shared_ptr.hpp>
#include <boost/call_traits.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/heap/priority_queue.hpp>
#include "events/MyEvent.h"
#include <events/MyPriorityComparator.h>
#include <boost/thread/condition.hpp>

using namespace std;

class MyEventQueue {
public:
	typedef boost::heap::priority_queue<boost::shared_ptr<MyEvent>, boost::heap::compare<MyPriorityComparator> > EventQueueType;
	typedef typename EventQueueType::value_type value_type;
	typedef typename EventQueueType::size_type size_type;
	typedef typename boost::call_traits<value_type>::param_type param_type;

	MyEventQueue();
	virtual ~MyEventQueue();
	void push(param_type event);
	value_type pop();
	size_type size();
	size_type maxSize();
	bool isNotEmpty() const {
		return eventQueue.size() > 0;
	}
private:
	EventQueueType eventQueue;
	boost::mutex m_mutex;
	boost::condition m_not_empty;

};

#endif /* MYEVENTQUEUE_H_ */
