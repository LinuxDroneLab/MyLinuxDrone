/*
 * MyEventQueue.cpp
 *
 *  Created on: 14 dic 2015
 *      Author: andrea
 */

#include "MyEventQueue.h"
#include <boost/bind.hpp>

MyEventQueue::MyEventQueue() {
	// TODO Auto-generated constructor stub

}

MyEventQueue::~MyEventQueue() {
	// TODO Auto-generated destructor stub
}

MyEventQueue::size_type MyEventQueue::size() {
	return eventQueue.size();
}
MyEventQueue::size_type MyEventQueue::maxSize() {
	return eventQueue.max_size();
}
void MyEventQueue::push(MyEventQueue::param_type event) {
	boost::mutex::scoped_lock lock(m_mutex);
	eventQueue.push(event);
	lock.unlock();
	m_not_empty.notify_one();

}
MyEventQueue::value_type MyEventQueue::pop() {
	boost::mutex::scoped_lock lock(m_mutex);
	m_not_empty.wait(lock,
			boost::bind(&MyEventQueue::isNotEmpty, this));
	value_type result = eventQueue.top();
		eventQueue.pop();
	lock.unlock();
	return result;
}
