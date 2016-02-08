/*
 * BoundedBuffer.h
 *
 *  Created on: 13 dic 2015
 *      Author: andrea
 */

#ifndef MYBOUNDEDBUFFER_H_
#define MYBOUNDEDBUFFER_H_

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

template<class T>
class MyBoundedBuffer {
public:
	typedef boost::circular_buffer<T> container_type;
	typedef typename container_type::size_type size_type;
	typedef typename container_type::value_type value_type;
	typedef typename boost::call_traits<value_type>::param_type param_type;

	explicit MyBoundedBuffer(size_type capacity) :
			m_container(capacity) {
	}

	void push_front(param_type item) {
		// `param_type` represents the "best" way to pass a parameter of type `value_type` to a method.

		boost::mutex::scoped_lock lock(m_mutex);
		m_not_full.wait(lock,
				boost::bind(&MyBoundedBuffer<value_type>::is_not_full, this));
		m_container.push_front(item);
		lock.unlock();
		m_not_empty.notify_one();
	}

	value_type pop_back() {
		boost::mutex::scoped_lock lock(m_mutex);
		m_not_empty.wait(lock,
				boost::bind(&MyBoundedBuffer<value_type>::is_not_empty, this));
		value_type pItem = m_container.back();
		m_container.pop_back();
		lock.unlock();
		m_not_full.notify_one();
		return pItem;
	}
	size_type size() {
		return m_container.size();
	}
private:
	MyBoundedBuffer(const MyBoundedBuffer&);           // Disabled copy constructor.
	MyBoundedBuffer& operator =(const MyBoundedBuffer&); // Disabled assign operator.

	bool is_not_empty() const {
		return m_container.size() > 0;
	}
	bool is_not_full() const {
		return m_container.size() < m_container.capacity();
	}

	container_type m_container;
	boost::mutex m_mutex;
	boost::condition m_not_empty;
	boost::condition m_not_full;
};

#endif /* MYBOUNDEDBUFFER_H_ */
