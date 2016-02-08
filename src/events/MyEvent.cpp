/*
 * MyEvent.cpp
 *
 *  Created on: 12 dic 2015
 *      Author: andrea
 */

#include "events/MyEvent.h"
#include "commons/MyUuid.h"
#include <boost/log/trivial.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <iostream>
MyEvent::MyEvent(boost::uuids::uuid origin) :
		uuid(MyUuid::generateUuid()), timestamp(
				boost::posix_time::microsec_clock::local_time()), origin(
				origin), priority(100), livingTimeMillis(10) {
}

MyEvent::~MyEvent() {
//	BOOST_LOG_TRIVIAL(info)<< "destroy event: " << uuid;
//	cout << "destroy event: " << uuid << endl;
}

std::ostream& operator <<(std::ostream& os, const MyEvent::EventType& obj) {
	os << static_cast<std::underlying_type<MyEvent::EventType>::type>(obj);
	return os;
}
ostream& operator<<(ostream& ostr, const MyEvent& event) {
	ostr << "{uuid:\"" << event.uuid << "\", type:\"" << event.getType()
			<< "\", ts:\"" << event.timestamp << "\", origin:\"" << event.origin
			<< "\", priority:\"" << event.priority << "\"}";
	return ostr;
}

istream& operator>>(istream& istr, const MyEvent& event) {
	// TODO: implements
	return istr;
}

boost::uuids::uuid MyEvent::getUuid() const {
	return uuid;
}
boost::uuids::uuid MyEvent::getOrigin() {
	return origin;
}

boost::posix_time::ptime MyEvent::getTimestamp() const {
	return timestamp;
}
long MyEvent::getTimestampMillis() const {
	return timestamp.time_of_day().total_milliseconds();
}

uint MyEvent::getPriority() {
	return priority;
}
void MyEvent::setPriority(uint priority) {
	this->priority = priority;
}

uint MyEvent::getLivingTimeMillis() const {
	return livingTimeMillis;
}
void MyEvent::setLivingTimeMillis(uint livingTimeMillis) {
	this->livingTimeMillis = livingTimeMillis;
}

bool MyEvent::isExpired() const {
	long now = boost::posix_time::microsec_clock::local_time().time_of_day().total_milliseconds();
	return (now - this->getTimestampMillis()) > this->livingTimeMillis;
}
bool MyEvent::isCommand() const {
	return false;
}
