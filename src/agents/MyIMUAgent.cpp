/*
 * MyIMUAgent.cpp
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#include <agents/MyIMUAgent.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/log/trivial.hpp>
#include <boost/math/quaternion.hpp>
#include <events/MyIMUSample.h>
#include <events/MyBaroSample.h>
#include <syslog.h>

MyIMUAgent::MyIMUAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes) : MyAgent(bus, acceptedEventTypes), initialized(false), lastTickMicros(0), lastDTimeMicros(0) {
}

MyIMUAgent::~MyIMUAgent() {

}
bool MyIMUAgent::initialize() {
	if(imu.dmpBegin()) {
		initialized = true;
	}
	return initialized;
}
void MyIMUAgent::calcTickTimestamp() {
    uint32_t now = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
    lastDTimeMicros = uint16_t(now - lastTickMicros);
    lastTickMicros = now;
}
void MyIMUAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if(!initialized) {
		initialize();
	}
	if(this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::Tick) {
		    this->calcTickTimestamp();
			if(imu.pulse()) {
				const MPU6050::SensorData& md = imu.getData();
//				syslog(LOG_INFO, "YPR: y(%3.2f), p(%3.2f), r(%3.2f) - Accel: x(%d), y(%d), z(%d)", md.ypr[0], md.ypr[1], md.ypr[2], md.accel.x, md.accel.y, md.accel.z);

				boost::math::quaternion<float> q(md.q.w,md.q.x,md.q.y,md.q.z);
				boost::shared_ptr<MyEvent> evOut(boost::make_shared<MyIMUSample>(this->getUuid(), q, md.ypr[0]*57.324840764f, md.ypr[1]*57.324840764f, md.ypr[2]*57.324840764f, md.gravity, md.accel, md.linearAccel));
				m_signal(evOut);
			}
			if(bmp.pulse()) {
				const BMP085::SensorData& md = bmp.getData();
				boost::shared_ptr<MyBaroSample> evOut(boost::make_shared<MyBaroSample>(this->getUuid(), md.altitude, md.estimatedAltitude, md.pressure, md.seaLevelPressure, md.temperature, md.rawPressure, md.rawTemperature, md.dtimeMillis));
				m_signal(evOut);
			}
		}
	}
}
