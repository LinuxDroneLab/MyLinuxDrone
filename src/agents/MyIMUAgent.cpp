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

MyIMUAgent::MyIMUAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes) : MyAgent(bus, acceptedEventTypes), initialized(false), prevMicros(0) {
}

MyIMUAgent::~MyIMUAgent() {

}
bool MyIMUAgent::initialize() {
	if(imu.dmpBegin()) {
		initialized = true;
	}
	return initialized;
}
void MyIMUAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if(!initialized) {
		initialize();
	}
	if(this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::Tick) {
			//long now = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
			//long diff = now - prevMicros;
			// prevMicros = now;
			if(imu.pulse()) {
				const MPU6050::SensorData& md = imu.getData();
				syslog(LOG_INFO, "YPR: y(%3.2f), p(%3.2f), r(%3.2f) - Accel: x(%d), y(%d), z(%d)", md.ypr[0], md.ypr[1], md.ypr[2], md.accel.x, md.accel.y, md.accel.z);

				boost::math::quaternion<float> q(md.q.w,md.q.x,md.q.y,md.q.z);
				boost::shared_ptr<MyEvent> evOut(boost::make_shared<MyIMUSample>(this->getUuid(), q));
				m_signal(evOut);
				// TODO: creare evento con boost::math::Quaternion
				//BOOST_LOG_TRIVIAL(info) << "micros: " << diff << "Size: " << received.size() << " Q: [" << q << "]";
				//long now1 = boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
				//cout << now1 - now << " - " << diff << " S: " << received.size() << ", Q: " << q << ", N: " << norm(q) << endl;
			}
			if(bmp.pulse()) {
				const BMP085::SensorData& md = bmp.getData();
				boost::shared_ptr<MyBaroSample> evOut(boost::make_shared<MyBaroSample>(this->getUuid(), md.altitude, md.pressure, md.seaLevelPressure, md.temperature));
				m_signal(evOut);
				// cout << "A: " << evOut->getAltitude() << ", P: " << evOut->getPressure() << ", SLP: " << evOut->getSeeLevelPressure() << ", TEMP: " << evOut->getTemperature() << endl;
			}
		}
	}
}
