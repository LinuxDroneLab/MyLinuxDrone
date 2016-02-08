/*
 * MyIMUSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyIMUSample.h>

MyIMUSample::MyIMUSample(boost::uuids::uuid origin, boost::math::quaternion<float> q) : MyEvent(origin), q(q) {
	this->setPriority(MyPriority::IMU_SAMPLE_PRIORITY);
}

MyIMUSample::~MyIMUSample() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyIMUSample::getType() const {
	return MyEvent::EventType::IMUSample;
}
boost::math::quaternion<float> MyIMUSample::getQuaternion() const {
	return q;
}
