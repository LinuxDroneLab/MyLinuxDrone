/*
 * MyIMUSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyIMUSample.h>

MyIMUSample::MyIMUSample(boost::uuids::uuid origin, boost::math::quaternion<float> q, float yaw, float pitch, float roll, VectorFloat gravity, VectorInt16 accel, VectorInt16 linearAccel) : MyEvent(origin), q(q) {
	this->setPriority(MyPriority::IMU_SAMPLE_PRIORITY);
	this->yaw = yaw;
	this->pitch = pitch;
	this->roll = roll;
	this->gravity = gravity;
	this->accel = accel;
	this->linearAccel = linearAccel;
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
float MyIMUSample::getYaw() const {
    return yaw;
}
float MyIMUSample::getPitch() const {
    return pitch;
}
float MyIMUSample::getRoll() const {
    return roll;
}
VectorFloat MyIMUSample::getGravity() const {
    return gravity;
}
VectorInt16 MyIMUSample::getAccel() const {
    return accel;
}
VectorInt16 MyIMUSample::getLinearAccel() const {
    return linearAccel;
}
