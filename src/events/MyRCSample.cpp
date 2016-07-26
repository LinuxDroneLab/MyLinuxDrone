/*
 * MyRCSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyRCSample.h>

MyRCSample::MyRCSample(boost::uuids::uuid origin, float thrustPercent, float rollPercent, float pitchPercent, float yawPercent, float aux1Percent, float aux2Percent) : MyEvent(origin){
	this->setPriority(MyPriority::RC_SAMPLE_PRIORITY);
	this->thrustPercent = thrustPercent;
	this->rollPercent = rollPercent;
	this->pitchPercent = pitchPercent;
	this->yawPercent = yawPercent;
	this->aux1Percent = aux1Percent;
	this->aux2Percent = aux2Percent;
}

MyRCSample::~MyRCSample() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyRCSample::getType() const {
	return MyEvent::EventType::RCSample;
}

float MyRCSample::getRollPercent() {
	return this->rollPercent;
}
float MyRCSample::getPitchPercent() {
	return this->pitchPercent;
}
float MyRCSample::getYawPercent() {
	return this->yawPercent;
}
float MyRCSample::getThrustPercent() {
	return this->thrustPercent;
}
float MyRCSample::getAux1Percent() {
	return this->aux1Percent;
}
float MyRCSample::getAux2Percent() {
	return this->aux2Percent;
}
