/*
 * MyYPRError.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyYPRError.h>

MyYPRError::MyYPRError(boost::uuids::uuid origin, float yawCurr,float pitchCurr,float rollCurr, float yawTrg,float pitchTrg,float rollTrg, float eRoll, float eIRoll,
		float eDRoll, float ePitch, float eIPitch, float eDPitch, float eYaw,
		float eIYaw, float eDYaw) :
		MyEvent(origin), yawCurr(yawCurr), pitchCurr(pitchCurr), rollCurr(rollCurr), yawTrg(yawTrg), pitchTrg(pitchTrg), rollTrg(rollTrg), eRoll(eRoll), eIRoll(eIRoll), eDRoll(eDRoll),
		ePitch(ePitch), eIPitch(eIPitch), eDPitch(eDPitch),
		eYaw(eYaw), eIYaw(eIYaw), eDYaw(eDYaw) {
	this->setPriority(MyPriority::YPR_ERROR_PRIORITY);

}

MyYPRError::~MyYPRError() {
}
MyEvent::EventType MyYPRError::getType() const {
	return MyEvent::EventType::YPRError;
}
float MyYPRError::getYawCurr() const {
	return yawCurr;
}
float MyYPRError::getPitchCurr() const {
	return pitchCurr;
}
float MyYPRError::getRollCurr() const {
	return rollCurr;
}

float MyYPRError::getYawTrg() const {
	return yawTrg;
}
float MyYPRError::getPitchTrg() const {
	return pitchTrg;
}
float MyYPRError::getRollTrg() const {
	return rollTrg;
}
float MyYPRError::getERoll() const {
	return eRoll;
}
float MyYPRError::getEIRoll() const {
	return eIRoll;
}
float MyYPRError::getEDRoll() const {
	return eDRoll;
}

float MyYPRError::getEPitch() const {
	return ePitch;
}
float MyYPRError::getEIPitch() const {
	return eIPitch;
}
float MyYPRError::getEDPitch() const {
	return eDPitch;
}

float MyYPRError::getEYaw() const {
	return eYaw;
}
float MyYPRError::getEIYaw() const {
	return eIYaw;
}
float MyYPRError::getEDYaw() const {
	return eDYaw;
}
