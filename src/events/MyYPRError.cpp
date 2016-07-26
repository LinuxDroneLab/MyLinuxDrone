/*
 * MyYPRError.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyYPRError.h>

MyYPRError::MyYPRError(boost::uuids::uuid origin, int16_t yawCurr,int16_t pitchCurr,int16_t rollCurr, int16_t yawTrg,int16_t pitchTrg,int16_t rollTrg, float eRoll, float eIRoll,
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
int16_t MyYPRError::getYawCurr() const {
	return yawCurr;
}
int16_t MyYPRError::getPitchCurr() const {
	return pitchCurr;
}
int16_t MyYPRError::getRollCurr() const {
	return rollCurr;
}

int16_t MyYPRError::getYawTrg() const {
	return yawTrg;
}
int16_t MyYPRError::getPitchTrg() const {
	return pitchTrg;
}
int16_t MyYPRError::getRollTrg() const {
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
