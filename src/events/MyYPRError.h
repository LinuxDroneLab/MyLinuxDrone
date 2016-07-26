/*
 * MyYPRError.h
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYYPRERROR_H_
#define EVENTS_MYYPRERROR_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyYPRError: public MyEvent {
public:
	MyYPRError(boost::uuids::uuid origin, int16_t yawCurr,int16_t pitchCurr,int16_t rollCurr, int16_t yawTrg,int16_t pitchTrg,int16_t rollTrg,float eRoll, float eIRoll, float eDRoll, float ePitch, float eIPitch, float eDPitch, float eYaw, float eIYaw, float eDYaw);
	virtual ~MyYPRError();
	virtual MyEvent::EventType getType() const;

	int16_t getYawCurr() const;
	int16_t getPitchCurr() const;
	int16_t getRollCurr() const;
	int16_t getYawTrg() const;
	int16_t getPitchTrg() const;
	int16_t getRollTrg() const;
	float getERoll() const;
	float getEIRoll() const;
	float getEDRoll() const;
	float getEPitch() const;
	float getEIPitch() const;
	float getEDPitch() const;
	float getEYaw() const;
	float getEIYaw() const;
	float getEDYaw() const;
private:
	const int16_t yawCurr;
	const int16_t pitchCurr;
	const int16_t rollCurr;

	const int16_t yawTrg;
	const int16_t pitchTrg;
	const int16_t rollTrg;

	const float eRoll;
	const float eIRoll;
	const float eDRoll;

	const float ePitch;
	const float eIPitch;
	const float eDPitch;

	const float eYaw;
	const float eIYaw;
	const float eDYaw;

};

#endif /* EVENTS_MYYPRERROR_H_ */
