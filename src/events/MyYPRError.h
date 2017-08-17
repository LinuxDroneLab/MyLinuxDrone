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
	MyYPRError(boost::uuids::uuid origin, float yawCurr,float pitchCurr,float rollCurr, float yawTrg,float pitchTrg,float rollTrg,float eRoll, float eIRoll, float eDRoll, float ePitch, float eIPitch, float eDPitch, float eYaw, float eIYaw, float eDYaw);
	virtual ~MyYPRError();
	virtual MyEvent::EventType getType() const;

	float getYawCurr() const;
	float getPitchCurr() const;
	float getRollCurr() const;
	float getYawTrg() const;
	float getPitchTrg() const;
	float getRollTrg() const;
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
	const float yawCurr;
	const float pitchCurr;
	const float rollCurr;

	const float yawTrg;
	const float pitchTrg;
	const float rollTrg;

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
