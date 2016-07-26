/*
 * MyRCSample.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYRCSAMPLE_H_
#define EVENTS_MYRCSAMPLE_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyRCSample: public MyEvent {
public:
	explicit MyRCSample(boost::uuids::uuid origin, float thrustPercent, float rollPercent, float pitchPercent, float yawPercent, float aux1Percent, float aux2Percent);
	virtual ~MyRCSample();
	virtual MyEvent::EventType getType() const;

	float getThrustPercent();
	float getRollPercent();
	float getPitchPercent();
	float getYawPercent();
	float getAux1Percent();
	float getAux2Percent();
private:
	float thrustPercent;
	float rollPercent;
	float pitchPercent;
	float yawPercent;
	float aux1Percent;
	float aux2Percent;
};

#endif /* EVENTS_MYRCSAMPLE_H_ */
