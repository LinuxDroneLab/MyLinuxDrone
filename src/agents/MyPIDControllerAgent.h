/*
 * MyPIDControllerAgent.h
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYPIDCONTROLLERAGENT_H_
#define AGENTS_MYPIDCONTROLLERAGENT_H_

#include <agents/MyAgent.h>
#include <boost/math/quaternion.hpp>
#include <queues/MyPIDBuffer.h>
#include <commons/RangeInt16.h>
#include <commons/ValueInt16.h>

class MyPIDControllerAgent: public MyAgent {
public:
	MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyPIDControllerAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	bool initialized;
	void initialize();
	void calcErr(boost::math::quaternion<float> q);
	void calcCorrection();

	int16_t yawCurr;
	int16_t pitchCurr;
	int16_t rollCurr;

	MyPIDBuffer yawErr;
	MyPIDBuffer pitchErr;
	MyPIDBuffer rollErr;

	// PID parameters
	float keRoll;
	float keIRoll;
	float keDRoll;

	float kePitch;
	float keIPitch;
	float keDPitch;

	float keYaw;
	float keIYaw;
	float keDYaw;

	static RangeInt16 TARGET_RANGES[];
	static ValueInt16 TARGET_VALUES[];
};

#endif /* AGENTS_MYPIDCONTROLLERAGENT_H_ */
