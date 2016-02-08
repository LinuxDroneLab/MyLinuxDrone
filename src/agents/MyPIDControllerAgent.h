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

class MyPIDControllerAgent: public MyAgent {
public:
	MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyPIDControllerAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	bool initialized;
	void initialize();
	boost::math::quaternion<float> qtrg; // target quaternion
	unsigned int thrust; // thrust target
	float yawCurr;
	float pitchCurr;
	float rollCurr;
	float yawTrg;
	float pitchTrg;
	float rollTrg;
	void mockTarget(boost::math::quaternion<float> q);
	void calcErr(boost::math::quaternion<float> q);
	void calcCorrection();
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
};

#endif /* AGENTS_MYPIDCONTROLLERAGENT_H_ */
