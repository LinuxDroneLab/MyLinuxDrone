/*
 * MyPIDControllerAgent.h
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYPIDCONTROLLERAGENT_H_
#define AGENTS_MYPIDCONTROLLERAGENT_H_

#include <agents/MyAgent.h>
#include <commons/MyGlobalDefs.h>
#include <boost/math/quaternion.hpp>
#include <queues/MyPIDBuffer.h>
#include <commons/RangeFloat.h>
#include <commons/ValueFloat.h>

class MyPIDControllerAgent: public MyAgent {
public:
	MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyPIDControllerAgent();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	bool initialized;
	void initialize();
	bool armed;
	void calcErr(boost::math::quaternion<float> q);
	void calcCorrection();

	float yawCurr;
	float pitchCurr;
	float rollCurr;

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

	static RangeFloat TARGET_RANGES[];
	static ValueFloat TARGET_VALUES[];
    static RangeFloat INTEGRAL_RANGE;
};

#endif /* AGENTS_MYPIDCONTROLLERAGENT_H_ */
