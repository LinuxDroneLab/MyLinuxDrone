/*
 * MyPIDControllerAgent.cpp
 *
 *  Created on: 26 dic 2015
 *      Author: andrea
 */
#include <cmath>

#include <agents/MyPIDControllerAgent.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/log/trivial.hpp>
#include <events/MyIMUSample.h>
#include <events/MyBaroSample.h>
#include <events/MyOutMotors.h>
#include <events/MyYPRError.h>
#include <events/MyRCSample.h>
#include <iostream>

#define ROLL_POS 0
#define PITCH_POS 1
#define YAW_POS 2
#define THRUST_POS 3


RangeInt16 MyPIDControllerAgent::TARGET_RANGES[] = {RangeInt16(-62, 62), // roll
		                                            RangeInt16(-62, 62), // pitch
		                                            RangeInt16(-125, 125), // yaw
		                                            RangeInt16(1000, 2000) // thrust
};
ValueInt16 MyPIDControllerAgent::TARGET_VALUES[] = {ValueInt16(0, MyPIDControllerAgent::TARGET_RANGES[ROLL_POS]),
													ValueInt16(0, MyPIDControllerAgent::TARGET_RANGES[PITCH_POS]),
													ValueInt16(0, MyPIDControllerAgent::TARGET_RANGES[YAW_POS]),
													ValueInt16(0, MyPIDControllerAgent::TARGET_RANGES[THRUST_POS]),
};

using namespace std;

MyPIDControllerAgent::MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), yawCurr(0), pitchCurr(0), rollCurr(0), yawErr(1.0f, 4, 10, 3), pitchErr(
				1.0f, 4, 10, 3), rollErr(1.0f, 4, 10, 3) {
	keRoll = 1.587f;
	keIRoll = 0.0186f;
	keDRoll = 52.00f;

	kePitch = 1.587f; // old 1.321f
	keIPitch = 0.0186f; // old 0.0060f
	keDPitch = 52.00f; // old 30.00f

	keYaw = 1.687f;
	keIYaw = 0.156f;
	keDYaw = 65.000f;

}

MyPIDControllerAgent::~MyPIDControllerAgent() {
}
void MyPIDControllerAgent::initialize() {
	initialized = true;
}

void MyPIDControllerAgent::calcErr(boost::math::quaternion<float> q) {
	float real = q.real();
	float x = q.R_component_2();
	float y = q.R_component_3();
	float z = q.R_component_4();


	yawCurr = std::rint((atan2(2.0f * (real * z + x * y), 1.0f - 2.0f * (y * y + z * z))
			* 57.295779513f));
	pitchCurr = std::rint((asin(2.0f * real * y - 2.0 * z * x) * 57.295779513f));
	rollCurr = std::rint((atan2(2.0f * (real * x + y * z), 1.0f - 2.0f * (x * x + y * y))
			* 57.295779513f));

//	yawCurr = uint16_t(std::rint(0.8f*float(yawCurr) + 0.2f*float(_yawCurr)));
//	pitchCurr = uint16_t(std::rint(0.8f*float(pitchCurr) + 0.2f*float(_pitchCurr)));
//	rollCurr = uint16_t(std::rint(0.8f*float(rollCurr) + 0.2f*float(_rollCurr)));

	yawErr.push(std::min(60.0f, std::max(-60.0f, float(yawCurr - MyPIDControllerAgent::TARGET_VALUES[YAW_POS].getValue()))));
	pitchErr.push(std::min(30.0f, std::max(-30.0f, float(pitchCurr - MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].getValue()))));
	rollErr.push(std::min(30.0f, std::max(-30.0f, float(rollCurr - MyPIDControllerAgent::TARGET_VALUES[ROLL_POS].getValue()))));
}
void MyPIDControllerAgent::calcCorrection() {
//	float eRoll = std::min(10.0f, std::max(-10.0f, rollErr.getMean()));
	float eRoll = rollErr.getMean();
	float eIRoll = rollErr.getIntegral();
	float eDRoll = rollErr.getDerivate();

//	float ePitch = std::min(10.0f, std::max(-10.0f, pitchErr.getMean()));
	float ePitch = pitchErr.getMean();
	float eIPitch = pitchErr.getIntegral();
	float eDPitch = pitchErr.getDerivate();

	float eYaw = yawErr.getMean();
	float eIYaw = yawErr.getIntegral();
	float eDYaw = yawErr.getDerivate();

	float corrRoll = keRoll * eRoll + min(60.0f, max(-60.0f, keIRoll*eIRoll)) + keDRoll * eDRoll;
	float corrPitch = kePitch * ePitch + min(60.0f, max(-60.0f, keIPitch*eIPitch)) + keDPitch * eDPitch;
	float corrYaw = keYaw * eYaw + min(20.0f, max(-20.0f, keIYaw*eIYaw)) + keDYaw * eDYaw;

	int32_t front = std::rint((float(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) + corrPitch - corrYaw)*1000.0f);
	int32_t rear = std::rint((float(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) - corrPitch - corrYaw)*1000.0f);
	int32_t left = std::rint((float(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) - corrRoll + corrYaw)*1000.0f);
	int32_t right = std::rint((float(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) + corrRoll + corrYaw)*1000.0f);

//	printf("corrRoll:%5.5f, corrPitch:%5.5f, corrYaw:%5.5f, keIR/P=%2.2f, keDR/P=%2.2f, eRoll=%2.2f, ePitch=%2.2f, eIRoll=%2.2f, eIPitch=%2.2f, eDRoll=%2.2f, eDPitch=%2.2f \n",
//			corrRoll, corrPitch, corrYaw, keIRoll, keDRoll, eRoll, ePitch, eIRoll, eIPitch, eDRoll, eDPitch);

	// level motors on [1000000, 2000000]
	int32_t minMotors = 9999999;
	if(front < minMotors) {
		minMotors = front;
	}
	if(rear < minMotors) {
		minMotors = rear;
	}
	if(left < minMotors) {
		minMotors = left;
	}
	if(right < minMotors) {
		minMotors = right;
	}
	int32_t diff = 0;
	if(minMotors < 1000000) {
		diff = 1000000 - minMotors;
	}
	front = std::min(2000000, front + diff);
	rear = std::min(2000000, rear + diff);
	left = std::min(2000000, left + diff);
	right = std::min(2000000, right + diff);

	int32_t maxMotors = 0;
	if(front > maxMotors) {
		maxMotors = front;
	}
	if(rear > maxMotors) {
		maxMotors = rear;
	}
	if(left > maxMotors) {
		maxMotors = left;
	}
	if(right > maxMotors) {
		maxMotors = right;
	}
	diff = 0;
	if(maxMotors > 2000000) {
		diff = maxMotors - 2000000;
	}
	front = std::max(1000000, front - diff);
	rear = std::max(1000000, rear - diff);
	left = std::max(1000000, left - diff);
	right = std::max(1000000, right - diff);

	// printf("%6.3f, %6.3f, %6.3f, %d, %6.3f, %d, %d \n", ePitch, eIPitch, eDPitch, front, corrPitch, pitchCurr, MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].getValue());

	{ // out error event
		boost::shared_ptr<MyYPRError> evOut(boost::make_shared<MyYPRError>(this->getUuid(), yawCurr, pitchCurr, rollCurr, MyPIDControllerAgent::TARGET_VALUES[YAW_POS].getValue(), MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].getValue(), MyPIDControllerAgent::TARGET_VALUES[ROLL_POS].getValue(), keRoll*eRoll, keIRoll*eIRoll, keDRoll*eDRoll,
				kePitch*ePitch, keIPitch*eIPitch, keDPitch*eDPitch,
				keYaw*eYaw, keIYaw*eIYaw, keDYaw*eDYaw));
		m_signal(evOut);
	}
	{// out state motors
		boost::shared_ptr<MyOutMotors> evOut(boost::make_shared<MyOutMotors>(this->getUuid(), front, rear, left, right));
		m_signal(evOut);
	}
}
void MyPIDControllerAgent::processEvent(boost::shared_ptr<MyEvent> event) {
	if (!initialized) {
		initialize();
	}
	if (this->getState() == MyAgentState::Active) {
		if (event->getType() == MyEvent::EventType::IMUSample) {
			boost::shared_ptr<MyIMUSample> imuSample =
					boost::static_pointer_cast<MyIMUSample>(event);

			boost::math::quaternion<float> q = imuSample->getQuaternion();
			calcErr(q);
			calcCorrection();
		} else if(event->getType() == MyEvent::EventType::RCSample) {
			boost::shared_ptr<MyRCSample> rcSample =
					boost::static_pointer_cast<MyRCSample>(event);
//			keIRoll = keIPitch = (1.0f + (*rcSample).getAux1Percent());
//			keDRoll = keDPitch = (1.0f + (*rcSample).getAux2Percent());

			MyPIDControllerAgent::TARGET_VALUES[ROLL_POS].setPercentValue((*rcSample).getRollPercent());
			MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].setPercentValue((*rcSample).getPitchPercent());
			MyPIDControllerAgent::TARGET_VALUES[YAW_POS].setPercentValue((*rcSample).getYawPercent());
			MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].setPercentValue((*rcSample).getThrustPercent());
//			cout << "PitchSample: " << (*rcSample).getPitchPercent() << endl;
		}
	}
}
