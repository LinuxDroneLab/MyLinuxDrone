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
#include <events/MyMotorsDisarmed.h>
#include <events/MyMotorsArmed.h>
#include <iostream>
#include <syslog.h>

#define ROLL_POS 0
#define PITCH_POS 1
#define YAW_POS 2
#define THRUST_POS 3


RangeFloat MyPIDControllerAgent::TARGET_RANGES[] = {RangeFloat(-62.0f, 62.0f), // roll
		                                            RangeFloat(-62.0f, 62.0f), // pitch
		                                            RangeFloat(-125.0f, 125.0f), // yaw
		                                            RangeFloat(1000.0f, 2000.0f) // thrust
};
ValueFloat MyPIDControllerAgent::TARGET_VALUES[] = {ValueFloat(0.0f, MyPIDControllerAgent::TARGET_RANGES[ROLL_POS]),
													ValueFloat(0.0f, MyPIDControllerAgent::TARGET_RANGES[PITCH_POS]),
													ValueFloat(0.0f, MyPIDControllerAgent::TARGET_RANGES[YAW_POS]),
													ValueFloat(0.0f, MyPIDControllerAgent::TARGET_RANGES[THRUST_POS])
};

using namespace std;

MyPIDControllerAgent::MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), armed(false), yawCurr(0.0f), pitchCurr(0.0f), rollCurr(0.0f), yawErr(1.0f, 4, 10, 3), pitchErr(
				1.0f, 4, 10, 3), rollErr(1.0f, 4, 10, 3) {
	keRoll = 8.55f;      // local tests: 1.587f
	keIRoll = 0.0073f; //0.0353f; //= 1.5186f;    // local tests: 0.0186f
	keDRoll = 90.0125f; //112.0186f; //= 62.00f;     // local tests: 52.00f

	kePitch = 8.55f;     // local tests: 1.587f
	keIPitch = 0.0073f; //0.0353f; //= 1.5186f;   // local tests: 0.0186f
	keDPitch = 90.0125f; //112.0186f; //= 62.00f;    // local tests: 52.00f

	keYaw = 1.687f;       // local tests:  1.687f
	keIYaw = 0.0f; //= 0.056f;      // local tests:  0.156f
	keDYaw = 0.0f; //= 10.000f;     // local tests:  65.00f

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


	yawCurr = atan2(2.0f * (real * z + x * y), 1.0f - 2.0f * (y * y + z * z))
			* 57.295779513f;
	pitchCurr = asin(2.0f * real * y - 2.0 * z * x) * 57.295779513f;
	rollCurr = atan2(2.0f * (real * x + y * z), 1.0f - 2.0f * (x * x + y * y))
			* 57.295779513f;

//	yawCurr = uint16_t(std::rint(0.8f*float(yawCurr) + 0.2f*float(_yawCurr)));
//	pitchCurr = uint16_t(std::rint(0.8f*float(pitchCurr) + 0.2f*float(_pitchCurr)));
//	rollCurr = uint16_t(std::rint(0.8f*float(rollCurr) + 0.2f*float(_rollCurr)));

	// considero errore limitato a 10 deg. more less for yaw an 60deg for pitch and roll
	yawErr.push(std::min(10.0f, std::max(-10.0f, yawCurr - MyPIDControllerAgent::TARGET_VALUES[YAW_POS].getValue())));
	pitchErr.push(std::min(60.0f, std::max(-60.0f, pitchCurr - MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].getValue())));
	rollErr.push(std::min(60.0f, std::max(-60.0f, rollCurr - MyPIDControllerAgent::TARGET_VALUES[ROLL_POS].getValue())));
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

	// integrale limitato
//	float corrRoll = keRoll * eRoll + min(50.0f, max(-50.0f, keIRoll*eIRoll)) + keDRoll * eDRoll;
//	float corrPitch = kePitch * ePitch + min(50.0f, max(-50.0f, keIPitch*eIPitch)) + keDPitch * eDPitch;
//	float corrYaw = keYaw * eYaw + min(10.0f, max(-10.0f, keIYaw*eIYaw)) + keDYaw * eDYaw;

	float corrRoll = keRoll * eRoll + keIRoll*eIRoll + keDRoll * eDRoll;
	float corrPitch = kePitch * ePitch + keIPitch*eIPitch + keDPitch * eDPitch;
	float corrYaw = keYaw * eYaw + min(10.0f, max(-10.0f, keIYaw*eIYaw)) + keDYaw * eDYaw;

	int32_t front = std::lrint(std::max(double(1000000), std::min(double(2000000), double(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) + corrPitch - corrYaw)*1000.0));
	int32_t rear = std::lrint(std::max(double(1000000), std::min(double(2000000), double(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) - corrPitch - corrYaw)*1000.0));
	int32_t left = std::lrint(std::max(double(1000000), std::min(double(2000000), double(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) - corrRoll + corrYaw)*1000.0));
	int32_t right = std::lrint(std::max(double(1000000), std::min(double(2000000), double(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()) + corrRoll + corrYaw)*1000.0));

//    syslog(LOG_INFO, "mydrone Thrust: %f", double(MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].getValue()));

//	syslog(LOG_INFO, "errRoll: %5.5f, eRoll:%5.5f, eIRoll: %5.5f, eDRoll: %5.5f, totCorrRoll:%5.5f. left: %d, right: %d \n",
//			eRoll, keRoll * eRoll, keIRoll*eIRoll, keDRoll * eDRoll, corrRoll, left, right);

	// level motors on [1000000, 2000000]
//	int32_t minMotors = 9999999;
//	if(front < minMotors) {
//		minMotors = front;
//	}
//	if(rear < minMotors) {
//		minMotors = rear;
//	}
//	if(left < minMotors) {
//		minMotors = left;
//	}
//	if(right < minMotors) {
//		minMotors = right;
//	}
//	int32_t diff = 0;
//	if(minMotors < 1000000) {
//		diff = 1000000 - minMotors;
//	}
//	front = std::min(2000000, front + diff);
//	rear = std::min(2000000, rear + diff);
//	left = std::min(2000000, left + diff);
//	right = std::min(2000000, right + diff);
//
//	int32_t maxMotors = 0;
//	if(front > maxMotors) {
//		maxMotors = front;
//	}
//	if(rear > maxMotors) {
//		maxMotors = rear;
//	}
//	if(left > maxMotors) {
//		maxMotors = left;
//	}
//	if(right > maxMotors) {
//		maxMotors = right;
//	}
//	diff = 0;
//	if(maxMotors > 2000000) {
//		diff = maxMotors - 2000000;
//	}
//	front = std::max(1000000, front - diff);
//	rear = std::max(1000000, rear - diff);
//	left = std::max(1000000, left - diff);
//	right = std::max(1000000, right - diff);

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
		if(event->getType() == MyEvent::EventType::MotorsDisarmed) {
			syslog(LOG_INFO, "Motors Disarmed");
			this->armed = false;
		} else
		if(event->getType() == MyEvent::EventType::MotorsArmed) {
			syslog(LOG_INFO, "Motors Armed");
			this->armed = true;
		} else
		if (event->getType() == MyEvent::EventType::IMUSample && this->armed) {
			boost::shared_ptr<MyIMUSample> imuSample =
					boost::static_pointer_cast<MyIMUSample>(event);

			boost::math::quaternion<float> q = imuSample->getQuaternion();
			calcErr(q);
			calcCorrection();
		} else if(event->getType() == MyEvent::EventType::RCSample && this->armed) {
			boost::shared_ptr<MyRCSample> rcSample =
					boost::static_pointer_cast<MyRCSample>(event);
			MyPIDControllerAgent::TARGET_VALUES[ROLL_POS].setPercentValue((*rcSample).getRollPercent());
			MyPIDControllerAgent::TARGET_VALUES[PITCH_POS].setPercentValue((*rcSample).getPitchPercent());
			MyPIDControllerAgent::TARGET_VALUES[YAW_POS].setPercentValue((*rcSample).getYawPercent());
			MyPIDControllerAgent::TARGET_VALUES[THRUST_POS].setPercentValue((*rcSample).getThrustPercent());
		} else {
			// skip events
		}
	}
}
