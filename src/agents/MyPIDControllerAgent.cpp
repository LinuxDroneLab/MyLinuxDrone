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

using namespace std;

MyPIDControllerAgent::MyPIDControllerAgent(boost::shared_ptr<MyEventBus> bus,
		vector<MyEvent::EventType> acceptedEventTypes) :
		MyAgent(bus, acceptedEventTypes), initialized(false), thrust(0), yawTrg(
				0), pitchTrg(0), rollTrg(0), yawErr(0.01f, 20, 100, 4), pitchErr(
				0.01f, 4, 1000, 10), rollErr(0.01f, 4, 1000, 10) {
	keRoll = 1.2f;
	keIRoll = 0.5f;
	keDRoll = 0.3f;

	kePitch = 1.2f;
	keIPitch = 0.5f;
	keDPitch = 0.3f;

	keYaw = 0.0f;
	keIYaw = 0.00f;
	keDYaw = 0.000f;

}

MyPIDControllerAgent::~MyPIDControllerAgent() {
}
void MyPIDControllerAgent::initialize() {
	initialized = true;
}
void MyPIDControllerAgent::mockTarget(boost::math::quaternion<float> q) {

	// TEST: utilizzo come target la prima posizione rilevata
	if (thrust == 0) {
		float real = q.real();
		float x = q.R_component_2();
		float y = q.R_component_3();
		float z = q.R_component_4();

		float yaw = (atan2(2.0f * (real * z + x * y),
				1.0f - 2.0f * (y * y + z * z)) * 57.295779513f);
		float pitch = (asin(2.0 * real * y - 2.0 * z * x) * 57.295779513);
		float roll = (atan2(2.0 * (real * x + y * z),
				1.0 - 2.0 * (x * x + y * y)) * 57.295779513);
		thrust = 1200;
		float cy = cos(yaw * 0.017453293 / 2.0);
		float cp = cos(pitch * 0.017453293 / 2.0);
		float cr = cos(roll * 0.017453293 / 2.0);
		float sy = sin(yaw * 0.017453293 / 2.0);
		float sp = sin(pitch * 0.017453293 / 2.0);
		float sr = sin(roll * 0.017453293 / 2.0);

		qtrg = boost::math::quaternion<float>(cy * cp * cr + sy * sp * sr,
				cy * cp * sr - sy * sp * cr, cy * sp * cr + sy * cp * sr,
				sy * cp * cr - cy * sp * sr);
		yawTrg = yaw;
		pitchTrg = pitch;
		rollTrg = roll;
	}
	// TODO:
	/*
	 * - definire calcError(q) riportandoci i calcoli sopra
	 * - eseguire push su yawErr, pitchErr e rollErr.
	 * - rilevare media, integrale e derivata
	 */
}
void MyPIDControllerAgent::calcErr(boost::math::quaternion<float> q) {
	float real = q.real();
	float x = q.R_component_2();
	float y = q.R_component_3();
	float z = q.R_component_4();

	yawCurr = (atan2(2.0f * (real * z + x * y), 1.0f - 2.0f * (y * y + z * z))
			* 57.295779513f);
	pitchCurr = (asin(2.0 * real * y - 2.0 * z * x) * 57.295779513);
	rollCurr = (atan2(2.0 * (real * x + y * z), 1.0 - 2.0 * (x * x + y * y))
			* 57.295779513);

	yawErr.push(yawCurr - yawTrg);
	pitchErr.push(pitchCurr - pitchTrg);
	rollErr.push(rollCurr - rollTrg);
}
void MyPIDControllerAgent::calcCorrection() {
	float eRoll = rollErr.getMean();
	float eIRoll = rollErr.getIntegral();
	float eDRoll = rollErr.getDerivate();

	float ePitch = pitchErr.getMean();
	float eIPitch = pitchErr.getIntegral();
	float eDPitch = pitchErr.getDerivate();

	float eYaw = yawErr.getMean();
	float eIYaw = yawErr.getIntegral();
	float eDYaw = yawErr.getDerivate();

	float corrRoll = keRoll * eRoll + keIRoll*eIRoll + keDRoll * eDRoll;
	float corrPitch = kePitch * ePitch + keIPitch*eIPitch + keDPitch * eDPitch;
	float corrYaw = keYaw * eYaw + keIYaw*eIYaw + keDYaw * eDYaw;

	int32_t front = std::round((thrust + corrPitch - corrYaw)*1000);
	int32_t rear = std::round((thrust - corrPitch - corrYaw)*1000);
	int32_t left = std::round((thrust - corrRoll + corrYaw)*1000);
	int32_t right = std::round((thrust + corrRoll + corrYaw)*1000);

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

	{ // out error event
		boost::shared_ptr<MyYPRError> evOut(boost::make_shared<MyYPRError>(this->getUuid(), yawCurr, pitchCurr, rollCurr, yawTrg, pitchTrg, rollTrg, eRoll, eIRoll, eDRoll,
				ePitch, eIPitch, eDPitch,
				eYaw, eIYaw, eDYaw));
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

			// FIXME: solo per test. Uso come target il primo sample
			boost::math::quaternion<float> q = imuSample->getQuaternion();
			mockTarget(q);
			calcErr(q);
			calcCorrection();
		}
	}
}
