/*
 * MyPIDCntrllr.cpp
 *
 *  Created on: 11 set 2017
 *      Author: andrea
 */

#include <agents/MyPIDCntrllr.h>
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
#include <events/MyBaroSample.h>
#include <iostream>
#include <syslog.h>

RangeFloat MyPIDCntrllr::TARGET_RANGES[] = {RangeFloat(-62.0f, 62.0f), // roll
		                                            RangeFloat(-62.0f, 62.0f), // pitch
		                                            RangeFloat(-125.0f, 125.0f), // yaw
		                                            RangeFloat(1000.0f, 2000.0f) // thrust
};
ValueFloat MyPIDCntrllr::TARGET_VALUES[] = {ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[ROLL_POS]),
													ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[PITCH_POS]),
													ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[YAW_POS]),
													ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[THRUST_POS])
};
int8_t MyPIDCntrllr::QUATERNION_DIRECTION_RPY[] = {-1, -1, 1};
int8_t MyPIDCntrllr::RC_DIRECTION_RPY[] = {1, -1, 1};
float  MyPIDCntrllr::FREQUENCY = 100.0f;

using namespace std;


MyPIDCntrllr::MyPIDCntrllr(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes) :
   MyAgent(bus, acceptedEventTypes),
   initialized(false),
   armed(false),
   yawErr(1.0f, 4, 10, 3),
   pitchErr(1.0f, 4, 10, 3),
   rollErr(1.0f, 4, 10, 3)
{
	keRoll = 0.48f;
	keIRoll = 0.005;
	keDRoll = 2.5f;

	kePitch = 0.48f;
	keIPitch = 0.005f;
	keDPitch = 2.5f;

	keYaw = 0.15f;
	keIYaw = 0.0f;
	keDYaw = 0.0f;

	// TODO: usare parametro diverso per yaw. La rotazione richiede molti più giri
	// modificare di conseguenza la funzione calcOutput
	deg2MicrosFactor = 350.0f;
	deg2MicrosYawFactor = 60.0f;
}

MyPIDCntrllr::~MyPIDCntrllr() {

}

void MyPIDCntrllr::initialize() {
	initialized = true;
}

MyPIDCntrllr::YPRT MyPIDCntrllr::calcYPRData(boost::math::quaternion<float> q) {
	YPRT result = {};
	float real = q.real();
	float x = q.R_component_2();
	float y = q.R_component_3();
	float z = q.R_component_4();

	result.yaw = QUATERNION_DIRECTION_RPY[YAW_POS]*atan2(2.0f * (real * z + x * y), 1.0f - 2.0f * (y * y + z * z))
			* 57.295779513f;
	result.pitch = QUATERNION_DIRECTION_RPY[PITCH_POS]*asin(2.0f * real * y - 2.0 * z * x) * 57.295779513f;
	result.roll = QUATERNION_DIRECTION_RPY[ROLL_POS]*atan2(2.0f * (real * x + y * z), 1.0f - 2.0f * (x * x + y * y))
			* 57.295779513f;
	return result;
}

void MyPIDCntrllr::calcErr(YPRT &yprtReq, YPRT &yprtReal) {
	// considero errore limitato a 10 deg. more less for yaw and 60deg for pitch and roll
	yawErr.push(std::min<float>(10.0f, std::max<float>(-10.0f, yprtReal.yaw - yprtReq.yaw)));
	pitchErr.push(std::min<float>(60.0f, std::max<float>(-60.0f, yprtReal.pitch - yprtReq.pitch)));
	rollErr.push(std::min<float>(60.0f, std::max<float>(-60.0f, yprtReal.roll - yprtReq.roll)));
//	syslog(LOG_INFO, "EYPRT: y(%3.5f, %3.5f), p(%3.5f, %3.5f), r(%3.5f, %3.5f), t(%5.5f)", yprtReal.yaw, yprtReq.yaw, yprtReal.pitch, yprtReq.pitch, yprtReal.roll, yprtReq.roll, yprtReal.thrust);

}

// The YPR input is the distance from real to target data with PID correction.
// the thrust is absolute value
MyPIDCntrllr::YPRT MyPIDCntrllr::calcCorrection(YPRT &yprt) {
	YPRT result = {0.0f, 0.0f, 0.0f, 0.0f};
	float yawCorr = yawErr.getMean()*keYaw + yawErr.getIntegral()*keIYaw + yawErr.getDerivate()*keDYaw;
	float pitchCorr = pitchErr.getMean()*kePitch + pitchErr.getIntegral()*keIPitch + pitchErr.getDerivate()*keDPitch;
	float rollCorr = rollErr.getMean()*keRoll + rollErr.getIntegral()*keIRoll + rollErr.getDerivate()*keDRoll;
	result.yaw = yprt.yaw - yawCorr;
	result.pitch = yprt.pitch - pitchCorr;
	result.roll = yprt.roll - rollCorr;
	result.thrust = yprt.thrust;
//	syslog(LOG_INFO, "CYPRT: y(%3.5f, %3.5f), p(%3.5f, %3.5f), r(%3.5f, %3.5f), t(%5.5f)", yprt.yaw, yawCorr, yprt.pitch, pitchCorr, yprt.roll, rollCorr, yprt.thrust);
//	syslog(LOG_INFO, "CYPRT: y(%3.5f), p(%3.5f), r(%3.5f), t(%5.5f)", yawCorr, pitchCorr, rollCorr, yprt.thrust);

	return result;
}

MyPIDCntrllr::YPRT  MyPIDCntrllr::calcDelta(YPRT  &yprt1, YPRT  &yprt2) {
	YPRT result = yprt1 - yprt2;
	return result;
}

MyPIDCntrllr::PIDOutput MyPIDCntrllr::calcOutput(YPRT &data) {
	// Transform input (delta attitude and thrust) to output (nanoseconds for motors)
	long front = std::max<long>(1000000L, std::min<long>(2000000L, std::lrint((data.thrust + (data.pitch*deg2MicrosFactor - data.yaw*deg2MicrosYawFactor))*1000.0f)));
	long rear = std::max<long>(1000000L, std::min<long>(2000000L, std::lrint((data.thrust - (data.pitch*deg2MicrosFactor + data.yaw*deg2MicrosYawFactor))*1000.0f)));
	long left = std::max<long>(1000000L, std::min<long>(2000000L, std::lrint((data.thrust - (data.roll*deg2MicrosFactor - data.yaw*deg2MicrosYawFactor))*1000.0f)));
	long right = std::max<long>(1000000L, std::min<long>(2000000L, std::lrint((data.thrust + (data.roll*deg2MicrosFactor + data.yaw*deg2MicrosYawFactor))*1000.0f)));

	MyPIDCntrllr::PIDOutput result = {};
	result.front = front;
	result.rear = rear;
	result.left = left;
	result.right = right;
	//syslog(LOG_INFO, "MOT: f(%d), r(%d), l(%d), r(%d), y(%5.5f), p(%5.5f), r(%5.5f)", front, rear, left, right, data.yaw, data.pitch, data.roll);
	return result;
}
void MyPIDCntrllr::sendOutput(PIDOutput &data) {
	{ // out error event
		boost::shared_ptr<MyYPRError> evOut(boost::make_shared<MyYPRError>(this->getUuid(), realData.yaw, realData.pitch, realData.roll,
				                                                                            targetData.yaw,
																							targetData.pitch,
																							targetData.roll,
																							keRoll*rollErr.getMean(), keIRoll*rollErr.getIntegral(), keDRoll*rollErr.getDerivate(),
				                                                                            kePitch*pitchErr.getMean(), keIPitch*pitchErr.getIntegral(), keDPitch*pitchErr.getDerivate(),
				                                                                            keYaw*yawErr.getMean(), keIYaw*yawErr.getIntegral(), keDYaw*yawErr.getDerivate()));
		m_signal(evOut);
	}
	{// out state motors
		boost::shared_ptr<MyOutMotors> evOut(boost::make_shared<MyOutMotors>(this->getUuid(), data.front, data.rear, data.left, data.right));
		m_signal(evOut);
	}
}

MyPIDCntrllr::YPRT MyPIDCntrllr::getYPRTFromRcData() {
	YPRT result = {};
	result.yaw = RC_DIRECTION_RPY[YAW_POS]*MyPIDCntrllr::TARGET_VALUES[YAW_POS].getValue();
	result.pitch = RC_DIRECTION_RPY[PITCH_POS]*MyPIDCntrllr::TARGET_VALUES[PITCH_POS].getValue();
	result.roll = RC_DIRECTION_RPY[ROLL_POS]*MyPIDCntrllr::TARGET_VALUES[ROLL_POS].getValue();
	result.thrust = MyPIDCntrllr::TARGET_VALUES[THRUST_POS].getValue();
	return result;
}

void MyPIDCntrllr::processImuSample(boost::math::quaternion<float> sampleQ) {

	YPRT sample = calcYPRData(sampleQ);
	sample.thrust = targetData.thrust; // not evaluated from IMU. I force to requested.
	if(targetData.isZero()) {
		targetData = sample;
		realData = sample;
	}

	/* CALCOLO ERRORE
	 * l'errore è dato dalla differenza tra la variazione avvenuta e quella richiesta nel ciclo precedente
	 * deltaReal - deltaRequested = (sample - realData) - (targetData - realData) = sample - targetData
	 */
	YPRT deltaRequested = (targetData - realData);
	deltaRequested.divideYPR(FREQUENCY);
	YPRT deltaReal = sample - realData;
	calcErr(deltaRequested, deltaReal);

	targetData = this->getYPRTFromRcData();
	realData = sample;

	/* CALCOLO INPUT
	 * calculate input data for transformation function
	 * La correzione è data dalla variazione richiesta (espressa in gradi) in un ciclo di frequenza e compensata con l'errore (gained)
	 */
	YPRT newDeltaRequested = (targetData - sample);
	newDeltaRequested.divideYPR(FREQUENCY);
	newDeltaRequested.thrust = sample.thrust;
    YPRT input = calcCorrection(newDeltaRequested);

    // calculate transformation function
    PIDOutput output = calcOutput(input);

    // send to motors
    this->sendOutput(output);
}
void MyPIDCntrllr::clean() {
	yawErr.clean();
	pitchErr.clean();
	rollErr.clean();
	targetData.clean();
	realData.clean();
	this->armed = false;
}
void MyPIDCntrllr::disarm() {
	this->clean();
}
void MyPIDCntrllr::arm() {
	this->clean();
	this->armed = true;
}
void MyPIDCntrllr::processEvent(boost::shared_ptr<MyEvent> event) {
	if (!initialized) {
		initialize();
	}
	if (this->getState() == MyAgentState::Active) {
		if(event->getType() == MyEvent::EventType::MotorsDisarmed) {
			syslog(LOG_INFO, "Motors Disarmed");
			this->disarm();
		} else
		if(event->getType() == MyEvent::EventType::MotorsArmed) {
			syslog(LOG_INFO, "Motors Armed");
			this->arm();
		} else
		if (event->getType() == MyEvent::EventType::IMUSample && this->armed) {
			boost::shared_ptr<MyIMUSample> imuSample =
					boost::static_pointer_cast<MyIMUSample>(event);

			boost::math::quaternion<float> q = imuSample->getQuaternion();
			this->processImuSample(q);
		} else if(event->getType() == MyEvent::EventType::RCSample) {
			boost::shared_ptr<MyRCSample> rcSample =
					boost::static_pointer_cast<MyRCSample>(event);
			MyPIDCntrllr::TARGET_VALUES[ROLL_POS].setPercentValue((*rcSample).getRollPercent());
			MyPIDCntrllr::TARGET_VALUES[PITCH_POS].setPercentValue((*rcSample).getPitchPercent());
			MyPIDCntrllr::TARGET_VALUES[YAW_POS].setPercentValue((*rcSample).getYawPercent());
			MyPIDCntrllr::TARGET_VALUES[THRUST_POS].setPercentValue((*rcSample).getThrustPercent());
		} else if(event->getType() == MyEvent::EventType::BaroSample) {
			boost::shared_ptr<MyBaroSample> baroSample =
					boost::static_pointer_cast<MyBaroSample>(event);
			// syslog(LOG_INFO, "BaroSample: press=%d, alt=%5.5f, temp=%5.5f, seeLevelPress=%d", baroSample->getPressure(), baroSample->getAltitude(), baroSample->getTemperature(), baroSample->getSeeLevelPressure());
		}
        else {
			// skip events
		}
	}
}


