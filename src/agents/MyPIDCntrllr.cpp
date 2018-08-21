/*
 * MyPIDCntrllr.cpp
 *
 *  Created on: 11 set 2017
 *      Author: andrea
 */

#include <agents/MyPIDCntrllr.h>
#include <iostream>
#include <syslog.h>

#define pi 3.1415926
#define pi2 1.5707963
#define PID_CNTRLLR_MAX_ROLL 400
#define PID_CNTRLLR_MAX_YAW 400

RangeFloat MyPIDCntrllr::INTEGRAL_RANGE = RangeFloat(-10.0f, 10.0f);
RangeFloat MyPIDCntrllr::ALTITUDE_RANGE = RangeFloat(-20000.0f, 20000.0f);
using namespace std;

MyPIDCntrllr::MyPIDCntrllr(): initialized(false), firstCycle(true)
{
    this->clean();

    keRoll   = 1.3f;
    keDRoll  = 18.0f;
    keIRoll  = 0.04f;

    kePitch   = 1.3f;
    keDPitch  = 18.0f;
    keIPitch  = 0.04f;

    keYaw   = 4.0;
    keDYaw  = 0.0f;
    keIYaw  = 0.02f;
}

MyPIDCntrllr::~MyPIDCntrllr()
{

}

bool MyPIDCntrllr::initialize()
{
    bool result = true;
    if(!initialized) {
        result &= this->motorsAgent.initialize();
        result &= this->rcAgent.initialize();
        result &= this->imuAgent.initialize();

        initialized = result;
        syslog(LOG_INFO, "mydrone: MyPIDCntrllr: initialization OK");
    }
    return result;
}
bool MyPIDCntrllr::pulse()
{
    initialize();
    bool imuChanged = this->imuAgent.loadData();
    bool rcChanged = this->rcAgent.loadData();
    if(imuChanged) { // I work at imu frequency
        this->control();
    }
    return imuChanged | rcChanged;
}
/*
 * Get input from imu and rc,
 * then calc pid and output
 */
void MyPIDCntrllr::control() {
    if(rcAgent.isMinThrustMaxPitch()) {
        this->motorsAgent.armMotors();
    } else if(rcAgent.isMinThrustMinPitch()) {
        this->motorsAgent.disarmMotors();
    } else {
        this->updateTargetDataFromRCSample();
    }

    this->calcPID();
    this->calcMotorsInput();
    this->send2Motors();

    this->firstCycle = false;

//    syslog(LOG_INFO, "mydrone: G(%5.5f,%5.5f,%5.5f)", this->imuAgent.getData().gyroDegxSec.x, this->imuAgent.getData().gyroDegxSec.y, this->imuAgent.getData().gyroDegxSec.z);
}
void MyPIDCntrllr::calcMotorsInput() {

}
void MyPIDCntrllr::send2Motors() {

}
void MyPIDCntrllr::calcPID() {
    MyIMUAgent::Motion6Data& imuData = this->imuAgent.getData();
    VectorInt16& gyroData = imuData.gyroDegxSec;
    if(this->firstCycle) {
        this->inputData.roll = gyroData.y;
        this->inputData.pitch = gyroData.x;
        this->inputData.yaw = gyroData.z;
    } else {
        this->inputData.roll = (this->inputData.roll * 0.7f) + (gyroData.y * 0.3f);
        this->inputData.pitch = (this->inputData.pitch * 0.7f) + (gyroData.x * 0.3f);
        this->inputData.yaw = (this->inputData.yaw * 0.7f) + (gyroData.z * 0.3f);
    }

    this->inputData.roll = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->inputData.roll, -PID_CNTRLLR_MAX_ROLL));
    this->inputData.pitch = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->inputData.pitch, -PID_CNTRLLR_MAX_ROLL));
    this->inputData.yaw = std::min<int16_t>(PID_CNTRLLR_MAX_YAW, std::max<int16_t>(this->inputData.yaw, -PID_CNTRLLR_MAX_YAW));

    // Calc error (real - requested at prev cycle)
    int16_t newRollErr = this->inputData.roll - this->targetData.roll;
    int16_t newPitchErr = this->inputData.pitch - this->targetData.pitch;
    int16_t newYawErr = this->inputData.yaw - this->targetData.yaw;

    rollDErr = newRollErr - this->rollErr;
    pitchDErr = newPitchErr - this->pitchErr;
    yawDErr = newYawErr - this->yawErr;

    rollErr = newRollErr;
    pitchErr = newPitchErr;
    yawErr = newYawErr;

    rollIErr += this->rollErr;
    pitchIErr += this->pitchErr;
    yawIErr += this->yawErr;


    this->outputData.roll = keRoll * rollErr + keIRoll * rollIErr + keDRoll * rollDErr;
    this->outputData.pitch = kePitch * pitchErr + keIPitch * pitchIErr + keDPitch * pitchDErr;
    this->outputData.yaw = keYaw * yawErr + keIYaw * yawIErr + keDYaw * yawDErr;

    this->outputData.roll = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->outputData.roll, -PID_CNTRLLR_MAX_ROLL));
    this->outputData.pitch = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->outputData.pitch, -PID_CNTRLLR_MAX_ROLL));
    this->outputData.yaw = std::min<int16_t>(PID_CNTRLLR_MAX_YAW, std::max<int16_t>(this->outputData.roll, -PID_CNTRLLR_MAX_YAW));
}

void MyPIDCntrllr::updateTargetDataFromRCSample() {
    MyRCAgent::RCSample& sample = this->rcAgent.getRCSample();
    this->targetData.roll = sample.roll >> 1; // 250 deg/sec max
    this->targetData.pitch = sample.pitch >> 1; // 250 deg/sec max
    this->targetData.yaw = sample.yaw >> 3; // 62 deg/sec max
    this->targetData.thrust = 1500 + sample.thrust; // range [1000,2000]
}

void MyPIDCntrllr::clean()
{
    rollErr = 0;
    rollDErr = 0;
    rollIErr = 0;

    pitchErr = 0;
    pitchDErr = 0;
    pitchIErr = 0;

    yawErr = 0;
    yawDErr = 0;
    yawIErr = 0;

    this->targetData.roll = 0;
    this->targetData.pitch = 0;
    this->targetData.yaw = 0;
    this->targetData.thrust = 1000;
}
void MyPIDCntrllr::disarm()
{
    this->motorsAgent.disarmMotors();
    this->clean();
}
void MyPIDCntrllr::arm()
{
    this->clean();
    this->motorsAgent.armMotors();
}


