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
#define PID_CNTRLLR_MIN_THRUST 1000
#define PID_CNTRLLR_MAX_THRUST 2000

RangeFloat MyPIDCntrllr::INTEGRAL_RANGE = RangeFloat(-10.0f, 10.0f);
RangeFloat MyPIDCntrllr::ALTITUDE_RANGE = RangeFloat(-20000.0f, 20000.0f);
using namespace std;

MyPIDCntrllr::MyPIDCntrllr(): initialized(false), firstCycle(true)
{
    this->clean();

    keRoll   = 2.44f;
    keDRoll  = 0.0f;
    keIRoll  = 0.00f;

    kePitch   = 2.44f;
    keDPitch  = 0.0f;
    keIPitch  = 0.0f;

    keYaw   = 0.0;
    keDYaw  = 0.0f;
    keIYaw  = 0.0f;
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
//    float newKeRoll = float((rcAgent.getRCSample().aux1 + 500))/200.0f;
//    float newKeDRoll = float((rcAgent.getRCSample().aux2 + 500))/50.0f;
//    if(abs(keRoll - newKeRoll) > 0.01 || abs(keDRoll - newKeDRoll) > 0.01) {
//        keRoll = newKeRoll;
//        kePitch = newKeRoll;
//        keDRoll = newKeDRoll;
//        keDPitch = newKeDRoll;
//        syslog(LOG_INFO, "E(%5.5f) - D(%5.5f)", keRoll, keDRoll);
//    }

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
        /* 3      1
             \  /
              \/
              /\
             /  \
           2      4
         */
        // F3 e F4 girano in senso orario (CW)
        // F1 + F3 - F2 - F4 = pitch
        // F2 + F3 - F1 - F4 = roll
        // F3 + F4 - F1 - F2 = yaw
        // F1 + F2 + F3 + F4 = thrust
        // +--+--+--+--+   +--+   +--+
        // |-1|-1| 1| 1|   |F1|   |Y |
        // +--+--+--+--+   +--+   +--+
        // | 1|-1| 1|-1|   |F2|   |P |
        // +--+--+--+--+ * +--+ = +--+
        // |-1| 1| 1|-1|   |F3|   |R |
        // +--+--+--+--+   +--+   +--+
        // | 1| 1| 1| 1|   |F4|   |T |
        // +--+--+--+--+   +--+   +--+
        /*
         -->inv(M)
         +-----+-----+-----+-----+   +---+   +----+
         |-0.25| 0.25|-0.25| 0.25|   | Y |   | F1 |
         +-----+-----+-----+-----+   +---+   +----+
         |-0.25|-0.25| 0.25| 0.25|   | P |   | F2 |
         +-----+-----+-----+-----+ * +---+ = +----+
         | 0.25| 0.25| 0.25| 0.25|   | R |   | F3 |
         +-----+-----+-----+-----+   +---+   +----+
         | 0.25|-0.25|-0.25| 0.25|   | T |   | F4 |
         +-----+-----+-----+-----+   +---+   +----+
         */
        // Transform input (delta attitude and thrust) to output (nanoseconds for motors)
        uint32_t f1 = std::max<uint32_t>(1000000, std::min<uint32_t>(2000000, (outputData.thrust + outputData.pitch - outputData.roll - outputData.yaw) * 1000.0f));
        uint32_t f2 = std::max<uint32_t>(1000000, std::min<uint32_t>(2000000, (outputData.thrust - outputData.pitch + outputData.roll - outputData.yaw) * 1000.0f));
        uint32_t f3 = std::max<uint32_t>(1000000, std::min<uint32_t>(2000000, (outputData.thrust + outputData.pitch + outputData.roll + outputData.yaw) * 1000.0f));
        uint32_t f4 = std::max<uint32_t>(1000000, std::min<uint32_t>(2000000, (outputData.thrust - outputData.pitch - outputData.roll + outputData.yaw) * 1000.0f));

        this->motorsInput.front = uint16_t(f1/320); // in range [3125,6250]
        this->motorsInput.rear = uint16_t(f2/320);
        this->motorsInput.left = uint16_t(f3/320);
        this->motorsInput.right = uint16_t(f4/320);

        // syslog(LOG_INFO, "motorsInput: FRLR(%d, %d, %d, %d), T(%d)", this->motorsInput.front, this->motorsInput.rear, this->motorsInput.left, this->motorsInput.right, outputData.thrust);

}

void MyPIDCntrllr::send2Motors() {
    this->motorsAgent.writeMotors(this->motorsInput.rear, this->motorsInput.front, this->motorsInput.left, this->motorsInput.right);
}

void MyPIDCntrllr::calcPID() {
    MyIMUAgent::Motion6Data& imuData = this->imuAgent.getData();
    VectorInt16& gyroData = imuData.gyroDegxSec;
    if(this->firstCycle) {
        this->inputData.roll = gyroData.x;
        this->inputData.pitch = gyroData.y;
        this->inputData.yaw = gyroData.z;
    } else {
        this->inputData.roll = (this->inputData.roll * 0.7f) + (gyroData.x * 0.3f);
        this->inputData.pitch = (this->inputData.pitch * 0.7f) + (gyroData.y * 0.3f);
        this->inputData.yaw = (this->inputData.yaw * 0.7f) + (gyroData.z * 0.3f);
    }
    this->inputData.thrust = this->targetData.thrust;
    this->inputData.roll = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->inputData.roll, -PID_CNTRLLR_MAX_ROLL));
    this->inputData.pitch = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->inputData.pitch, -PID_CNTRLLR_MAX_ROLL));
    this->inputData.yaw = std::min<int16_t>(PID_CNTRLLR_MAX_YAW, std::max<int16_t>(this->inputData.yaw, -PID_CNTRLLR_MAX_YAW));

    // syslog(LOG_INFO, "inputData: RPY(%d,%d,%d)", this->inputData.roll, this->inputData.pitch, this->inputData.yaw);

    // Calc error (real - requested at prev cycle)
    int16_t newRollErr = -(this->inputData.roll - this->targetData.roll);
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

    if(inputData.thrust > 1010) {
        this->outputData.roll = (0.25f) * (keRoll * rollErr + keIRoll * rollIErr + keDRoll * rollDErr);
        this->outputData.pitch = (0.25f) * (kePitch * pitchErr + keIPitch * pitchIErr + keDPitch * pitchDErr);
        this->outputData.yaw = (0.25f) * (keYaw * yawErr + keIYaw * yawIErr + keDYaw * yawDErr);
    } else {
        this->outputData.roll = 0;
        this->outputData.pitch = 0;
        this->outputData.yaw = 0;
    }

    this->outputData.roll = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->outputData.roll, -PID_CNTRLLR_MAX_ROLL));
    this->outputData.pitch = std::min<int16_t>(PID_CNTRLLR_MAX_ROLL, std::max<int16_t>(this->outputData.pitch, -PID_CNTRLLR_MAX_ROLL));
    this->outputData.yaw = std::min<int16_t>(PID_CNTRLLR_MAX_YAW, std::max<int16_t>(this->outputData.yaw, -PID_CNTRLLR_MAX_YAW));
    this->outputData.thrust = inputData.thrust;

//    syslog(LOG_INFO, "outputData: RPY(%d,%d,%d)", this->outputData.roll, this->outputData.pitch, this->outputData.yaw);
}

void MyPIDCntrllr::updateTargetDataFromRCSample() {
    MyRCAgent::RCSample& sample = this->rcAgent.getRCSample();
    this->targetData.roll = sample.roll >> 1; // 250 deg/sec max
    this->targetData.pitch = sample.pitch >> 1; // 250 deg/sec max
    this->targetData.yaw = sample.yaw >> 3; // 62 deg/sec max
    this->targetData.thrust = std::min<int16_t>(PID_CNTRLLR_MAX_THRUST, std::max<int16_t>(1500 + sample.thrust, PID_CNTRLLR_MIN_THRUST)); // range [1000,2000]
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


