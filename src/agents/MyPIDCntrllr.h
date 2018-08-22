/*
 * MyPIDCntrllr.h
 *
 *  Created on: 11 set 2017
 *      Author: andrea
 */

#ifndef AGENTS_MYPIDCNTRLLR_H_
#define AGENTS_MYPIDCNTRLLR_H_

#include <commons/MyGlobalDefs.h>
#include <commons/RangeFloat.h>
#include <commons/ValueFloat.h>
#include <commons/helper_3dmath.h>
#include <agents/MyIMUAgent.h>
#include <agents/MyMotorsAgent.h>
#include <agents/MyRCAgent.h>

#define MYPIDCNTRLLR_MAX_DEG_PER_SEC       200.0f
#define MYPIDCNTRLLR_MAX_DEG_PER_SEC_YAW   200.0f
#define MYPIDCNTRLLR_IMU_FREQUENCY         100.0f


class MyPIDCntrllr {
public:
	MyPIDCntrllr();
	virtual ~MyPIDCntrllr();
    bool pulse();
    bool initialize();
protected:

private:

    typedef struct {
        int16_t thrust;
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    } DegPerSecond;

    typedef struct {
        int16_t front;
        int16_t rear;
        int16_t left;
        int16_t right;
    } MotorsInput;

	typedef struct {
	    int32_t pressure;
	    float temperature;
	    float altitude;
	    float speedMetersPerSecond;

	} PIDBaroData;

	bool initialized;
	bool firstCycle;

	float keRoll;
    float keDRoll;
    float keIRoll;

    float kePitch;
    float keDPitch;
    float keIPitch;

    float keYaw;
    float keDYaw;
    float keIYaw;

	DegPerSecond targetData;
    DegPerSecond inputData;
    DegPerSecond outputData;

    MotorsInput motorsInput;

	int16_t rollErr;
    int16_t rollDErr;
    int16_t rollIErr;

    int16_t pitchErr;
    int16_t pitchDErr;
    int16_t pitchIErr;

    int16_t yawErr;
    int16_t yawDErr;
    int16_t yawIErr;

    float rollDeg;
    float pitchDeg;

    float rollDegAcc;
    float pitchDegAcc;

	void disarm();
	void arm();
	void clean();
	void control();
    void updateTargetDataFromRCSample();
    void calcPID();
    void calcMotorsInput();
    void send2Motors();
    void calcRollPitchAccel();
    void calcRollPitch();

    static RangeFloat INTEGRAL_RANGE;
    static RangeFloat ALTITUDE_RANGE;

    MyMotorsAgent motorsAgent;
    MyIMUAgent imuAgent;
    MyRCAgent rcAgent;
};

#endif /* AGENTS_MYPIDCNTRLLR_H_ */
