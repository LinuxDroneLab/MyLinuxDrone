/*
 * MyPIDCntrllr.h
 *
 *  Created on: 11 set 2017
 *      Author: andrea
 */

#ifndef AGENTS_MYPIDCNTRLLR_H_
#define AGENTS_MYPIDCNTRLLR_H_

#include <agents/MyAgent.h>
#include <commons/MyGlobalDefs.h>
#include <boost/math/quaternion.hpp>
#include <queues/MyPIDBuffer.h>
#include <commons/RangeFloat.h>
#include <commons/ValueFloat.h>
#include <commons/helper_3dmath.h>

#define MYPIDCNTRLLR_MAX_DEG_PER_SEC       200.0f
#define MYPIDCNTRLLR_MAX_DEG_PER_SEC_YAW   200.0f
#define MYPIDCNTRLLR_IMU_FREQUENCY         100.0f


class MyPIDCntrllr: public MyAgent {
public:
	MyPIDCntrllr(boost::shared_ptr<MyEventBus> bus,  vector<MyEvent::EventType> acceptedEventTypes);
	virtual ~MyPIDCntrllr();
protected:
	virtual void processEvent(boost::shared_ptr<MyEvent> event);

private:
	// espressi in gradi
	class YPRT {
	public:
		float yaw;
		float pitch;
		float roll;
		float thrust;
		void clean() {
			yaw = 0.0f;
			pitch = 0.0f;
			roll = 0.0f;
			thrust = 0.0f;
		}
		bool isZero() {
			return yaw == 0.0f && pitch == 0.0f && roll == 0.0f && thrust == 0.0f;
		}
		void limitYPR(float PRLimit, float YLimit){
			if(PRLimit < 0.0f) {
				PRLimit = -PRLimit;
			}
			if(YLimit < 0.0f) {
				YLimit = -YLimit;
			}
			yaw = max(std::min(yaw, YLimit), -YLimit);
			pitch = max(std::min(pitch, PRLimit), -PRLimit);
			roll = max(std::min(roll, PRLimit), -PRLimit);
		}
		void divideYPR(float divisor){
			yaw /= divisor;
			pitch /= divisor;
			roll /= divisor;
		}
		void multiplyYPR(float multiplier){
			yaw *= multiplier;
			pitch *= multiplier;
			roll *= multiplier;
		}
		friend const YPRT operator-(const YPRT& lhs, const YPRT& rhs) {
			    YPRT r = {};
			    float appL = lhs.yaw * 1000.0f;
			    float appR = rhs.yaw * 1000.0f;
			    float app = appL - appR;
			    app = app/1000.0f;
			    r.yaw = app;

			    appL = lhs.pitch * 1000.0f;
			    appR = rhs.pitch * 1000.0f;
			    app = appL - appR;
			    app = app/1000.0f;
			    r.pitch = app;

			    appL = lhs.roll * 1000.0f;
			    appR = rhs.roll * 1000.0f;
			    app = appL - appR;
			    app = app/1000.0f;
			    r.roll = app;

			    r.thrust = lhs.thrust - rhs.thrust;
		        return r;
		}
		friend const YPRT operator+(const YPRT& lhs, const YPRT& rhs) {
		    YPRT r = {};
		    r.yaw = lhs.yaw + rhs.yaw;
		    r.pitch = lhs.pitch + rhs.pitch;
		    r.roll = lhs.roll + rhs.roll;
		    r.thrust = lhs.thrust + rhs.thrust;
		    return r;
		}

		YPRT& operator=(const YPRT& rhs) {
		    yaw = rhs.yaw;
		    pitch = rhs.pitch;
		    roll = rhs.roll;
		    thrust = rhs.thrust;
		    return *this;
		}
	};

	typedef struct {
		uint16_t front;
		uint16_t rear;
		uint16_t left;
		uint16_t right;
		void clean() {
			front = 3125;
			rear = 3125;
			left = 3125;
			right = 3125;
		}
	} PIDOutput;

	typedef struct {
	    int32_t pressure;
	    float temperature;
	    float altitude;
	    float speedMetersPerSecond;

	} PIDBaroData;

	bool initialized;
	void initialize();
	bool armed;

	YPRT requestedData = {}; // from RC
	YPRT prevSample = {}; // from IMU Sample
    YPRT prevExpected = {}; // from IMU Sample
    long int imuSampleTimeStampMillis;

	void disarm();
	void arm();
	void clean();

	void processImuSample(boost::math::quaternion<float> sample, float yaw, float pitch, float roll, VectorFloat gravity, VectorInt16 accel, VectorInt16 linearAccel, long int elapsedMillis);

	YPRT getYPRTFromTargetData();
	YPRT calcYPRData(boost::math::quaternion<float> q);
	YPRT calcCorrection(YPRT &delta);
	YPRT calcDelta(YPRT &yprt1, YPRT &yprt2);

	void calcErr(YPRT &yprtReq, YPRT &yprtReal);
	PIDOutput calcOutput(YPRT &data);
	void sendOutput(PIDOutput &data);

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

	float deg2MicrosFactor;
	float deg2MicrosYawFactor;

	PIDBaroData baroData;
	MyPIDBuffer altitudeBuff;

	static RangeFloat TARGET_RANGES[];
	static ValueFloat TARGET_VALUES[];
	static int8_t QUATERNION_DIRECTION_RPY[];
	static int8_t RC_DIRECTION_RPY[];
    static float FREQUENCY;
    static RangeFloat INTEGRAL_RANGE;
    static RangeFloat ALTITUDE_RANGE;
};

#endif /* AGENTS_MYPIDCNTRLLR_H_ */
