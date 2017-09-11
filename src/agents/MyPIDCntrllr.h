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
		friend const YPRT operator-(const YPRT& lhs, const YPRT& rhs) {
			    YPRT r = {};
			    r.yaw = lhs.yaw - rhs.yaw;
			    r.pitch = lhs.pitch - rhs.pitch;
			    r.roll = lhs.roll - rhs.roll;
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
		long front;
		long rear;
		long left;
		long right;
		void clean() {
			front = 1000000L;
			rear = 1000000L;
			left = 1000000L;
			right = 1000000L;
		}
	} PIDOutput;

	bool initialized;
	void initialize();
	bool armed;

	YPRT targetData = {}; // from RC
	YPRT realData = {}; // from IMU Sample

	void disarm();
	void arm();
	void clean();

	void processImuSample(boost::math::quaternion<float> sample);
	YPRT getYPRTFromRcData();
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

	float deg2MillisFactor;

	static RangeFloat TARGET_RANGES[];
	static ValueFloat TARGET_VALUES[];
};

#endif /* AGENTS_MYPIDCNTRLLR_H_ */
