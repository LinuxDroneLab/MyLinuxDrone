/*
 * MyIMUSample.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYIMUSAMPLE_H_
#define EVENTS_MYIMUSAMPLE_H_

#include <boost/math/quaternion.hpp>
#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>
#include <commons/helper_3dmath.h>

class MyIMUSample: public MyEvent
{
public:
    explicit MyIMUSample(boost::uuids::uuid origin,
                         boost::math::quaternion<float> q, float yaw,
                         float pitch, float roll, VectorFloat gravity,
                         VectorInt16 accel, VectorInt16 linearAccel);
    virtual ~MyIMUSample();
    virtual MyEvent::EventType getType() const;
    boost::math::quaternion<float> getQuaternion() const;
    float getYaw() const;
    float getPitch() const;
    float getRoll() const;
    VectorFloat getGravity() const;
    VectorInt16 getAccel() const;
    VectorInt16 getLinearAccel() const;
private:
    boost::math::quaternion<float> q;
    float yaw;
    float pitch;
    float roll;
    VectorFloat gravity;
    VectorInt16 accel;
    VectorInt16 linearAccel;

};

#endif /* EVENTS_MYIMUSAMPLE_H_ */
