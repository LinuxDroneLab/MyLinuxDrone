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

class MyIMUSample: public MyEvent {
public:
	explicit MyIMUSample(boost::uuids::uuid origin, boost::math::quaternion<float> q);
	virtual ~MyIMUSample();
	virtual MyEvent::EventType getType() const;
	boost::math::quaternion<float> getQuaternion() const;
private:
	boost::math::quaternion<float> q;

};

#endif /* EVENTS_MYIMUSAMPLE_H_ */
