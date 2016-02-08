/*
 * MyBaroSample.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef EVENTS_MYBAROSAMPLE_H_
#define EVENTS_MYBAROSAMPLE_H_

#include <boost/uuid/uuid.hpp>
#include <events/MyEvent.h>

class MyBaroSample: public MyEvent {
public:
	MyBaroSample(boost::uuids::uuid origin, float altitude, int32_t pressure, int32_t seeLevelPressure, float temperature);
	virtual ~MyBaroSample();
	virtual MyEvent::EventType getType() const;
	float getAltitude() const;
	int32_t getPressure() const;
	int32_t getSeeLevelPressure() const;
	float getTemperature() const;
private:
	const float altitude;
	const int32_t pressure;
	const int32_t seeLevelPressure;
	const float temperature;
};

#endif /* EVENTS_MYBAROSAMPLE_H_ */
