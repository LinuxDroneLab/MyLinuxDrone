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
	MyBaroSample(boost::uuids::uuid origin, float altitude, float estimatedAltitude, int32_t pressure, int32_t seeLevelPressure, float temperature, uint32_t rawPressure, uint16_t rawTemperature, uint16_t dtime);
	virtual ~MyBaroSample();
	virtual MyEvent::EventType getType() const;
	float getAltitude() const;
    float getEstimatedAltitude() const;
	int32_t getPressure() const;
	int32_t getSeeLevelPressure() const;
	float getTemperature() const;
    uint32_t getRawPressure() const;
    uint16_t getRawTemperature() const;
    uint16_t getDTimeMillis() const;
private:
	const float altitude;
    const float estimatedAltitude;
	const int32_t pressure;
	const int32_t seeLevelPressure;
	const float temperature;
	const uint32_t rawPressure;
    const uint16_t rawTemperature;
    uint16_t dTimeMillis;

};

#endif /* EVENTS_MYBAROSAMPLE_H_ */
