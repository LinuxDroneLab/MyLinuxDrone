/*
 * MyBaroSample.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyBaroSample.h>

MyBaroSample::MyBaroSample(boost::uuids::uuid origin, float altitude,
		int32_t pressure, int32_t seeLevelPressure, float temperature) :
		MyEvent(origin), altitude(altitude), pressure(pressure), seeLevelPressure(
				seeLevelPressure), temperature(temperature) {
	this->setPriority(MyPriority::BAROMETER_SAMPLE_PRIORITY);
}

MyBaroSample::~MyBaroSample() {
	// TODO Auto-generated destructor stub
}

MyEvent::EventType MyBaroSample::getType() const {
	return MyEvent::EventType::BaroSample;
}

float MyBaroSample::getAltitude() const {
	return this->altitude;
}

float MyBaroSample::getTemperature() const {
	return this->temperature;
}
int32_t MyBaroSample::getPressure() const {
	return this->pressure;
}
int32_t MyBaroSample::getSeeLevelPressure() const {
	return this->seeLevelPressure;
}
