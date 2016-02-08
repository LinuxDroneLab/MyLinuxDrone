/*
 * GY88.cpp
 *
 *  Created on: 14 set 2015
 *      Author: andrea
 */

#include <imu/GY88.h>

GY88::GY88() {
}

GY88::~GY88() {
}

GY88::SensorData const & GY88::getData() const {
	return data;
}
bool GY88::initialize() {
	initialized = (initMPU() || initBMP());
	lastEpoc = std::chrono::system_clock::now();

	return initialized;
}
bool GY88::initMPU() {
	// TODO: trasferire inizializzazione nel pulse di MPU6050.
	// in questo modo il client non deve eseguire i processi di inizializzazione interni del sensore,
	// ma occuparsi solo di pulsare fino ad ottenere i dati
	return mpu.dmpBegin();
}
bool GY88::initBMP() {
	// nothing to do.
	// trasferito inizializzazione nel pulse
	return true;
}
uint64_t GY88::nextEpoc() {
	auto now = std::chrono::system_clock::now();
	auto duration = now - lastEpoc;
	lastEpoc = now;
	auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
			duration).count();
	return micros;
}

bool GY88::pulse() {
	bool result = false;
	if(initialized) {
		// TODO: definire operatori di assegnamento
		if(mpu.pulse()) {
			const MPU6050::SensorData& md = mpu.getData();
			data.imu.accel = md.accel;
			data.imu.gravity = md.gravity;
			data.imu.linearAccel = md.linearAccel;
			data.imu.q = md.q;
			data.imu.ypr[0] = md.ypr[0];
			data.imu.ypr[1] = md.ypr[1];
			data.imu.ypr[2] = md.ypr[2];
			result = true;
		}
		if(dps.pulse()) {
			const BMP085::SensorData& bd = dps.getData();
			data.barometer.altitude = bd.altitude;
			data.barometer.pressure = bd.pressure;
			data.barometer.rawPressure = bd.rawPressure;
			data.barometer.rawTemperature = bd.rawTemperature;
			data.barometer.seaLevelPressure = bd.seaLevelPressure;
			data.barometer.temperature = bd.temperature;
			result = true;
		}
		if(result) {
			data.durationMicros = nextEpoc();
		}
	}
	// verificare i tempi di esecuzione, deve rientrare in un ciclo di frequenza
	return result;
}
