/*
 * GY88.h
 *
 *  Created on: 14 set 2015
 *      Author: andrea
 */

#ifndef GY88_H_
#define GY88_H_

#include <stdint.h>
#include <chrono>
#include <commons/helper_3dmath.h>
#include <imu/MPU6050.h>
#include <baro/BMP085.h>

class GY88 {
public:
	typedef struct {
		uint64_t durationMicros;
		struct {
			Quaternion q;
			VectorFloat gravity;
			float ypr[3];
			VectorInt16 accel;
			VectorInt16 linearAccel;
		} imu;
		struct {
			uint16_t rawTemperature;
			uint32_t rawPressure;
			float temperature; // gradi centigradi
			int32_t pressure; // Pa
			int32_t seaLevelPressure; // Pa
			float altitude; // meters
		} barometer;
	} SensorData;
	GY88();
	virtual ~GY88();
	bool initialize();
	bool pulse();
	SensorData const & getData() const;
private:
	std::chrono::time_point<std::chrono::system_clock>  lastEpoc;
	uint64_t nextEpoc();

	SensorData data = {};
	MPU6050 mpu;
	BMP085 dps;      // Digital Pressure Sensor
	bool initMPU();
	bool initBMP();
	bool initialized = false;
};

#endif /* GY88_H_ */
