/*
 * MyIMUAgent.h
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYIMUAGENT_H_
#define AGENTS_MYIMUAGENT_H_

#include <imu/MPU6050.h>

class MyIMUAgent {
public:
    typedef struct {
        VectorInt16 accel;
        VectorInt16 accelCal;
        VectorInt16 gyro;
        VectorInt16 gyroCal;
        VectorInt16 gyroDegxSec;
        uint8_t frequency;
        float gyroLSB;
        uint16_t accelLSB;
    } Motion6Data;

	MyIMUAgent();
	virtual ~MyIMUAgent();
    bool initialize();
    bool loadData();
    Motion6Data& getData();
protected:

private:
	bool initialized;
	MPU6050 imu;
	Motion6Data data;
};

#endif /* AGENTS_MYIMUAGENT_H_ */
