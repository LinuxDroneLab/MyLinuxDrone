/*
 * MyIMUAgent.cpp
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#include <agents/MyIMUAgent.h>
#include <syslog.h>

MyIMUAgent::MyIMUAgent() :
        initialized(false)
{
    data.frequency = 250;
    data.gyroLSB = 16.4;
    data.accelLSB = 1024;
    data.gyroCal.x = -32;
    data.gyroCal.y = 3;
    data.gyroCal.z = -3;
}

MyIMUAgent::~MyIMUAgent()
{

}
bool MyIMUAgent::initialize()
{
    if (!initialized)
    {
        initialized = imu.initialize();
    }
    return initialized;
}
bool MyIMUAgent::loadData()
{
    bool result = false;
    if (initialize())
    {
        if (imu.getIntDataReadyStatus())
        {
            imu.getMotion6(&data.accel.x, &data.accel.y, &data.accel.z,
                           &data.gyro.x, &data.gyro.y, &data.gyro.z);

            data.gyroDegxSec.x = (data.gyro.x - data.gyroCal.x) / data.gyroLSB;
            data.gyroDegxSec.y = (data.gyro.y - data.gyroCal.y) / data.gyroLSB;
            data.gyroDegxSec.z = (data.gyro.z - data.gyroCal.z) / data.gyroLSB;
            result = true;
        }

    }
    return result;
}
MyIMUAgent::Motion6Data& MyIMUAgent::getData() {
    return this->data;
}
