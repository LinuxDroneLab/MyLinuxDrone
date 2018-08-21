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
    data.gyroLSB = 65.5;
    data.accelLSB = 1024;

//16.4  S(-181197,-98537,-27111), M(-45,-25,-7), Min(-339,-175,-51), Max(0,0,0),
//65.5  S(-694062,-365156,-107261), M(-170,-90,-27), Min(-214,-112,-39), Max(0,0,0),
    data.gyroCal.x = -170;
    data.gyroCal.y = -90;
    data.gyroCal.z = -27;
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
