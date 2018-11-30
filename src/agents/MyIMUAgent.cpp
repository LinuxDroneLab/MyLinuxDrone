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
    data.accelLSB = 8192;

    /* old
     * AxisFactors(1.04191, 1.0508, 1.01568)
     * Accel AxisFactors(1.04191, 1.0508, 1.01568)
     * Accel K=(0.00232558, 0.00206612. 0.00163934)
     * Gyro Offsets(-171, -93, -26)
     *
     * new: 30/11/2018 (nuova mpu6050)
     * AxisFactors(1.03861, 1.00961, 0.987404)
     * AxisOffset(0.000488281, 0, -0)
     *
     *
     */
    data.gyroCal.x = -387;
    data.gyroCal.y = -65;
    data.gyroCal.z = 5;

    data.accelKX = 0.00232558f;
    data.accelKY = 0.00206612f;
    data.accelKZ = 0.00163934f;

    data.accelAxisFactorX = 1.03861f;
    data.accelAxisFactorY = 1.00961f;
    data.accelAxisFactorZ = 0.987404f;

    data.accel.x = 0;
    data.accel.y = 0;
    data.accel.z = 0;
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
bool MyIMUAgent::reset()
{
    this->initialized = false;
    this->imu.reset();
    return true;
}
bool MyIMUAgent::loadData()
{
    int16_t localData[6];
    bool result = false;
    if (initialize())
    {
        if (imu.getIntDataReadyStatus())
        {

            imu.getMotion6(localData, localData+1, localData+2,
                           localData+3, localData+4, localData+5);

            data.accel.x = float(data.accel.x) * (1.0f - data.accelKX) + float(localData[0]) * data.accelAxisFactorX * data.accelKX;
            data.accel.y = float(data.accel.y) * (1.0f - data.accelKY) + float(localData[1]) * data.accelAxisFactorY * data.accelKY;
            data.accel.z = float(data.accel.z) * (1.0f - data.accelKZ) + float(localData[2]) * data.accelAxisFactorZ * data.accelKZ;
            data.gyro.x = localData[3];
            data.gyro.y = localData[4];
            data.gyro.z = localData[5];

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
