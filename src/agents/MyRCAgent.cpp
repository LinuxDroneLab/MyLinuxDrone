/*
 * MyRCAgent.cpp
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#include <iostream>
#include <pthread.h>
#include <syslog.h>
#include <fcntl.h>
#include <agents/MyRCAgent.h>
#include <pru_rc_lib.h>

#define CHAN_THRUST 1
#define CHAN_ROLL 0
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_AUX1 4
#define CHAN_AUX2 5

unsigned char MyRCAgent::readBuf[MYRCAGENT_MAX_BUFFER_SIZE] = { };

// PRU1
#define MYRCAGENT_DEVICE_NAME             "/dev/rpmsg_pru31"

// 2 bytes per cmd_id e tipo record
// 32 bytes per ogni canale. Sono 8 canali. 8 * 32 bytes = 270 bytes
// totale 272 bytes
#define MYRCAGENT_ECAP_DATA_SIZE 272
/*
 * TODO: Change static values to dynamic configuration
 */
RangeInt16 MyRCAgent::PRU_RANGES[6] = { RangeInt16(628, 1586), RangeInt16(604,
                                                                         1583),
                                       RangeInt16(665, 1531), RangeInt16(604,
                                                                         1581),
                                       RangeInt16(595, 1588), RangeInt16(596,
                                                                         1587) };
RangeInt16 MyRCAgent::CHAN_RANGES[6] = { RangeInt16(-500, 500), RangeInt16(-500,
                                                                          500),
                                        RangeInt16(-500, 500), RangeInt16(-500,
                                                                          500),
                                        RangeInt16(-500, 500), RangeInt16(-500,
                                                                          500) };
int16_t MyRCAgent::CHAN_CENTER_VALUES[6] = { -8,0,8,0,0,0 };

ValueInt16 MyRCAgent::PRU_VALUES[6] = {
        ValueInt16(0, MyRCAgent::PRU_RANGES[CHAN_ROLL]), ValueInt16(
                0, MyRCAgent::PRU_RANGES[CHAN_THRUST]),
        ValueInt16(0, MyRCAgent::PRU_RANGES[CHAN_PITCH]), ValueInt16(
                0, MyRCAgent::PRU_RANGES[CHAN_YAW]),
        ValueInt16(0, MyRCAgent::PRU_RANGES[CHAN_AUX1]), ValueInt16(
                0, MyRCAgent::PRU_RANGES[CHAN_AUX2]) };
ValueInt16 MyRCAgent::CHAN_VALUES[6] = {
        ValueInt16(0, MyRCAgent::CHAN_RANGES[CHAN_ROLL]), ValueInt16(
                0, MyRCAgent::CHAN_RANGES[CHAN_THRUST]),
        ValueInt16(0, MyRCAgent::CHAN_RANGES[CHAN_PITCH]), ValueInt16(
                0, MyRCAgent::CHAN_RANGES[CHAN_YAW]),
        ValueInt16(0, MyRCAgent::CHAN_RANGES[CHAN_AUX1]), ValueInt16(
                0, MyRCAgent::CHAN_RANGES[CHAN_AUX2]) };

MyRCAgent::MyRCAgent() :
        initialized(false),
        minThrustMaxPitch(false),
        minThrustMinPitch(false)
{
    this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
    this->lastDTimeMicros = 0;
    this->lastTickMicros =
            boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
    this->tickDTimeSum = 0;
    this->tickDTimeWaitMicros = 100000; // 10Hz
}

MyRCAgent::~MyRCAgent()
{
}

bool MyRCAgent::initialize()
{
    if (!initialized)
    {
        syslog(LOG_INFO, "mydrone: MyRCAgent: initializing ...");

        /* Open the rpmsg_pru character device file */
        pruDevice.fd = open(MYRCAGENT_DEVICE_NAME, O_RDWR);

        if (pruDevice.fd < 0)
        {
            syslog(LOG_ERR, "mydrone: MyRCAgent: Failed to open [%s] ",
                   MYRCAGENT_DEVICE_NAME);
            initialized = false;
        }
        else
        {
            initialized = true;
        }
    }
    return initialized;
}

MyRCAgent::RCSample& MyRCAgent::getRCSample()
{
    return this->rcSample;
}

void MyRCAgent::setRCSample()
{
    this->rcSample.thrust = MyRCAgent::CHAN_VALUES[CHAN_THRUST].getValue();
    this->rcSample.roll = MyRCAgent::CHAN_VALUES[CHAN_ROLL].getValue() - MyRCAgent::CHAN_CENTER_VALUES[CHAN_ROLL];
    this->rcSample.pitch = MyRCAgent::CHAN_VALUES[CHAN_PITCH].getValue() - MyRCAgent::CHAN_CENTER_VALUES[CHAN_PITCH];
    this->rcSample.yaw = MyRCAgent::CHAN_VALUES[CHAN_YAW].getValue() - MyRCAgent::CHAN_CENTER_VALUES[CHAN_YAW];
    this->rcSample.aux1 = MyRCAgent::CHAN_VALUES[CHAN_AUX1].getValue() - MyRCAgent::CHAN_CENTER_VALUES[CHAN_AUX1];
    this->rcSample.aux2 = MyRCAgent::CHAN_VALUES[CHAN_AUX2].getValue() - MyRCAgent::CHAN_CENTER_VALUES[CHAN_AUX2];
}

bool MyRCAgent::sendDataRequest()
{
    for (int i = 0; i < MYRCAGENT_MAX_BUFFER_SIZE; i++)
    {
        readBuf[i] = 0;
    }
    readBuf[0] = PRU_RC_LIB_CMD_ID;
    readBuf[1] = PRU_RC_LIB_CMD_GET_DATA;
    int result = write(pruDevice.fd, readBuf, 2);
//    if (result > 0) {
//        printf("Message GET_DATA sent to PRU\n");
//    }
    return (result > 0);
}
bool MyRCAgent::receiveData()
{
    bool result = false;
    uint32_t* data = (uint32_t*) (readBuf + 2);
    int bytes = read(pruDevice.fd, readBuf, MYRCAGENT_ECAP_DATA_SIZE);
    if (bytes > 0)
    {
//        printf("Message received from PRU:%d:%d:%d 1\n",readBuf[0], readBuf[1], bytes);
        if ((readBuf[0] == PRU_RC_LIB_CMD_ID)
                && (readBuf[1] == PRU_RC_LIB_CMD_GET_DATA_RSP))
        {
            for (int j = 0; j < 6; j++)
            {
                MyRCAgent::PRU_VALUES[j].setValue(uint16_t(data[j] / 200));
                MyRCAgent::CHAN_VALUES[j].setValue(MyRCAgent::PRU_VALUES[j]);
            }
            this->setRCSample();
        }
        result = true;
    }

    return result;
}

bool MyRCAgent::isMinThrustMaxPitch()
{
    return this->minThrustMaxPitch;
}

bool MyRCAgent::isMinThrustMinPitch()
{
    return this->minThrustMinPitch;
}

bool MyRCAgent::updateTickTimestamp()
{
    uint32_t now =
            boost::posix_time::microsec_clock::local_time().time_of_day().total_microseconds();
    lastDTimeMicros = uint32_t(now - lastTickMicros);
    lastTickMicros = now;
    this->tickDTimeSum += this->lastDTimeMicros;
    if (this->tickDTimeSum > this->tickDTimeWaitMicros)
    {
        this->tickDTimeSum -= this->tickDTimeWaitMicros;
        return true;
    }
    return false;
}

/*
 * Count tick events. If (num ticks % divider) == 0, then go to RequireMode State
 * States:
 * - Idle: nothing to do
 * - RequireMode: send 'require data' message to PRU.
 * - Transition: RequireMode -> ReceiveMode. Immediate on RequireMode activity done.
 * - ReceiveMode: non blocking read data from /dev/rpmsg_31 (PRU1).
 * - Transition: ReceiveMode -> Idle if ((num ticks % divider) != 0 and data received)
 * - Transition: ReceiveMode -> RequireMode if (num tiks % divider) == 0
 * - Transition: Idle -> RequireMode if (num ticks % divider) == 0
 */
bool MyRCAgent::loadData()
{
    bool result = false;
    if (!initialize())
    {
        return false;
    }
    bool updatedRc = this->updateTickTimestamp();
    switch (this->status)
    {
    case (MYRCAGENT_STATUS_REQUIRE_MODE):
    {
        if (this->sendDataRequest())
        {
            this->status = MYRCAGENT_STATUS_RECEIVE_MODE;
        } else {
            syslog(LOG_INFO, "MyRCAgent:Cannot send data request!");
        }
        break;
    }
    case (MYRCAGENT_STATUS_RECEIVE_MODE):
    {
        if (this->receiveData())
        {
            //syslog(LOG_INFO, "T[%d], P[%d]", rcSample.thrust, rcSample.pitch);
            this->minThrustMaxPitch = false;
            this->minThrustMinPitch = false;
            if (rcSample.thrust <= MYRCAGENT_MIN_LIMIT_THRUST && rcSample.pitch >= MYRCAGENT_MAX_LIMIT_PITCH)
            {
                this->minThrustMaxPitch = true;
            }
            else if (rcSample.thrust <= MYRCAGENT_MIN_LIMIT_THRUST && rcSample.pitch <= MYRCAGENT_MIN_LIMIT_PITCH)
            {
                this->minThrustMinPitch = true;
            }
            else
            {
            }
            result = true;
            this->status = MYRCAGENT_STATUS_WAIT_MODE;
        }
        else if (updatedRc)
        {
            syslog(LOG_INFO, "MyRCAgent: RC data not received!");
            this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
        }
        break;
    }
    case (MYRCAGENT_STATUS_WAIT_MODE):
    {
        if (updatedRc)
        {
            this->status = MYRCAGENT_STATUS_REQUIRE_MODE;
        }
        break;
    }
    }
    return result; // true => state changed
}
