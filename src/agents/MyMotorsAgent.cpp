/*
 * MyMotorsAgent.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <agents/MyMotorsAgent.h>
#include <syslog.h>
#include <fcntl.h>
#include <thread>
#include <chrono>

#define MYMOTORSAGENT_MAX_BUFFER_SIZE         64
unsigned char MyMotorsAgent::readBuf[MYMOTORSAGENT_MAX_BUFFER_SIZE] = { 0 };
#define MYMOTORSAGENT_DEVICE_NAME             "/dev/rpmsg_pru30"
#define MYMOTORSAGENT_MIN_DUTY 3125
#define MYMOTORSAGENT_MAX_DUTY 6250
#define MYMOTORSAGENT_PERIOD 62499

MyMotorsAgent::MyMotorsAgent() :
        initialized(false), armed(false)
{
}

MyMotorsAgent::~MyMotorsAgent()
{
    close(pruDevice.fd);
}

bool MyMotorsAgent::initialize()
{
    syslog(LOG_INFO, "mydrone: MyMotorsAgent: initializing motor drivers");
    bool result = true;
    if(!initialized) {
        /* Open the rpmsg_pru character device file */
        pruDevice.fd = open(MYMOTORSAGENT_DEVICE_NAME, O_RDWR);

        if (pruDevice.fd < 0)
        {
            syslog(LOG_ERR, "mydrone: MyMotorsAgent: Failed to open [%s] ",
                   MYMOTORSAGENT_DEVICE_NAME);
            result = false;
        }
        else
        {
            this->setThrottleRange();
            this->disarmMotors();
            syslog(LOG_INFO, "mydrone: MotorsAgent: initialization OK");
        }
        initialized = result;
    }
    return result;
}

void MyMotorsAgent::writeMotors(uint16_t rear, uint16_t front, uint16_t left, uint16_t right)
{
    if(this->armed) {
        this->sendDuty(rear, front, left,right);
    }
}

/*
 * Attention! this method is sync and
 * require at least five seconds
 */
void MyMotorsAgent::setThrottleRange()
{
    this->sendDuty(MYMOTORSAGENT_MAX_DUTY, MYMOTORSAGENT_MAX_DUTY,
                   MYMOTORSAGENT_MAX_DUTY, MYMOTORSAGENT_MAX_DUTY);
    sleep(3);
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY,
                   MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);
    sleep(2);
}

void MyMotorsAgent::disarmMotors()
{
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY,
                   MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);

    if(armed) {
        syslog(LOG_INFO, "mydrone: Motors Disarmed!");
    }
    armed = false;
}
void MyMotorsAgent::armMotors()
{
    this->sendDuty(MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY,
                   MYMOTORSAGENT_MIN_DUTY, MYMOTORSAGENT_MIN_DUTY);
    if(!armed) {
        syslog(LOG_INFO, "mydrone: Motors Armed!");
    }
    armed = true;
}

bool MyMotorsAgent::isArmed() {
    return this->armed;
}

bool MyMotorsAgent::sendStart()
{
    for (int i = 0; i < MYMOTORSAGENT_MAX_BUFFER_SIZE; i++)
    {
        readBuf[i] = 0;
    }
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_START;
    readBuf[2] = 5; // enable pwmss 0 and 2

    return (write(pruDevice.fd, readBuf, 3) > 0) && this->receiveData();
}
void MyMotorsAgent::cleanBuffer()
{
    for (int i = 0; i < MYMOTORSAGENT_MAX_BUFFER_SIZE; i++)
    {
        readBuf[i] = 0;
    }
}

bool MyMotorsAgent::sendStop()
{
    this->cleanBuffer();
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_STOP;
    readBuf[2] = 5; // disable pwmss 0 and 2
    return (write(pruDevice.fd, readBuf, 3) > 0) && this->receiveData();
}

bool MyMotorsAgent::sendDuty(uint16_t du1A, uint16_t du1B, uint16_t du2A, uint16_t du2B)
{
    this->cleanBuffer();
    readBuf[0] = PRU_PWMSS_LIB_CMD_ID;
    readBuf[1] = PRU_PWMSS_LIB_CMD_SET_DATA;
    readBuf[2] = 0; // pwmss 0
    readBuf[9] = 2; // pwmss 2
    uint16_t* data1 = (uint16_t*) (readBuf + 3);
    data1[0] = MYMOTORSAGENT_PERIOD;
    data1[1] = du1A;
    data1[2] = du1B;
    uint16_t* data2 = (uint16_t*) (readBuf + 10);
    data2[0] = MYMOTORSAGENT_PERIOD;
    data2[1] = du2A;
    data2[2] = du2B;
    return (write(pruDevice.fd, readBuf, 16) > 0) && this->receiveData();
}

bool MyMotorsAgent::receiveData()
{
    this->cleanBuffer();
    bool result = (read(pruDevice.fd, readBuf, 4) > 0);
    if (!result)
    {
        syslog(LOG_INFO,
               "mydrone: MyMotorsAgent: cannot receive data from PWMSS");
    }
    else
    {
        if ((uint8_t) readBuf[3] != 1)
        {
            syslog(LOG_INFO,
                   "mydrone: MyMotorsAgent ERROR: PWMSSs response=%d:%d:%d:%d\n",
                   (uint8_t) readBuf[0], (uint8_t) readBuf[1],
                   (uint8_t) readBuf[2], (uint8_t) readBuf[3]);
            result = false;
        }
    }
    return result;
}
