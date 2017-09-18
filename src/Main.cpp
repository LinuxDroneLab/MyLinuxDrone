//============================================================================
// Name        : MyDrone.cpp
// Author      : Andrea Lambruschini
// Version     :
// Copyright   : hi!
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <events/MyArmMotorsCmd.h>
#include <agents/MyIMUAgent.h>
#include <agents/MyPIDControllerAgent.h>
#include <agents/MyPIDCntrllr.h>
#include <agents/MyMotorsAgent.h>
#include <agents/MyDBusEmitterAgent.h>
#include <agents/MyClock.h>
#include <agents/MyRCAgent.h>
#include <bus/MyEventBus.h>
#include <i2c/I2Cdev.h>
#include <imu/MPU6050.h>
#include <dbus/MyPIDControllerServiceWrap.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>
#include <cfenv>
using namespace std;

#define DAEMON_NAME "mydrone"

typedef boost::signals2::signal<void(boost::shared_ptr<MyEvent>)> SignalType;
boost::uuids::uuid uuid;

int main() {
#pragma STDC FENV_ACCESS ON
    std::fesetround(FE_TONEAREST);
    openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);

    syslog(LOG_INFO, "Entering Daemon");

    pid_t pid, sid;

   //Fork the Parent Process
    pid = fork();

    if (pid < 0) { exit(EXIT_FAILURE); }

    //We got a good pid, Close the Parent Process
    if (pid > 0) { exit(EXIT_SUCCESS); }

    //Change File Mask
    umask(0);

    //Create a new Signature Id for our child
    sid = setsid();
    if (sid < 0) { exit(EXIT_FAILURE); }

    //Change Directory
    //If we cant find the directory we exit with failure.
    // if ((chdir("/")) < 0) { exit(EXIT_FAILURE); }

    //Close Standard File Descriptors
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);

    syslog(LOG_INFO, "mydrone Daemon started");

	boost::uuids::uuid uuidMain;
	uuid = uuidMain;
//	cout << "Start Agent Application" << endl;
//	cout << "long: " << sizeof(long long) << ", int: " << sizeof(int) << ", short: " << sizeof(short) << endl;
	// create eventBus
	boost::shared_ptr<MyEventBus> eventBus(boost::make_shared<MyEventBus>());


	MyClock myClock(eventBus, 5);
	MyIMUAgent imuAgent(eventBus, {MyEvent::EventType::Tick});
	MyPIDCntrllr pidControlledAgent(eventBus, {MyEvent::EventType::MotorsArmed, MyEvent::EventType::MotorsDisarmed, MyEvent::EventType::IMUSample, MyEvent::EventType::RCSample, MyEvent::EventType::BaroSample});
	MyMotorsAgent motorsAgent(eventBus, {MyEvent::EventType::MinThrustMaxPitch, MyEvent::EventType::MinThrustMinPitch, MyEvent::EventType::OutMotors});
    MyRCAgent rcAgent(eventBus, {MyEvent::EventType::Tick});

	MyDBusEmitterAgent dbusAgent(eventBus, {MyEvent::EventType::YPRError});

	boost::thread imuAgentThr(boost::ref(imuAgent));
	boost::thread rcAgentThr(boost::ref(rcAgent));
	boost::thread pidControllerAgentThr(boost::ref(pidControlledAgent));
	boost::thread motorsAgentThr(boost::ref(motorsAgent));
	boost::thread myDBusThread(boost::ref(dbusAgent));
	boost::thread myClockThread(boost::ref(myClock));

	// arm motors
	boost::shared_ptr<MyArmMotorsCmd> armMotors(boost::make_shared<MyArmMotorsCmd>(uuid, motorsAgent.getUuid()));
	eventBus->doEvent(armMotors);

	MyPIDControllerServiceWrap::initialize();
//	cout << "Initialized: org.mydrone.MyPIDControllerService" << endl;

	// wait for exit;
	imuAgentThr.join();
	rcAgentThr.join();
	pidControllerAgentThr.join();
	motorsAgentThr.join();
	myDBusThread.join();
	myClockThread.join();

    syslog(LOG_INFO, "mydrone Daemon bye bye");

	closelog();
	return 0;
}
