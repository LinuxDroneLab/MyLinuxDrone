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
#include <agents/MyMotorsAgent.h>
#include <agents/MyDBusEmitterAgent.h>
#include <agents/MyClock.h>
#include <bus/MyEventBus.h>
#include <i2c/I2Cdev.h>
#include <imu/MPU6050.h>
#include <dbus/MyPIDControllerServiceWrap.h>

using namespace std;

typedef boost::signals2::signal<void(boost::shared_ptr<MyEvent>)> SignalType;
boost::uuids::uuid uuid;

int main() {
	boost::uuids::uuid uuidMain;
	uuid = uuidMain;
	cout << "Start Agent Application" << endl;
	cout << "long: " << sizeof(long long) << ", int: " << sizeof(int) << ", short: " << sizeof(short) << endl;
	// create eventBus
	boost::shared_ptr<MyEventBus> eventBus(boost::make_shared<MyEventBus>());


	MyClock myClock(eventBus, 5);
	MyIMUAgent imuAgent(eventBus, {MyEvent::EventType::Tick});
	MyPIDControllerAgent pidControlledAgent(eventBus, {MyEvent::EventType::IMUSample});
	MyMotorsAgent motorsAgent(eventBus, {MyEvent::EventType::ArmMotorsCmd, MyEvent::EventType::DisarmMotorsCmd, MyEvent::EventType::OutMotors});
	MyDBusEmitterAgent dbusAgent(eventBus, {MyEvent::EventType::YPRError});


	boost::thread imuAgentThr(boost::ref(imuAgent));
	boost::thread pidControllerAgentThr(boost::ref(pidControlledAgent));
	boost::thread motorsAgentThr(boost::ref(motorsAgent));
	boost::thread myDBusThread(boost::ref(dbusAgent));
	boost::thread myClockThread(boost::ref(myClock));

	// arm motors
	boost::shared_ptr<MyArmMotorsCmd> armMotors(boost::make_shared<MyArmMotorsCmd>(uuid, motorsAgent.getUuid()));
	eventBus->doEvent(armMotors);

	MyPIDControllerServiceWrap::initialize();
	cout << "Initialized: org.mydrone.MyPIDControllerService" << endl;

	// wait for exit;
	imuAgentThr.join();
	pidControllerAgentThr.join();
	motorsAgentThr.join();
	myDBusThread.join();
	myClockThread.join();
	return 0;
}
