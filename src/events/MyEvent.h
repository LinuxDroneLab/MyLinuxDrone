/*
 * MyEvent.h
 *
 *  Created on: 12 dic 2015
 *      Author: andrea
 */

#ifndef MYEVENT_H_
#define MYEVENT_H_

#include <iostream>
#include <boost/uuid/uuid.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;

class MyEvent {
public:
	explicit MyEvent(boost::uuids::uuid origin);
	virtual ~MyEvent();
	enum class EventType {
		Alarm,
		BaroSample,
		EndAccellCalCmd,
		EndBaroCalCmd,
		EndCompassCalCmd,
		EndGyroCalCmd,
		IMUSample,
		RCSample,
		ShutdownCmd,
		ShutdownImmCmd,
		StartAccellCalCmd,
		StartBaroCalCmd,
		StartCmd,
		StartCompassCalCmd,
		StartGyroCalCmd,
		StopCmd,
		StopImmCmd,
		TargetSample,
		ChangeState,
		Tick,
		OutMotors,
		YPRError,
		ArmMotorsCmd,
		DisarmMotorsCmd,
		MotorsArmed,
		MotorsDisarmed
	} ;

	friend ostream& operator<<(ostream& ostr, const MyEvent& event);
	friend istream& operator>>(istream& istr,const MyEvent& event);
	boost::uuids::uuid getUuid() const;
	boost::posix_time::ptime getTimestamp() const;
	long getTimestampMillis() const;
	uint getPriority();
	void setPriority(uint priority);
	boost::uuids::uuid getOrigin();
	virtual MyEvent::EventType getType() const = 0;
	uint getLivingTimeMillis() const;
	void setLivingTimeMillis(uint millis);
	bool isExpired() const;
	virtual bool isCommand() const;
protected:

private:
	boost::uuids::uuid uuid;
	boost::posix_time::ptime timestamp;
	boost::uuids::uuid origin; // agent uuid
	uint priority;
	uint livingTimeMillis;
};

#endif /* MYEVENT_H_ */
