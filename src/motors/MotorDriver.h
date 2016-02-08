/*
 * MotorDriver.h
 *
 *  Created on: 29 dic 2015
 *      Author: andrea
 */

#ifndef MOTORS_MOTORDRIVER_H_
#define MOTORS_MOTORDRIVER_H_
#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <string>
using namespace std;

class MotorDriver {
public:
	MotorDriver(std::string pinmuxPath, std::string pwmchipPath, uint8_t deviceNum);
	virtual ~MotorDriver();
	bool initialize();
	bool setDutyCycleNanos(uint32_t nanos);
private:
	bool initialized;
	const string pinmuxPath;
	const string pwmchipPath;
	const uint8_t deviceNum;

	boost::shared_ptr<ofstream> dutyCycle;

};

#endif /* MOTORS_MOTORDRIVER_H_ */
