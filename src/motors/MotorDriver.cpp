/*
 * MotorDriver.cpp
 *
 *  Created on: 29 dic 2015
 *      Author: andrea
 */

#include <motors/MotorDriver.h>

MotorDriver::MotorDriver(std::string pinmuxPath, std::string pwmchipPath, uint8_t deviceNum) : initialized(false), pinmuxPath(pinmuxPath), pwmchipPath(pwmchipPath), deviceNum(deviceNum){
}

MotorDriver::~MotorDriver() {
	ofstream duty(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out);
	if(duty.is_open()) {
		duty << std::to_string(1000000);
		duty.close();
	}
}

bool MotorDriver::initialize() {
	{ // set pinmux as 'pwm'
		ofstream fout(pinmuxPath + "/state",ios::out);
		if(fout.is_open()) {
			fout << "pwm";
			fout.close();
		} else {
			return false;
		}
	}
	{ // export device
		ofstream fout(pwmchipPath + "/export",ios::out);
		if(fout.is_open()) {
			fout << std::to_string(deviceNum);
			fout.close();
		} else {
			return false;
		}
	}

	{ // set period
		ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/period",ios::out);
		if(fout.is_open()) {
			fout << std::to_string(20000000);
			fout.close();
		} else {
			return false;
		}
	}
	{ // set duty_cicle
		ofstream duty(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out);
//		boost::shared_ptr<ofstream> duty(boost::make_shared<ofstream>(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out));

		if(duty.is_open()) {
			duty << std::to_string(1000000);
			duty.close();
		} else {
			return false;
		}

	}
	{ // enable pin
		ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/enable",ios::out);
		if(fout.is_open()) {
			fout << std::to_string(1);
			fout.close();
		} else {
			return false;
		}
	}
	initialized = true;
	return true;
}

bool MotorDriver::setDutyCycleNanos(uint32_t nanos) {
	if(!this->initialized) {
		if(!this->initialize()) {
			return false;
		}
	}
	if(nanos < 1000000) {
		nanos = 1000000;
	} else if(nanos > 2000000) {
		nanos = 2000000;
	}
	ofstream duty(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out);

	if(duty.is_open()) {
		duty << std::to_string(nanos);
		duty.close();
	} else {
		return false;
	}
	return true;
}
