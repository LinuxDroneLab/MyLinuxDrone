/*
 * MotorDriver.cpp
 *
 *  Created on: 29 dic 2015
 *      Author: andrea
 */

#include <motors/MotorDriver.h>
#include <syslog.h>

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
	syslog(LOG_INFO, "mydrone: MotorDriver: initializing pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
//	{ // set pinmux as 'pwm'
//		ofstream fout(pinmuxPath + "/state",ios::out);
//		if(fout.is_open()) {
//			fout << "pwm";
//			fout.close();
//		} else {
//			syslog(LOG_ERR, "mydrone: MotorDriver: cannot open pwm stream. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
//			return false;
//		}
//	}
	{ // export device
		ofstream fout(pwmchipPath + "/export",ios::out);
		if(fout.is_open()) {
			fout << std::to_string(deviceNum);
			fout.close();
			syslog(LOG_INFO, "mydrone: MotorDriver: pwm pin exported. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
		} else {
			syslog(LOG_ERR, "mydrone: MotorDriver: cannot export pwm pin. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
			return false;
		}
	}

	{ // set period
		ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/period",ios::out);
		for(int i = 0; (i < 100) && (!fout.is_open()); i++) {
	        ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/period",ios::out);
		}
		if(fout.is_open()) {
			fout << std::to_string(20000000);
			fout.close();
			syslog(LOG_INFO, "mydrone: MotorDriver: pwm period initialized. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
		} else {
			syslog(LOG_ERR, "mydrone: MotorDriver: cannot set pwm period. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
			return false;
		}
	}
	{ // set duty_cicle
		ofstream duty(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out);
//		boost::shared_ptr<ofstream> duty(boost::make_shared<ofstream>(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out));

        for(int i = 0; (i < 100) && (!duty.is_open()); i++) {
            ofstream duty(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/duty_cycle",ios::out);
        }
		if(duty.is_open()) {
			duty << std::to_string(1000000);
			duty.close();
			syslog(LOG_INFO, "mydrone: MotorDriver: pwm duty cycle initialized. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
		} else {
			syslog(LOG_ERR, "mydrone: MotorDriver: cannot set initilize pwm duty cycle. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
			return false;
		}

	}
	{ // enable pin
		ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/enable",ios::out);
        for(int i = 0; (i < 100) && (!fout.is_open()); i++) {
            ofstream fout(pwmchipPath + "/pwm" + std::to_string(deviceNum) + "/enable",ios::out);
        }
		if(fout.is_open()) {
			fout << std::to_string(1);
			fout.close();
			syslog(LOG_INFO, "mydrone: MotorDriver: pwm enabled. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
		} else {
			syslog(LOG_ERR, "mydrone: MotorDriver: cannot enable pwm. pinmux: %s, pwmchip: %s, device: %d", pinmuxPath.c_str(), pwmchipPath.c_str(), uint8_t(deviceNum));
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
