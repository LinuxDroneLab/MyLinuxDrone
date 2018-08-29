/*
 * BMP085.cpp
 *
 *  Created on: 14 set 2015
 *      Author: andrea
 */
/****************************************************************************
 * BMP085.cpp - BMP085/I2C (Digital Pressure Sensor) library for Arduino     *
 * Copyright 2010-2012 Filipe Vieira & various contributors                  *
 *                                                                           *
 * This file is part of BMP085 Arduino library.                              *
 *                                                                           *
 * This library is free software: you can redistribute it and/or modify      *
 * it under the terms of the GNU Lesser General Public License as published  *
 * by the Free Software Foundation, either version 3 of the License, or      *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU Lesser General Public License for more details.                       *
 *                                                                           *
 * You should have received a copy of the GNU Lesser General Public License  *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
 ****************************************************************************/
/****************************************************************************
 * Tested on Arduino Mega with BMP085 Breakout                               *
 * SDA   -> pin 20   (no pull up resistors)                                  *
 * SCL   -> pin 21   (no pull up resistors)                                  *
 * XCLR  -> not connected                                                    *
 * EOC   -> not connected                                                    *
 * GND   -> pin GND                                                          *
 * VCC   -> pin 3.3V                                                         *
 * NOTE: SCL and SDA needs pull-up resistors for each I2C bus.               *
 *  2.2kOhm..10kOhm, typ. 4.7kOhm                                            *
 *****************************************************************************/
#include <inttypes.h>
#include <baro/BMP085.h>
#include <i2c/I2Cdev.h>
#include <unistd.h>
#include <cmath>
#include <syslog.h>

BMP085::BMP085() {
	// temperature, pressure, altitude
	data.altitude = 0.0f;
	data.dtimeMillis = 0;
	data.estimatedAltitude = 0.0f;
	data.pressure = 0.0f;
	data.rawPressure = 0;
	data.rawTemperature = 0;
	data.seaLevelPressure = BMP085__SEALEVEL_PRESSURE;
	data.speedMetersPerSeconds = 0.0f;
	data.temperature = 0.0f;
	data.timestamp = boost::posix_time::microsec_clock::local_time();

	// calibration parameters
	ac1 = 0;
	ac2 = 0;
	ac3 = 0;
	ac4 = 0;
	ac5 = 0;
	ac6 = 0;
	b1 = 0;
	b2 = 0;
	mb = 0;
	mc = 0;
	md = 0;
	oversampling = 0;
	status = none;
	this->discardSamples = 3000;
	this->pressureCycles = 0;
	this->pressureValueFast = 0.0f;
	this->pressureValueSlow = 0.0f;
	this->pressureBufferPosition = 0.0f;
    for(uint8_t i = 0; i < BMP085_MAX_PRESSURE_CYCLES; i++) {
        pressureBuffer[i] = 0;
    }
    data.seaLevelPressure = 101500;

}
BMP085::~BMP085() {
}

// state machine based on ticks
bool BMP085::pulse() {
	bool result = false;
	if(initialized || status == none || status == waitStartup) { // initialized only after begin() on waitStartup
		switch (status) {
		case none: {
			changeStatus(waitStartup);
			break;
		}
		case waitStartup: {
			// 10 millis
			if (calcMillisFrom(statusAtTime) >= 10) {
				begin(BMP085_ULTRAHIGHRES);
				startCycle();
				changeStatus(requireTemperature);
			}
			break;
		}
        case requireTemperature: {
            sendRawTemperatureCmd();
            changeStatus(waitTemperature);
            break;
        }
		case waitTemperature: {
			// 5 millis
			if (calcMillisFrom(statusAtTime) >= 5) {
				loadRawTemperature();
				changeStatus(requirePressure);
			}
			break;
		}
        case requirePressure: {
            sendRawPressureCmd();
            changeStatus(waitPressure);
            break;
        }
		case waitPressure: {
			uint16_t waitMillis = 0;
			if (oversampling == BMP085_ULTRALOWPOWER)
				waitMillis = 5;
			else if (oversampling == BMP085_STANDARD)
				waitMillis = 8;
			else if (oversampling == BMP085_HIGHRES)
				waitMillis = 14;
			else
				waitMillis = 26;

			if (calcMillisFrom(statusAtTime) >= waitMillis) {
			    boost::posix_time::ptime prev = data.timestamp;
			    data.timestamp = boost::posix_time::microsec_clock::local_time();
			    data.dtimeMillis = data.timestamp.time_of_day().total_milliseconds() - prev.time_of_day().total_milliseconds();

				loadRawPressure();
				calcTemperature();
				calcPressure();
				calcAltitude(data.seaLevelPressure);
//				calcSealevelPressure();
				if(this->discardSamples == 0) {
	                result = true;
				} else {
				    this->discardSamples--;
				}
				this->pressureCycles++;
                this->pressureCycles %= BMP085_MAX_PRESSURE_CYCLES;
                if(this->pressureCycles == 0) {
                    changeStatus(requireTemperature);
                } else {
                    changeStatus(requirePressure);
                }
			}
			break;
		}
		}
	}
	return result; // true if update data (cycle complete)
}
BMP085::SensorData const & BMP085::getData() const {
	return data;
}
uint64_t BMP085::calcMillisFrom(
		std::chrono::time_point<std::chrono::system_clock> since) {
	auto duration = std::chrono::system_clock::now() - since;
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
			duration).count();
	return millis;
}
void BMP085::startCycle() {
	cycleAtTime = std::chrono::system_clock::now();
}
void BMP085::changeStatus(SensorStatus s) {
	statusAtTime = std::chrono::system_clock::now();
	status = s;

}
bool BMP085::begin(uint8_t mode) {
	if (mode > BMP085_ULTRAHIGHRES)
		mode = BMP085_ULTRAHIGHRES;
	oversampling = mode;
	if (read8(0xD0) != 0x55)
		return initialized = false;

	/* read calibration data */
	ac1 = read16(BMP085_CAL_AC1);
	ac2 = read16(BMP085_CAL_AC2);
	ac3 = read16(BMP085_CAL_AC3);
	ac4 = read16(BMP085_CAL_AC4);
	ac5 = read16(BMP085_CAL_AC5);
	ac6 = read16(BMP085_CAL_AC6);

	b1 = read16(BMP085_CAL_B1);
	b2 = read16(BMP085_CAL_B2);

	mb = read16(BMP085_CAL_MB);
	mc = read16(BMP085_CAL_MC);
	md = read16(BMP085_CAL_MD);

	return initialized = true;
}

int32_t BMP085::computeB5(int32_t UT) {
	int32_t X1 = (UT - (int32_t) ac6) * ((int32_t) ac5) >> 15;
	int32_t X2 = ((int32_t) mc << 11) / (X1 + (int32_t) md);
	return X1 + X2;
}
void BMP085::sendRawTemperatureCmd(void) {
	write8(BMP085_CONTROL, BMP085_READTEMPCMD);
}
void BMP085::loadRawTemperature(void) {
	data.rawTemperature = read16(BMP085_TEMPDATA);
}
void BMP085::sendRawPressureCmd(void) {
	write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
}

void BMP085::loadRawPressure(void) {
	uint32_t raw;
	raw = read16(BMP085_PRESSUREDATA);

	raw <<= 8;
	raw |= read8(BMP085_PRESSUREDATA + 2);
	raw >>= (8 - oversampling);
	data.rawPressure = raw;
//	if(data.rawPressure != 0) {
//	    data.rawPressure = 0.75f * data.rawPressure + 0.25f * raw;
//	} else {
//	    data.rawPressure = raw;
//	}
}

void BMP085::calcPressure() {
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;

	UT = data.rawTemperature;
	UP = data.rawPressure;
	B5 = computeB5(UT);
	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t) b2 * ((B6 * B6) >> 12)) >> 11;
	X2 = ((int32_t) ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t) ac1 * 4 + X3) << oversampling) + 2) / 4;
	X1 = ((int32_t) ac3 * B6) >> 13;
	X2 = ((int32_t) b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t) ac4 * (uint32_t) (X3 + 32768)) >> 15;
	B7 = ((uint32_t) UP - B3) * (uint32_t) (50000UL >> oversampling);
	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p = p + ((X1 + X2 + (int32_t) 3791) >> 4);

	this->pushPressure(p);
	this->calcPressureFast();
    this->calcPressureSlow();

	data.pressure = this->pressureValueSlow;
}
void BMP085::calcTemperature(void) {
	int32_t UT, B5;     // following ds convention
	float temp;

	UT = data.rawTemperature;

	B5 = computeB5(UT);
	temp = (B5 + 8) >> 4;
	temp /= 10;

	data.temperature = temp;
}
void BMP085::calcAltitude(int32_t sealevelPressure) {
	float newAltitude = 44330.0f
			* (1.0f - pow((data.pressure) / float(sealevelPressure), 0.1903f));
	data.altitude = data.altitude * 0.80f + newAltitude * 0.20f;
//    data.altitude = (sealevelPressure - data.pressure) / 100 * 8.43f;
}
void BMP085::calcSealevelPressure() {
    float calc = (data.pressure / pow(1.0f - data.altitude / 44330.0f, 5.255f));
	data.seaLevelPressure = int32_t(calc);
}

uint8_t BMP085::read8(uint8_t a) {
	uint8_t ret;
	readmem(a, 1, &ret);
	return ret;
}
void BMP085::write8(uint8_t a, uint8_t d) {
	writemem(a, d);
}
uint16_t BMP085::read16(uint8_t a) {
	uint16_t ret;
	uint8_t __buff[2] = { 0, 0 };
	readmem(a, 2, __buff);
	ret = ((uint16_t) __buff[0]) << 8 | ((uint16_t) __buff[1]);
	return ret;
}
void BMP085::writemem(uint8_t _addr, uint8_t _val) {
	I2Cdev::writeByte(BMP085_I2CADDR, _addr, _val);
}

void BMP085::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
	I2Cdev::readBytes(BMP085_I2CADDR, _addr, _nbytes, __buff);
}
void BMP085::pushPressure(int32_t pressure) {
    this->pressureBuffer[this->pressureBufferPosition] = pressure;
    this->pressureBufferPosition++;
    this->pressureBufferPosition %= BMP085_MAX_PRESSURE_CYCLES;
}
void BMP085::calcPressureSlow() {
    this->pressureValueSlow = this->pressureValueSlow * 0.985 + this->pressureValueFast * 0.015;

    float pressureDiff = std::max<float>(-8.0f, std::min<float>(8.0f, this->pressureValueSlow - this->pressureValueFast));
    if(pressureDiff < -1 || pressureDiff > 1) {
        this->pressureValueSlow -= pressureDiff / 6.0f;
    }

}
void BMP085::calcPressureFast() {
    uint8_t samples = 0;
    int32_t sum = 0;
    int32_t sample = 0;
    for(uint8_t i = 0; i < BMP085_MAX_PRESSURE_CYCLES; i++) {
        sample=this->pressureBuffer[(this->pressureBufferPosition + i) % BMP085_MAX_PRESSURE_CYCLES];
        if(sample == 0) {
            break;
        }
        samples++;
        sum += sample;
    }
    if(samples != 0) {
        this->pressureValueFast = sum/float(samples);
    } else {
        this->pressureValueFast = 0.0f;
    }
    if(this->pressureValueSlow == 0.0f) {
        this->pressureValueSlow = this->pressureValueFast;
    }
}


