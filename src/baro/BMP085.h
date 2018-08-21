/****************************************************************************
 * BMP085.h - BMP085/I2C (Digital Pressure Sensor) library for Arduino       *
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
#ifndef BMP085_h
#define BMP085_h
#include <stdint.h>
#include <chrono>
#include "boost/date_time/posix_time/posix_time.hpp"

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3
#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD          0x2E
#define BMP085_READPRESSURECMD            0x34
#define BMP085__SEALEVEL_PRESSURE 101600
class BMP085 {
public:
	BMP085();
	virtual ~BMP085();
	typedef struct {
		uint16_t rawTemperature;
		uint32_t rawPressure;
		float temperature; // gradi centigradi
		int32_t pressure; // Pa
		int32_t seaLevelPressure; // Pa
		float altitude; // meters
		float estimatedAltitude; //meters
		boost::posix_time::ptime timestamp;
		uint16_t dtimeMillis;
		float speedMetersPerSeconds;
	} SensorData;

	bool begin(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go ultrahighres
	SensorData const & getData() const;
	bool pulse();

private:
	typedef enum {none, waitStartup, waitTemperature, waitPressure} SensorStatus;

	bool initialized = false;
	SensorData data;
	SensorStatus status = none;
	std::chrono::time_point<std::chrono::system_clock>  statusAtTime;
	std::chrono::time_point<std::chrono::system_clock>  cycleAtTime;
	boost::posix_time::ptime pulseAtTime;
	uint64_t calcMillisFrom(std::chrono::time_point<std::chrono::system_clock> since);


	void startCycle();
	void changeStatus(SensorStatus s);
	void sendRawTemperatureCmd(void);
	void sendRawPressureCmd(void);
	void loadRawTemperature(void);
	void loadRawPressure(void);
	void calcTemperature(void);
	void calcPressure(void);
	void calcAltitude(float sealevelPressure = BMP085__SEALEVEL_PRESSURE); // std atmosphere
	void calcSealevelPressure();
	int32_t computeB5(int32_t UT);
	uint8_t read8(uint8_t addr);
	uint16_t read16(uint8_t addr);
	void write8(uint8_t addr, uint8_t data);
	void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);
	void writemem(uint8_t _addr, uint8_t _val);

	uint8_t oversampling;
	uint16_t discardSamples;

	int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
	uint16_t ac4, ac5, ac6;
};

#endif // BMP085_h
