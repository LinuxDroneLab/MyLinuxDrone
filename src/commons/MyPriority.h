/*
 * MyPriority.h
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#ifndef COMMONS_MYPRIORITY_H_
#define COMMONS_MYPRIORITY_H_
#include <sys/types.h>

class MyPriority {
public :

	static const uint MAX_PRIORITY = 0;

	// commands have all the same priority (ordered by timestamp)
	static const uint STOP_IMMEDIATELY_PRIORITY = 100;
	static const uint SHUTDOWN_IMMEDIATELY_PRIORITY = 100;
	static const uint STOP_PRIORITY = 100;
	static const uint SHUTDOWN_PRIORITY = 100;
	static const uint START_PRIORITY = 100;
	static const uint START_COMPASS_CALIBRATION_PRIORITY = 100;
	static const uint END_COMPASS_CALIBRATION_PRIORITY = 100;
	static const uint START_GYRO_CALIBRATION_PRIORITY = 100;
	static const uint END_GYRO_CALIBRATION_PRIORITY = 100;
	static const uint START_ACCEL_CALIBRATION_PRIORITY = 100;
	static const uint END_ACCEL_CALIBRATION_PRIORITY = 100;
	static const uint START_BARO_CALIBRATION_PRIORITY = 100;
	static const uint END_BARO_CALIBRATION_PRIORITY = 100;
	static const uint ARM_MOTORS_PRIORITY = 100;
	static const uint DISARM_MOTORS_PRIORITY = 100;


	// events
	static const uint ALARM_PRIORITY = 500;
	static const uint CHANGE_STATE_PRIORITY = 500;
	static const uint MOTORS_ARMED_PRIORITY = 500;
	static const uint MOTORS_DISARMED_PRIORITY = 500;

	static const uint TICK_PRIORITY = 600;
	static const uint OUT_MOTORS_PRIORITY = 600;
	static const uint YPR_ERROR_PRIORITY = 700;
	static const uint RC_SAMPLE_PRIORITY = 700;
	static const uint TARGET_SAMPLE_PRIORITY = 700;

	static const uint IMU_SAMPLE_PRIORITY = 800;
	static const uint BAROMETER_SAMPLE_PRIORITY = 900;

	static const uint MIN_PRIORITY = 9999;
};

#endif /* COMMONS_MYPRIORITY_H_ */
