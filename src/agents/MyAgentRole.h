/*
 * MyAgentRole.h
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYAGENTROLE_H_
#define AGENTS_MYAGENTROLE_H_

enum class MyAgentRole
	: unsigned short {
		IMUDriver,
	MotorsDriver,
	Controller,
	RCDriver,
	TargetDetector,
	ErrorDetector,
	Generic
};

#endif /* AGENTS_MYAGENTROLE_H_ */
