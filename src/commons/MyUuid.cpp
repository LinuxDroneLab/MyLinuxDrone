/*
 * MyUuid.cpp
 *
 *  Created on: 13 dic 2015
 *      Author: andrea
 */

#include "MyUuid.h"

MyUuid::MyUuid() {
}

MyUuid::~MyUuid() {
}
boost::mutex MyUuid::rndGenMutex;
boost::uuids::random_generator MyUuid::rndGen;

boost::uuids::uuid MyUuid::generateUuid() {
	boost::mutex::scoped_lock lk(rndGenMutex);
	return rndGen();
}

