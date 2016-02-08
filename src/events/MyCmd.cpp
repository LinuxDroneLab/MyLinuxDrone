/*
 * MyCmd.cpp
 *
 *  Created on: 17 dic 2015
 *      Author: andrea
 */

#include <iostream>
#include <commons/MyPriority.h>
#include <events/MyCmd.h>

MyCmd::MyCmd(boost::uuids::uuid origin, boost::uuids::uuid destination) : MyEvent(origin), destination(destination) {
}

MyCmd::~MyCmd() {
}

bool MyCmd::isCommand() const {
	return true;
}

boost::uuids::uuid MyCmd::getDestination() const {
	return this->destination;
}
