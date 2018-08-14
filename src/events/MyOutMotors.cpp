/*
 * MyOutMotors.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyOutMotors.h>

MyOutMotors::MyOutMotors(boost::uuids::uuid origin, uint16_t front, uint16_t rear, uint16_t left, uint16_t right) : MyEvent(origin), front(front), rear(rear), left(left), right(right) {
	this->setPriority(MyPriority::OUT_MOTORS_PRIORITY);

}

MyOutMotors::~MyOutMotors() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyOutMotors::getType() const {
	return MyEvent::EventType::OutMotors;
}

uint16_t MyOutMotors::getFront() const {
	return front;
}
uint16_t MyOutMotors::getRear() const {
	return rear;
}
uint16_t MyOutMotors::getLeft() const {
	return left;
}
uint16_t MyOutMotors::getRight() const {
	return right;
}
