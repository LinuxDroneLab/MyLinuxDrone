/*
 * MyOutMotors.cpp
 *
 *  Created on: 28 dic 2015
 *      Author: andrea
 */

#include <commons/MyPriority.h>
#include <events/MyOutMotors.h>

MyOutMotors::MyOutMotors(boost::uuids::uuid origin, int32_t front, int32_t rear, int32_t left, int32_t right) : MyEvent(origin), front(front), rear(rear), left(left), right(right) {
	this->setPriority(MyPriority::OUT_MOTORS_PRIORITY);

}

MyOutMotors::~MyOutMotors() {
	// TODO Auto-generated destructor stub
}
MyEvent::EventType MyOutMotors::getType() const {
	return MyEvent::EventType::OutMotors;
}

int32_t MyOutMotors::getFront() const {
	return front;
}
int32_t MyOutMotors::getRear() const {
	return rear;
}
int32_t MyOutMotors::getLeft() const {
	return left;
}
int32_t MyOutMotors::getRight() const {
	return right;
}
