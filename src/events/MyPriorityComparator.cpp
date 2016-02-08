/*
 * MyPriorityComparator.cpp
 *
 *  Created on: 14 dic 2015
 *      Author: andrea
 */

#include "MyPriorityComparator.h"

MyPriorityComparator::MyPriorityComparator() {
	// TODO Auto-generated constructor stub

}

MyPriorityComparator::~MyPriorityComparator() {
	// TODO Auto-generated destructor stub
}

bool MyPriorityComparator::operator()(const boost::shared_ptr<MyEvent> & lhs, const boost::shared_ptr<MyEvent>& rhs) const
{
  return (lhs->getPriority() > rhs->getPriority()) || ((lhs->getPriority() == rhs->getPriority()) && (lhs->getTimestamp() >= rhs->getTimestamp()));
}
