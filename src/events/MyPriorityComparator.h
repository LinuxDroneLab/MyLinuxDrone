/*
 * MyPriorityComparator.h
 *
 *  Created on: 14 dic 2015
 *      Author: andrea
 */

#ifndef MYPRIORITYCOMPARATOR_H_
#define MYPRIORITYCOMPARATOR_H_
#include <boost/shared_ptr.hpp>
#include "events/MyEvent.h"

using namespace std;

class MyPriorityComparator {
public:
	MyPriorityComparator();
	virtual ~MyPriorityComparator();
	bool operator()(const boost::shared_ptr<MyEvent>& lhs, const boost::shared_ptr<MyEvent>& rhs) const;
};

#endif /* MYPRIORITYCOMPARATOR_H_ */
