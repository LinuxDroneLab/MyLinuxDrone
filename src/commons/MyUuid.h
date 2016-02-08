/*
 * MyUuid.h
 *
 *  Created on: 13 dic 2015
 *      Author: andrea
 */

#ifndef MYUUID_H_
#define MYUUID_H_

#include <boost/thread/mutex.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

class MyUuid {
public:
	MyUuid();
	virtual ~MyUuid();
	static boost::uuids::uuid generateUuid();

private:
	static boost::mutex rndGenMutex;
	static boost::uuids::random_generator rndGen;

};

#endif /* MYUUID_H_ */
