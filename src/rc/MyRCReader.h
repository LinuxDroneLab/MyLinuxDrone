/*
 * MyRCReader.h
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#ifndef RC_MYRCREADER_H_
#define RC_MYRCREADER_H_

#include <commons/ValueInt16.h>

class MyRCReader {
public:
	MyRCReader();
	virtual ~MyRCReader();
	virtual void operator()();
	bool initPRU();
	void releasePRU();

	bool initialize(void* agent);
	void setActive(bool active);
	bool isActive();
private:
	bool initialized;
	bool active;
	void* agent;
    static RangeInt16 PRU_RANGES[];
    static RangeInt16 CHAN_RANGES[];
    static ValueInt16 PRU_VALUES[];
    static ValueInt16 CHAN_VALUES[];
};

#endif /* RC_MYRCREADER_H_ */
