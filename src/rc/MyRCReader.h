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
	struct ValueTableType {
		int16_t value[6];
		int8_t freq[6];
		float range[6];
	} ;
	bool initialized;
	bool active;
	void* agent;
    static RangeInt16 PRU_RANGES[];
    static RangeInt16 CHAN_RANGES[];
    static ValueInt16 PRU_VALUES[];
    static ValueInt16 PRU_VALUES_PREV[];
    static ValueInt16 CHAN_VALUES[];
    static ValueTableType valueTable[];

    int16_t prevRoll;
    int16_t prevPitch;
    int16_t prevYaw;
    int16_t prevAux1;
    int16_t prevAux2;
    int16_t prevThrust;
    void setValue(int8_t chan, int16_t val);
    int16_t getValue(int8_t chan);
};

#endif /* RC_MYRCREADER_H_ */
