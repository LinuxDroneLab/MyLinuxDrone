/*
 * MyRCReader.cpp
 *
 *  Created on: 23 lug 2016
 *      Author: andrea
 */

#include <rc/MyRCReader.h>
#include <agents/MyRCAgent.h>

#include <stdint.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <syslog.h>

#define PRU_NUM 0
#define CHAN_THRUST 1
#define CHAN_ROLL 0
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_AUX1 4
#define CHAN_AUX2 5

static void *pru0DataMemory;
static unsigned int *pru0DataMemory_int;
RangeInt16 MyRCReader::PRU_RANGES[] = { RangeInt16(602, 1584), RangeInt16(611,
		1578), RangeInt16(665, 1535), RangeInt16(612, 1572), RangeInt16(597,
		1589), RangeInt16(597, 1588) };
RangeInt16 MyRCReader::CHAN_RANGES[] = { RangeInt16(-500, 500), RangeInt16(-500,
		500), RangeInt16(-500, 500), RangeInt16(-500, 500), RangeInt16(-500,
		500), RangeInt16(-500, 500) };
ValueInt16 MyRCReader::PRU_VALUES[] = { ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_ROLL]), ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_THRUST]), ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_PITCH]), ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_YAW]), ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_AUX1]), ValueInt16(0,
		MyRCReader::PRU_RANGES[CHAN_AUX2]) };
ValueInt16 MyRCReader::CHAN_VALUES[] = { ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_ROLL]), ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_THRUST]), ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_PITCH]), ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_YAW]), ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_AUX1]), ValueInt16(0,
		MyRCReader::CHAN_RANGES[CHAN_AUX2]) };

MyRCReader::ValueTableType MyRCReader::valueTable[10];

MyRCReader::MyRCReader() :
		initialized(false), active(false), agent(NULL) {
	this->prevRoll = 0;
	this->prevPitch = 0;
	this->prevYaw = 0;
	this->prevAux1 = 0;
	this->prevAux2 = 0;
	this->prevThrust = 0;

	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 6; j++) {
			valueTable[i].value[j] = 0;
			valueTable[i].freq[j] = 0;
			valueTable[i].range[j] = 0.0f;
		}
	}
}

MyRCReader::~MyRCReader() {
}

void MyRCReader::setActive(bool active) {
	this->active = active;
}
bool MyRCReader::isActive() {
	return this->active;
}
void MyRCReader::operator()() {
	syslog(LOG_INFO, "mydrone: PRU: thread MyRCReader started");
	uint8_t cycle = 0;
	do {
//		   boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();

		int notimes = prussdrv_pru_wait_event(PRU_EVTOUT_1);
		uint16_t chanRollValue = uint16_t((*(pru0DataMemory_int + CHAN_ROLL)) / 100);
		uint16_t chanThrustValue = uint16_t((*(pru0DataMemory_int + CHAN_THRUST)) / 100);
		uint16_t chanPitchValue = uint16_t((*(pru0DataMemory_int + CHAN_PITCH)) / 100);
		uint16_t chanYawValue = uint16_t((*(pru0DataMemory_int + CHAN_YAW)) / 100);
		uint16_t chanAux1Value = uint16_t((*(pru0DataMemory_int + CHAN_AUX1)) / 100);
		uint16_t chanAux2Value = uint16_t((*(pru0DataMemory_int + CHAN_AUX2)) / 100);

		int pruclear = prussdrv_pru_clear_event(PRU_EVTOUT_1,
				PRU0_ARM_INTERRUPT);

			// save PRU values in microseconds
			MyRCReader::PRU_VALUES[CHAN_ROLL].setValue(chanRollValue);
			MyRCReader::PRU_VALUES[CHAN_PITCH].setValue(chanPitchValue);
			MyRCReader::PRU_VALUES[CHAN_YAW].setValue(chanYawValue);
			MyRCReader::PRU_VALUES[CHAN_THRUST].setValue(chanThrustValue);
			MyRCReader::PRU_VALUES[CHAN_AUX1].setValue(chanAux1Value);
			MyRCReader::PRU_VALUES[CHAN_AUX2].setValue(chanAux2Value);

			// convert PRU values to CHAN values ([-500, 500]
			MyRCReader::CHAN_VALUES[CHAN_ROLL].setValue(
					MyRCReader::PRU_VALUES[CHAN_ROLL]);
			MyRCReader::CHAN_VALUES[CHAN_PITCH].setValue(
					MyRCReader::PRU_VALUES[CHAN_PITCH]);
			MyRCReader::CHAN_VALUES[CHAN_YAW].setValue(
					MyRCReader::PRU_VALUES[CHAN_YAW]);
			MyRCReader::CHAN_VALUES[CHAN_THRUST].setValue(
					MyRCReader::PRU_VALUES[CHAN_THRUST]);
			MyRCReader::CHAN_VALUES[CHAN_AUX1].setValue(
					MyRCReader::PRU_VALUES[CHAN_AUX1]);
			MyRCReader::CHAN_VALUES[CHAN_AUX2].setValue(
					MyRCReader::PRU_VALUES[CHAN_AUX2]);

			// send CHAN values as percent to agent
			((MyRCAgent*) agent)->setRCSample(
					MyRCReader::CHAN_VALUES[CHAN_THRUST].getValueAsPercent(),
					MyRCReader::CHAN_VALUES[CHAN_ROLL].getValueAsPercent(),
					MyRCReader::CHAN_VALUES[CHAN_PITCH].getValueAsPercent(),
					MyRCReader::CHAN_VALUES[CHAN_YAW].getValueAsPercent(),
					MyRCReader::CHAN_VALUES[CHAN_AUX1].getValueAsPercent(),
					MyRCReader::CHAN_VALUES[CHAN_AUX2].getValueAsPercent());

		if (cycle == 0) {
//			syslog(LOG_INFO, "mydrone: PRU: roll=%d, pitch=%d, yaw=%d, aux1=%d, aux2=%d, thr=%d", MyRCReader::PRU_VALUES[CHAN_ROLL].getValue(), MyRCReader::PRU_VALUES[CHAN_PITCH].getValue(), MyRCReader::PRU_VALUES[CHAN_YAW].getValue(), MyRCReader::PRU_VALUES[CHAN_AUX1].getValue(), MyRCReader::PRU_VALUES[CHAN_AUX2].getValue(), MyRCReader::PRU_VALUES[CHAN_THRUST].getValue());
//			syslog(LOG_INFO, "mydrone: PRU: roll=%d, pitch=%d, yaw=%d, aux1=%d, aux2=%d, thr=%d", chanRollValue, chanPitchValue, chanYawValue, chanAux1Value, chanAux2Value, chanThrustValue);
		}

		// if(MyRCReader::CHAN_VALUES[CHAN_PITCH].getValueAsPercent() < 0) {
//		  	  cout << "PS: P: " << currPitch << ", "<< MyRCReader::PRU_VALUES[CHAN_PITCH].getValue() << " - C: " << MyRCReader::CHAN_VALUES[CHAN_PITCH].getValue() << endl;
		//}

		// boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		cycle++;
		cycle %= 10;

	} while (this->active);

	this->releasePRU();
}

void MyRCReader::setValue(int8_t chan, int16_t value) {
	for (int i = 0; i < 10; i++) {
		int16_t diff = abs(valueTable[i].value[chan] - value);
		if (diff < valueTable[i].range[chan]) {
			valueTable[i].freq[chan]++;
			return;
		}
	}
	for (int i = 0; i < 10; i++) {
		if (valueTable[i].value[chan] == 0) {
			valueTable[i].value[chan] = value;
			valueTable[i].freq[chan] = 1;
			valueTable[i].range[chan] = float(value) * 0.003f;
//			if(chan == CHAN_PITCH) {
//				std::cout<< "setV: v=" << valueTable[i].value[chan] << ", r=" << valueTable[i].range[chan] << std::endl;
//			}
			return;
		}
	}
}

int16_t MyRCReader::getValue(int8_t chan) {
	int8_t maxFreq = 0;
	int16_t selectedValue = 0;
	for (int i = 0; i < 10; i++) {
		if (valueTable[i].freq[chan] > maxFreq) {
			maxFreq = valueTable[i].freq[chan];
			selectedValue = valueTable[i].value[chan];
		}
	}
//	if(chan == CHAN_PITCH) {
//		std::cout<< "getV: v=" << selectedValue << ", freq=" << int16_t(maxFreq) << std::endl;
//	}
	for (int i = 0; i < 10; i++) {
		if (valueTable[i].value[chan] != selectedValue) {
			valueTable[i].freq[chan] = 0;
			valueTable[i].value[chan] = 0;
			valueTable[i].range[chan] = 0.0f;
		} else {
			valueTable[i].freq[chan] = 0;
		}
	}
	return selectedValue;
}
bool MyRCReader::initPRU() {
	syslog(LOG_INFO, "mydrone: PRU: initializing ...");

	if (getuid() != 0) {
		printf("You must run this program as root. Exiting.\n");
		exit(EXIT_FAILURE);
	}
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	// Allocate and initialize memory
	prussdrv_init();
	prussdrv_open (PRU_EVTOUT_0);
	prussdrv_open (PRU_EVTOUT_1);

	// Map PRU's INTC
	prussdrv_pruintc_init(&pruss_intc_initdata);

	// Copy data to PRU memory - different way
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0DataMemory);

	pru0DataMemory_int = (unsigned int *) pru0DataMemory;

	// initialize data
	int i = 0;
	for (i = 0; i < 10; i++) {
		*(pru0DataMemory_int + i) = 0;
	}

	// Load and execute binary on PRU
	syslog(LOG_INFO, "mydrone: PRU: loading program ...");
	prussdrv_exec_program(PRU_NUM, "/root/MyDrone/provaPRU.bin");
	syslog(LOG_INFO, "mydrone: PRU: program loaded");
	prussdrv_pru_enable(PRU_NUM);
	syslog(LOG_INFO, "mydrone: PRU: initialized");

	return true;
}

void MyRCReader::releasePRU() {
	/* Disable PRU and close memory mappings */
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit();

}

bool MyRCReader::initialize(void* agent) {
	if (!initialized) {
		this->agent = agent;
		this->active = true;
		if (this->initPRU()) {
			initialized = true;
		}
	}
	return initialized;
}
