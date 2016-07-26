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


#define PRU_NUM 0
#define CHAN_THRUST 1
#define CHAN_ROLL 0
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_AUX1 4
#define CHAN_AUX2 5

static void *pru0DataMemory;
static unsigned int *pru0DataMemory_int;
RangeInt16 MyRCReader::PRU_RANGES[] = {RangeInt16(994,1988), RangeInt16(996,1966),
		                                      RangeInt16(1050,1944), RangeInt16(1013,1987),
											  RangeInt16(995,1987), RangeInt16(995,1986)
};
RangeInt16 MyRCReader::CHAN_RANGES[] = {RangeInt16(-500,500), RangeInt16(-500,500),
                                               RangeInt16(-500,500), RangeInt16(-500,500),
		                                       RangeInt16(-500,500), RangeInt16(-500,500)
};
ValueInt16 MyRCReader::PRU_VALUES[] = {ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_ROLL]),ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_THRUST]),
         		                              ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_PITCH]),ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_YAW]),
		                                      ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_AUX1]),ValueInt16(0, MyRCReader::PRU_RANGES[CHAN_AUX2])
};
ValueInt16 MyRCReader::CHAN_VALUES[] = {ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_ROLL]),ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_THRUST]),
                                               ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_PITCH]),ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_YAW]),
                                               ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_AUX1]),ValueInt16(0, MyRCReader::CHAN_RANGES[CHAN_AUX2])
};


MyRCReader::MyRCReader() : initialized(false), active(false), agent(NULL) {

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
	   do {
	      int notimes = prussdrv_pru_wait_event (PRU_EVTOUT_1);
	      // save PRU values in micorseconds
	      MyRCReader::PRU_VALUES[0].setValue(int16_t((*(pru0DataMemory_int))/100));
	      MyRCReader::PRU_VALUES[1].setValue(int16_t((*(pru0DataMemory_int+1))/100));
	      MyRCReader::PRU_VALUES[2].setValue(int16_t((*(pru0DataMemory_int+2))/100));
	      MyRCReader::PRU_VALUES[3].setValue(int16_t((*(pru0DataMemory_int+3))/100));
	      MyRCReader::PRU_VALUES[4].setValue(int16_t((*(pru0DataMemory_int+4))/100));
	      MyRCReader::PRU_VALUES[5].setValue(int16_t((*(pru0DataMemory_int+5))/100));

	      // convert PRU values to CHAN values ([-500, 500]
	      MyRCReader::CHAN_VALUES[0].setValue(MyRCReader::PRU_VALUES[0]);
	      MyRCReader::CHAN_VALUES[1].setValue(MyRCReader::PRU_VALUES[1]);
	      MyRCReader::CHAN_VALUES[2].setValue(MyRCReader::PRU_VALUES[2]);
	      MyRCReader::CHAN_VALUES[3].setValue(MyRCReader::PRU_VALUES[3]);
	      MyRCReader::CHAN_VALUES[4].setValue(MyRCReader::PRU_VALUES[4]);
	      MyRCReader::CHAN_VALUES[5].setValue(MyRCReader::PRU_VALUES[5]);
	      prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU0_ARM_INTERRUPT);


	      // send CHAN values as int16 to agent
	      ((MyRCAgent*)agent)->setRCSample(MyRCReader::CHAN_VALUES[CHAN_THRUST].getValueAsPercent(),
	    		             MyRCReader::CHAN_VALUES[CHAN_ROLL].getValueAsPercent(),
							 MyRCReader::CHAN_VALUES[CHAN_PITCH].getValueAsPercent(),
							 MyRCReader::CHAN_VALUES[CHAN_YAW].getValueAsPercent(),
							 MyRCReader::CHAN_VALUES[CHAN_AUX1].getValueAsPercent(),
							 MyRCReader::CHAN_VALUES[CHAN_AUX2].getValueAsPercent());


	   } while (this->active);

	this->releasePRU();
}
bool MyRCReader::initPRU() {
	   if(getuid()!=0){
	      printf("You must run this program as root. Exiting.\n");
	      exit(EXIT_FAILURE);
	   }
	   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	   // Allocate and initialize memory
	   prussdrv_init ();
	   prussdrv_open (PRU_EVTOUT_0);
	   prussdrv_open (PRU_EVTOUT_1);

	   // Map PRU's INTC
	   prussdrv_pruintc_init(&pruss_intc_initdata);

	   // Copy data to PRU memory - different way
	   prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0DataMemory);

	   pru0DataMemory_int = (unsigned int *) pru0DataMemory;

	   // initialize data
	   int i = 0;
	   for(i = 0; i < 10;i++) {
	       *(pru0DataMemory_int + i) = 0;
	   }

	   // Load and execute binary on PRU
	   prussdrv_exec_program (PRU_NUM, "./provaPRU.bin");

	   return true;
}

void MyRCReader::releasePRU() {
	   /* Disable PRU and close memory mappings */
	   prussdrv_pru_disable(PRU_NUM);
	   prussdrv_exit ();

}

bool MyRCReader::initialize(void* agent) {
	if(!initialized) {
		this->agent = agent;
		this->active = true;
		if(this->initPRU()) {
			initialized = true;
		}
	}
	return initialized;
}
