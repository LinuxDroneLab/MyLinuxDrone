.origin 0               // offset of start of program in PRU memory
.entrypoint START       // program entry point used by the debugger

#define MIN_CHAN_PERIOD_US    500
#define MAX_CHAN_PERIOD_US    10100
#define INS_PER_US            200
#define INS_PER_LOOP          2
#define MAX_CHAN_PERIOD_COUNT (MAX_CHAN_PERIOD_US * INS_PER_US) / INS_PER_LOOP
#define MIN_CHAN_PERIOD_COUNT (MIN_CHAN_PERIOD_US * INS_PER_US) / INS_PER_LOOP

#define PRU0_R31_VEC_VALID  32;
#define PRU_EVTOUT_0        3
#define PRU_EVTOUT_1        4

// r0 = counter
// r1 = MAX_CHAN_PERIOD_COUNT
// r2 = CURR_CHANNEL
// r4 = current memory address

START:
   // set MAX_CHAN_PERIOD_COUNT to r1
   MOV    r1.w0, (MAX_CHAN_PERIOD_COUNT) & 0xFFFF
   MOV    r1.w2, (MAX_CHAN_PERIOD_COUNT) >> 16
   MOV    r5.w0, (MIN_CHAN_PERIOD_COUNT) & 0xFFFF
   MOV    r5.w2, (MIN_CHAN_PERIOD_COUNT) >> 16
INIT_SYNC:
   // set initial memory location for results
   MOV    r4, 0x00000000  
   WBC    r31.t5
INITCOUNT_SYNC:
   MOV    r2, 0                 // reset curr_channel
   MOV    r0, 1                 // init counter
   // wait until PPM active
   WBS    r31.t5
COUNTHIGH_SYNC:
   ADD    r0, r0, 1                  // increment counter
   QBBS   COUNTHIGH_SYNC, r31.t5     // loop until signal is high
SIGNLOW_SYNC:
   QBLT   INITCOUNT_SYNC, r1, r0          // restart counting if not a sync signal

WAITHIGH:
   ADD    r2, r2, 1             // current channel [1-8]
   QBLT   SENDEV, r2, 8         // send event if got all channels
   MOV    r0, 1                 // init counter
   WBS    r31.t5                // wait until PPM active
COUNTHIGH:
   ADD    r0, r0, 1             // increment counter
   QBBS   COUNTHIGH, r31.t5     // loop until signal is high
   QBLT   INIT_SYNC, r5, r0     // reset cycle if counter < min period
SAVECHANNEL:
   // save data and increment address
   SBBO   r0, r4, 0, 4          // store the counter at r4 address
   ADD r4, r4, 4                // next address
   JMP WAITHIGH
SENDEV:
   MOV    R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_1  // generate interrupt for linux host
   JMP    INIT_SYNC

