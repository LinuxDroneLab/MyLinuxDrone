.origin 0               // offset of start of program in PRU memory
.entrypoint START       // program entry point used by the debugger

#define CHAN_PERIOD_US        5700
#define INS_PER_US            200
#define INS_PER_LOOP          2
#define MAX_CHAN_PERIOD_COUNT (CHAN_PERIOD_US * INS_PER_US) / INS_PER_LOOP

#define PRU0_R31_VEC_VALID  32;
#define PRU_EVTOUT_0        3
#define PRU_EVTOUT_1        4

// r0 = counter
// r1 = MAX_CHAN_PERIOD_COUNT
// r2 = CURR_CHANNEL
// r3 = STATUS_REG: t0=1 => send, t0=0 => discard
// r4 = current memory address

START:

   // set MAX_CHAN_PERIOD_COUNT to r1
   MOV    r1.w0, (MAX_CHAN_PERIOD_COUNT) & 0xFFFF
   MOV    r1.w2, (MAX_CHAN_PERIOD_COUNT) >> 16


INITCURRCHAN:
   // set initial memory location for results
   MOV    r4, 0x00000000  
   MOV    r2, 0                 // reset curr_channel
INITCOUNT:
   MOV    r0, 1                 // init counter
   // wait until PPM active
   WBS    r31.t5
COUNTHIGH:
   ADD    r0, r0, 1             // increment counter
   QBBS   COUNTHIGH, r31.t5     // loop until signal is high
SIGLOW:
   ADD    r2, r2, 1             // CURR_CHANNEL++
   QBLT   SAVECHANNEL, r1, r0   // save channel if it's not a sync signal
   QBEQ   SENDEV, r2, 9         // if I read all channels then send event else discard
   JMP INITCURRCHAN
SENDEV:
   MOV    R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_1  // generate interrupt for linux host
   JMP    INITCURRCHAN
SAVECHANNEL:
   // save data and increment address
   SBBO   r0, r4, 0, 4          // store the counter at r4 address
   ADD r4, r4, 4                // next address
   JMP INITCOUNT

