/* PRUSS program to drive a HC-SR04 sensor and display the sensor output
*  in Linux userspace by sending an interrupt.
*  written by Derek Molloy for the book Exploring BeagleBone
*/
#include <stdio.h>
#include <stdlib.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <pthread.h>
#include <unistd.h>

#define PRU_NUM 0

static void *pru0DataMemory;
static unsigned int *pru0DataMemory_int;

void *threadFunction(void *value){
   do {
      int notimes = prussdrv_pru_wait_event (PRU_EVTOUT_1);

      unsigned int chan1 = *(pru0DataMemory_int);
      unsigned int chan2 = *(pru0DataMemory_int + 1);
      unsigned int chan3 = *(pru0DataMemory_int + 2);
      unsigned int chan4 = *(pru0DataMemory_int + 3);
      unsigned int chan5 = *(pru0DataMemory_int + 4);
      unsigned int chan6 = *(pru0DataMemory_int + 5);
      printf("1=%d, 2=%d, 3=%d, 4=%d, 5=%d, 6=%d \n", chan1, chan2, chan3, chan4, chan5, chan6);
      prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU0_ARM_INTERRUPT);
   } while (1);
}

int  main (void)
{
   printf("STARTED \n");
   if(getuid()!=0){
      printf("You must run this program as root. Exiting.\n");
      exit(EXIT_FAILURE);
   }
   pthread_t thread;
   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

   // Allocate and initialize memory
   prussdrv_init ();
   printf("INITIALIZED \n");
   prussdrv_open (PRU_EVTOUT_0);
   prussdrv_open (PRU_EVTOUT_1);
   printf("EVENT CHANNELS OPENED \n");

   // Map PRU's INTC
   prussdrv_pruintc_init(&pruss_intc_initdata);
   printf("INTC MAPPED \n");

   // Copy data to PRU memory - different way
   prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0DataMemory);
   printf("MEMORY MAPPED \n");

   pru0DataMemory_int = (unsigned int *) pru0DataMemory;

   // initialize data
   int i = 0;
   for(i = 0; i < 10;i++) {
       *(pru0DataMemory_int + i) = 0;
   }
   printf("MEMORY INITIALIZED \n");

   // Load and execute binary on PRU
   prussdrv_exec_program (PRU_NUM, "./provaPRU.bin");
   if(pthread_create(&thread, NULL, &threadFunction, NULL)){
       printf("Failed to create thread!");
   }
   prussdrv_pru_enable(PRU_NUM);
   int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
   printf("PRU program completed, event number %d.\n", n);

   /* Disable PRU and close memory mappings */
   prussdrv_pru_disable(PRU_NUM);
   prussdrv_exit ();
   return EXIT_SUCCESS;
}
