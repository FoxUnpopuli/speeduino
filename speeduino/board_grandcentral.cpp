#include "globals.h"
#if defined(CORE_SAMD51)


void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */

    /*
    ***********************************************************************************************************
    * Timers
    */

    /*
    ***********************************************************************************************************
    * Auxiliaries
    */

    /*
    ***********************************************************************************************************
    * Idle
    */

    /*
    ***********************************************************************************************************
    * Schedules
    */
}


extern "C" char *sbrk(int i);

uint16_t freeRam() {
  uint32_t tmp_freeram;
  char stack_dummy = 0;  // stick a dummy on the stack...
  tmp_freeram = &stack_dummy - sbrk(0);  // address of dummy - address of heap = guess
  if(tmp_freeram>0xFFFF){return 0xFFFF;} // ... if guess bigger than 16bits return all the ones,
    else{return tmp_freeram;}            // else report the guess.  :)
}

void doSystemReset() { return; }
void jumpToBootloader() { return; }

#endif
