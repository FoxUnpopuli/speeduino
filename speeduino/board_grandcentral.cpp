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
    * Schedules
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
    // Setup clocks for ALL THE TIMER/COMPARES!
    MCLK->APBAMASK.reg = MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1 ;
    MCLK->APBBMASK.reg = MCLK_APBBMASK_TC2 | MCLK_APBBMASK_TC3 ;
    MCLK->APBCMASK.reg = MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 ;
    MCLK->APBDMASK.reg = MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 ;
              
    // Set up the generic clock (GCLK7) bang on 500kHz, 2us tick - for shiggles
    GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(96) |       // Divide the 48MHz clock source by divisor 96: 48MHz/96 = 500KHz
                           GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                           GCLK_GENCTRL_GENEN |         // Enable GCLK7
                           GCLK_GENCTRL_SRC_DFLL;       // Generate from 48MHz DFLL clock source
    while (GCLK->SYNCBUSY.bit.GENCTRL7);                // Wait for synchronization

    // Enable all TC perhipheral channels with appropriate GCLK_PCHCTRL_CHEN and...
    GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC0/1 (9)
    GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC2/3 (26)
    GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC4/5 (30)
    GCLK->PCHCTRL[TC6_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC6/7 (39)

    // Set prescaler to 1, set the reset/reload to trigger on the GCLK, set the counter to 16-bit mode...
    // ... for ALL THE... etc.
    TC0->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC1->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC2->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC6->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;
    TC7->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_MODE_COUNT16;

    // Disable all TC interrupts, then enable the MC0 and MC1 interrupts... and the overflow as well just in case
    TC0->COUNT16.INTENSET.reg = 0; TC0->COUNT16.INTENSET.bit.MC0 = 1; TC0->COUNT16.INTENSET.bit.MC1 = 1; TC0->COUNT16.INTENSET.bit.OVF = 1;
    TC1->COUNT16.INTENSET.reg = 0; TC1->COUNT16.INTENSET.bit.MC0 = 1; TC1->COUNT16.INTENSET.bit.MC1 = 1; TC1->COUNT16.INTENSET.bit.OVF = 1;
    TC2->COUNT16.INTENSET.reg = 0; TC2->COUNT16.INTENSET.bit.MC0 = 1; TC2->COUNT16.INTENSET.bit.MC1 = 1; TC2->COUNT16.INTENSET.bit.OVF = 1;
    TC3->COUNT16.INTENSET.reg = 0; TC3->COUNT16.INTENSET.bit.MC0 = 1; TC3->COUNT16.INTENSET.bit.MC1 = 1; TC3->COUNT16.INTENSET.bit.OVF = 1;
    TC4->COUNT16.INTENSET.reg = 0; TC4->COUNT16.INTENSET.bit.MC0 = 1; TC4->COUNT16.INTENSET.bit.MC1 = 1; TC4->COUNT16.INTENSET.bit.OVF = 1;
    TC5->COUNT16.INTENSET.reg = 0; TC5->COUNT16.INTENSET.bit.MC0 = 1; TC5->COUNT16.INTENSET.bit.MC1 = 1; TC5->COUNT16.INTENSET.bit.OVF = 1;
    TC6->COUNT16.INTENSET.reg = 0; TC6->COUNT16.INTENSET.bit.MC0 = 1; TC6->COUNT16.INTENSET.bit.MC1 = 1; TC6->COUNT16.INTENSET.bit.OVF = 1;
    TC7->COUNT16.INTENSET.reg = 0; TC7->COUNT16.INTENSET.bit.MC0 = 1; TC7->COUNT16.INTENSET.bit.MC1 = 1; TC7->COUNT16.INTENSET.bit.OVF = 1;

    // Set up all TCs in the Nested Vector Interrupt Controller...
    // ...all ahead full, warp drive at your command.
    NVIC_EnableIRQ(TC0_IRQn);
    NVIC_EnableIRQ(TC1_IRQn);
    NVIC_EnableIRQ(TC2_IRQn);
    NVIC_EnableIRQ(TC3_IRQn);
    NVIC_EnableIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC5_IRQn);
    NVIC_EnableIRQ(TC6_IRQn);
    NVIC_EnableIRQ(TC7_IRQn);
  

}


// Fun little freeRam routine.
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

// All ISRs 




#endif
