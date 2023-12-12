#include "globals.h"
#if defined(CORE_SAMD51)
#include "board_grandcentral.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"
#include "timers.h"
#include "comms_secondary.h"

// Might be useful for looping in setup...
Tc* TCx[8] = {TC0, TC1, TC2, TC3, TC4, TC5, TC6, TC7}; // Initialization
Tcc* TCCx[5] = {TCC0, TCC1, TCC2, TCC3, TCC4}; // Initialization; 0 has 6, 1 has 4, 2 has 3, 3 & 4 have 2 compare channels.

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    * I think this setup is correct, Serial1 is configured by default for the Adafruit GC.
    */
        pSecondarySerial = &Serial1;

    // Setup clocks for ALL THE TIMER/COMPARES!
    MCLK->APBAMASK.reg = MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1 ;
    MCLK->APBBMASK.reg = MCLK_APBBMASK_TC2 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 ;
    MCLK->APBCMASK.reg = MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 ;
    MCLK->APBDMASK.reg = MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 ;
 
    // FoxUnpop: TODO Set up a secondary serial port?
    // Serial1 is available on 0/1 and is set up in the Variant.cpp file...  do we need a Serial2 as well?
    /*
        Uart Serial2( SERCOM_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

        void SERCOM4_0_Handler()
        {
        Serial2.IrqHandler();
        }
        void SERCOM4_1_Handler()
        {
        Serial2.IrqHandler();
        }
        void SERCOM4_2_Handler()
        {
        Serial2.IrqHandler();
        }
        void SERCOM4_3_Handler()
        {
        Serial2.IrqHandler();
        }

    */
    
    /*
    ***********************************************************************************************************
    * Schedules
    */
              
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
    // Don't think this is MISRA-compliant...
    TC0->COUNT16.INTENSET.reg = 0; TC0->COUNT16.INTENSET.bit.MC0 = 1; TC0->COUNT16.INTENSET.bit.MC1 = 1; TC0->COUNT16.INTENSET.bit.OVF = 1;
    TC1->COUNT16.INTENSET.reg = 0; TC1->COUNT16.INTENSET.bit.MC0 = 1; TC1->COUNT16.INTENSET.bit.MC1 = 1; TC1->COUNT16.INTENSET.bit.OVF = 1;
    TC2->COUNT16.INTENSET.reg = 0; TC2->COUNT16.INTENSET.bit.MC0 = 1; TC2->COUNT16.INTENSET.bit.MC1 = 1; TC2->COUNT16.INTENSET.bit.OVF = 1;
    TC3->COUNT16.INTENSET.reg = 0; TC3->COUNT16.INTENSET.bit.MC0 = 1; TC3->COUNT16.INTENSET.bit.MC1 = 1; TC3->COUNT16.INTENSET.bit.OVF = 1;
    TC4->COUNT16.INTENSET.reg = 0; TC4->COUNT16.INTENSET.bit.MC0 = 1; TC4->COUNT16.INTENSET.bit.MC1 = 1; TC4->COUNT16.INTENSET.bit.OVF = 1;
    TC5->COUNT16.INTENSET.reg = 0; TC5->COUNT16.INTENSET.bit.MC0 = 1; TC5->COUNT16.INTENSET.bit.MC1 = 1; TC5->COUNT16.INTENSET.bit.OVF = 1;
    TC6->COUNT16.INTENSET.reg = 0; TC6->COUNT16.INTENSET.bit.MC0 = 1; TC6->COUNT16.INTENSET.bit.MC1 = 1; TC6->COUNT16.INTENSET.bit.OVF = 1;
    TC7->COUNT16.INTENSET.reg = 0; TC7->COUNT16.INTENSET.bit.MC0 = 1; TC7->COUNT16.INTENSET.bit.MC1 = 1; TC7->COUNT16.INTENSET.bit.OVF = 1;

    // Set up all TCs in the Nested Vector Interrupt Controller...
    NVIC_EnableIRQ(TC0_IRQn);
    NVIC_EnableIRQ(TC1_IRQn);
    NVIC_EnableIRQ(TC2_IRQn);
    NVIC_EnableIRQ(TC3_IRQn);
    NVIC_EnableIRQ(TC4_IRQn);
    NVIC_EnableIRQ(TC5_IRQn);
    NVIC_EnableIRQ(TC6_IRQn);
    NVIC_EnableIRQ(TC7_IRQn);

    // fill all the compare registers for fun, theoretically counters will just overflow at 0xFFFF, so no MAX/TOP need setting.
    // Now I setup  this loop method, I could have done all the setups above like this... TODO
    for (int i = 0; i < 8; i++) {
        TCx[i]->COUNT16.CC[0].reg = 65535;          // just load the compare at the top value to begin
        while (TCx[i]->COUNT16.SYNCBUSY.bit.CC0);   // synch
        TCx[i]->COUNT16.CC[1].reg = 65535;          
        while (TCx[i]->COUNT16.SYNCBUSY.bit.CC1);
        TCx[i]->COUNT16.COUNT.reg = 0;              // zero the count...
        }
        // ...all ahead full, warp drive at your command.

}

/*
    ***********************************************************************************************************
    * Auxiliaries
    * Setup big boy TCC timercounters in the simplest way:
    * Probably need to disassociate them from the PORTs to ensure GC board pin outputs behave as expected: TODO
    */

    GCLK->GENCTRL[1].reg =  GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                            GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                            GCLK_GENCTRL_GENEN |        // Enable GCLK1
                            GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
    while (GCLK->SYNCBUSY.bit.GENCTRL1);                // Wait for synchronization

    GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral channel
                            GCLK_PCHCTRL_GEN_GCLK1;     // Connect generic clock 1 to TCC0 and TCC1 at 16MHz




    /*
    ***********************************************************************************************************
    * Idle
    * Setup big boy TCC timercounter TCC0 as the PWM-output idle control.  Is it always PWM?
    * Probably need to disassociate them from the PORTs to ensure GC board pin outputs behave as expected: TODO
    */



// All ISRs 

void TC0_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC0->COUNT16.INTFLAG.bit.MC0) { TC0->COUNT16.INTFLAG.bit.MC0 = 1; fuelSchedule1Interrupt(); }
    if (TC0->COUNT16.INTFLAG.bit.MC1) { TC0->COUNT16.INTFLAG.bit.MC1 = 1; fuelSchedule2Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC1_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC1->COUNT16.INTFLAG.bit.MC0) { TC1->COUNT16.INTFLAG.bit.MC0 = 1; fuelSchedule3Interrupt(); }
    if (TC1->COUNT16.INTFLAG.bit.MC1) { TC1->COUNT16.INTFLAG.bit.MC1 = 1; fuelSchedule4Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC2_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC2->COUNT16.INTFLAG.bit.MC0) { TC2->COUNT16.INTFLAG.bit.MC0 = 1; ignitionSchedule1Interrupt(); }
    if (TC2->COUNT16.INTFLAG.bit.MC1) { TC2->COUNT16.INTFLAG.bit.MC1 = 1; ignitionSchedule2Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC3_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC3->COUNT16.INTFLAG.bit.MC0) { TC3->COUNT16.INTFLAG.bit.MC0 = 1; ignitionSchedule3Interrupt(); }
    if (TC3->COUNT16.INTFLAG.bit.MC1) { TC3->COUNT16.INTFLAG.bit.MC1 = 1; ignitionSchedule4Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC4_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC4->COUNT16.INTFLAG.bit.MC0) { TC4->COUNT16.INTFLAG.bit.MC0 = 1; fuelSchedule5Interrupt(); }
    if (TC4->COUNT16.INTFLAG.bit.MC1) { TC4->COUNT16.INTFLAG.bit.MC1 = 1; fuelSchedule6Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC5_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC5->COUNT16.INTFLAG.bit.MC0) { TC5->COUNT16.INTFLAG.bit.MC0 = 1; fuelSchedule7Interrupt(); }
    if (TC5->COUNT16.INTFLAG.bit.MC1) { TC5->COUNT16.INTFLAG.bit.MC1 = 1; fuelSchedule8Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC6_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC6->COUNT16.INTFLAG.bit.MC0) { TC6->COUNT16.INTFLAG.bit.MC0 = 1; ignitionSchedule5Interrupt(); }
    if (TC6->COUNT16.INTFLAG.bit.MC1) { TC6->COUNT16.INTFLAG.bit.MC1 = 1; ignitionSchedule6Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

void TC7_Handler() {
    // Check if MC0, if yes, clear it and git'er'done; or do same for MC1
    if (TC7->COUNT16.INTFLAG.bit.MC0) { TC7->COUNT16.INTFLAG.bit.MC0 = 1; ignitionSchedule7Interrupt(); }
    if (TC7->COUNT16.INTFLAG.bit.MC1) { TC7->COUNT16.INTFLAG.bit.MC1 = 1; ignitionSchedule8Interrupt(); }
    // TODO: probably should be some 'oh no it's all gone wrong' code here.
}

// Idle ISR handler
void TCC0_1_Handler() {
    // TCC0_1_Handler only fires on TCC0 MC0... 
    TCC0->INTFLAG.bit.MC0 = 1; // no need to check, just reset it.
    idleInterrupt();

 }

 // Boost ISR Handler
void TCC1_1_Handler() { 
    // TCC1_1_Handler only fires on TCC1 MC0...
    TCC1->INTFLAG.bit.MC0 = 1; // no need to check, just reset it.
    boostInterrupt();

 }

 // VVT ISR Handler
 void TCC1_2_Handler() { 
    // TCC1_2_Handler only fires on TCC1 MC1...
    TCC1->INTFLAG.bit.MC1 = 1; // no need to check, just reset it.
    vvtInterrupt();

 }

 // Fan ISR Handler
 void TCC1_3_Handler() { 
    // TCC1_3_Handler only fires on TCC1 MC2...
    TCC1->INTFLAG.bit.MC2 = 1; // no need to check, just reset it.
    FanInterrupt();

 }
 
//etc...

// Fun little free RAM routine.
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
