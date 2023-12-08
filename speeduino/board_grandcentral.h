#ifndef SAMD51_H
#define SAMD51_H

// FoxUnpop: Not sure about ^^those...

#if defined(CORE_SAMD51)

#include "sam.h"

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint32_t //Size of the port variables (Eg inj1_pin_port). Most systems use a byte, but SAMD21 and possibly others are a 32-bit unsigned int
  #define PINMASK_TYPE uint32_t
  #define COMPARE_TYPE uint16_t

  #define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
  #define FPU_MAX_SIZE 32 //Size of the FPU buffer. 0 means no FPU.  (FoxUnpop: this means word bitlength the FPU works in, right?)

  // FoxUnpop: SD card on SPI1.
  #define SD_LOGGING //SD logging enabled by default for Grand Central as it has the slot built in
  #define SD_CONFIG SDCARD_SS_PIN // on SAMD51, SD.begin(<chip select pin>) is all you need...?

  /* Needed for SD library
  #define SDCARD_SPI          SPI1
  #define SDCARD_MISO_PIN     PIN_SPI1_MISO
  #define SDCARD_MOSI_PIN     PIN_SPI1_MOSI
  #define SDCARD_SCK_PIN      PIN_SPI1_SCK
  #define SDCARD_SS_PIN       PIN_SPI1_SS
  */

  #define BOARD_MAX_IO_PINS  70     //Same as a MEGA...
  #define BOARD_MAX_DIGITAL_PINS 54 //Also same.  ADC pins in globals.h

  // FoxUnpop: Might need help with this, Grand Central has lovely QSPI flash onboard and a library to use it...  not sure if below is compatible?
  #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
  typedef uint16_t eeprom_address_t;
  
  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();

  #define pinIsReserved(pin)  ( ((pin) == 0) ) //Forbidden pins like USB
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 13
  #endif
/*
***********************************************************************************************************
* Schedules
* SAMD51/E54 TCs and TCCs, oh what fun.
* Initially tried Josh's setup for registers, i.e. TCC0->COUNT.reg / TCC0->CC[0].bit.CC
* Didn't finish - don't need the complexity, maybe better keep it simple?  8 TCs with 2 channels each...
* Probably need to unset all TCs and TCCs from any pins that happen to be attached in the variant file.
*
* Fuel first:
*/
  
  #define FUEL1_COUNTER TC0->COUNT16.COUNT.reg
  #define FUEL2_COUNTER TC0->COUNT16.COUNT.reg
  #define FUEL3_COUNTER TC1->COUNT16.COUNT.reg
  #define FUEL4_COUNTER TC1->COUNT16.COUNT.reg
  #define FUEL5_COUNTER TC4->COUNT16.COUNT.reg
  #define FUEL6_COUNTER TC4->COUNT16.COUNT.reg
  #define FUEL7_COUNTER TC5->COUNT16.COUNT.reg
  #define FUEL8_COUNTER TC5->COUNT16.COUNT.reg
  
  #define FUEL1_COMPARE TC0->COUNT16.CC[0].reg  // CC0 register, when matched, sets the MC0 event...
  #define FUEL2_COMPARE TC0->COUNT16.CC[1].reg  // CC1 register, when matched, sets the MC1 event.  Simples!(?)
  #define FUEL3_COMPARE TC1->COUNT16.CC[0].reg
  #define FUEL4_COMPARE TC1->COUNT16.CC[1].reg
  #define FUEL5_COMPARE TC4->COUNT16.CC[0].reg
  #define FUEL6_COMPARE TC4->COUNT16.CC[1].reg
  #define FUEL7_COMPARE TC5->COUNT16.CC[0].reg
  #define FUEL8_COMPARE TC5->COUNT16.CC[1].reg

  // Now Ignitions...

  #define IGN1_COUNTER  TC2->COUNT16.COUNT.reg
  #define IGN2_COUNTER  TC2->COUNT16.COUNT.reg
  #define IGN3_COUNTER  TC3->COUNT16.COUNT.reg
  #define IGN4_COUNTER  TC3->COUNT16.COUNT.reg
  #define IGN5_COUNTER  TC6->COUNT16.COUNT.reg
  #define IGN6_COUNTER  TC6->COUNT16.COUNT.reg
  #define IGN7_COUNTER  TC7->COUNT16.COUNT.reg
  #define IGN8_COUNTER  TC7->COUNT16.COUNT.reg

  #define IGN1_COMPARE  TC2->COUNT16.CC[0].reg
  #define IGN2_COMPARE  TC2->COUNT16.CC[1].reg
  #define IGN3_COMPARE  TC3->COUNT16.CC[0].reg
  #define IGN4_COMPARE  TC3->COUNT16.CC[1].reg
  #define IGN5_COMPARE  TC6->COUNT16.CC[0].reg
  #define IGN6_COMPARE  TC6->COUNT16.CC[1].reg
  #define IGN7_COMPARE  TC7->COUNT16.CC[0].reg
  #define IGN8_COMPARE  TC7->COUNT16.CC[1].reg


  // Fuel enables first... enable the Match Compare SET interrupts...
  static inline void FUEL1_TIMER_ENABLE(void)  { TC0->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC0->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL2_TIMER_ENABLE(void)  { TC0->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC0->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL3_TIMER_ENABLE(void)  { TC1->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC1->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL4_TIMER_ENABLE(void)  { TC1->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC1->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL5_TIMER_ENABLE(void)  { TC4->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL6_TIMER_ENABLE(void)  { TC4->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); }  ///  All these are wrong - I want to enable the MCx, not the whole timer...
  static inline void FUEL7_TIMER_ENABLE(void)  { TC5->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC5->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL8_TIMER_ENABLE(void)  { TC5->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC5->COUNT16.SYNCBUSY.bit.ENABLE); }

  // Fuel disables next... enable the Match Compare CLEAR interrupts... (*sigh*)
  static inline void FUEL1_TIMER_DISABLE(void)  { TC0->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC0; while (TC0->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL2_TIMER_DISABLE(void)  { TC0->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC1; while (TC0->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL3_TIMER_DISABLE(void)  { TC1->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC0; while (TC1->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL4_TIMER_DISABLE(void)  { TC1->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC1; while (TC1->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL5_TIMER_DISABLE(void)  { TC4->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC0; while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL6_TIMER_DISABLE(void)  { TC4->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC1; while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL7_TIMER_DISABLE(void)  { TC5->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC0; while (TC5->COUNT16.SYNCBUSY.bit.ENABLE); }
  static inline void FUEL8_TIMER_DISABLE(void)  { TC5->COUNT16.INTENCLR.reg |= TC_INTENCLR_MC1; while (TC5->COUNT16.SYNCBUSY.bit.ENABLE); }

    // Ignition compare enables...
    static inline void IGN1_TIMER_ENABLE(void)  { TC2->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC2->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN2_TIMER_ENABLE(void)  { TC2->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC2->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN3_TIMER_ENABLE(void)  { TC3->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC3->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN4_TIMER_ENABLE(void)  { TC3->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC3->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN5_TIMER_ENABLE(void)  { TC6->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC6->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN6_TIMER_ENABLE(void)  { TC6->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC6->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN7_TIMER_ENABLE(void)  { TC7->COUNT16.INTENSET.reg |= TC_INTENSET_MC0; while (TC7->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN8_TIMER_ENABLE(void)  { TC7->COUNT16.INTENSET.reg |= TC_INTENSET_MC1; while (TC7->COUNT16.SYNCBUSY.bit.ENABLE); }

    // and finally, ignition compare disables.  :)
    static inline void IGN1_TIMER_DISABLE(void)  { TC2->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC2->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN2_TIMER_DISABLE(void)  { TC2->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC2->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN3_TIMER_DISABLE(void)  { TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC3->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN4_TIMER_DISABLE(void)  { TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC3->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN5_TIMER_DISABLE(void)  { TC6->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC6->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN6_TIMER_DISABLE(void)  { TC6->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC6->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN7_TIMER_DISABLE(void)  { TC7->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC7->COUNT16.SYNCBUSY.bit.ENABLE); }
    static inline void IGN8_TIMER_DISABLE(void)  { TC7->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; while (TC7->COUNT16.SYNCBUSY.bit.ENABLE); }

    // Yet another tick-period... 2us (to begin.)
  #define MAX_TIMER_PERIOD 131070 //The longest period of time (in uS) that the timer can permit (In this case it is 65535 * 2, as each timer tick is 2uS)  (Why not 65536 * 2?)
  #define uS_TO_TIMER_COMPARE(uS) ((uS) >> 1) //Converts a given number of uS into the required number of timer ticks until that time has passed
/*
***********************************************************************************************************
* Auxiliaries
*/
  //macro functions for enabling and disabling timer interrupts for the boost and vvt functions
  #define ENABLE_BOOST_TIMER()  <macro here>
  #define DISABLE_BOOST_TIMER(void)  <macro here> 

  #define ENABLE_VVT_TIMER()    <macro here>
  #define DISABLE_VVT_TIMER()   <macro here>

  #define BOOST_TIMER_COMPARE   <register here>
  #define BOOST_TIMER_COUNTER   <register here>
  #define VVT_TIMER_COMPARE     <register here>
  #define VVT_TIMER_COUNTER     <register here>

/*
***********************************************************************************************************
* Idle
*/
  //Same as above, but for the timer controlling PWM idle
  #define IDLE_COUNTER          <register here>
  #define IDLE_COMPARE          <register here>

  #define IDLE_TIMER_ENABLE()   <macro here>
  #define IDLE_TIMER_DISABLE()  <macro here>

/*
***********************************************************************************************************
* CAN / Second serial
*/


#endif //CORE_SAMD51
#endif //BOARD_GRANDCENTRAL_H
