#ifndef SAMD51_H
#define SAMD51_H
#if defined(CORE_SAMD51)

#include "sam.h"

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint32_t //Size of the port variables (Eg inj1_pin_port). Most systems use a byte, but SAMD21 and possibly others are a 32-bit unsigned int
  #define PINMASK_TYPE uint32_t
  #define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
  #define FPU_MAX_SIZE 0 //Size of the FPU buffer. 0 means no FPU.
  #define BOARD_MAX_IO_PINS  52 //digital pins + analog channels + 1
  #define BOARD_MAX_DIGITAL_PINS 52 //Pretty sure this isn't right
  #define EEPROM_LIB_H <EEPROM.h> //The name of the file that provides the EEPROM class
  typedef int eeprom_address_t;
  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();

  #define pinIsReserved(pin)  ( ((pin) == 0) ) //Forbidden pins like USB

/*
***********************************************************************************************************
* Schedules
* SAMD51 TCs and TCCs, oh what fun.
* Will initially try Josh's setup for registers, i.e. TCC0->COUNT.reg / TCC0->CC[0].bit.CC
* Not sure if they'll work or not...
* Also not sure this can't be done with the simpler TC timers.  Will try later.
* Fuelling first:
*/
  
  #define FUEL1_COUNTER TCC0->COUNT.reg
  #define FUEL2_COUNTER TCC0->COUNT.reg
  #define FUEL3_COUNTER TCC0->COUNT.reg
  #define FUEL4_COUNTER TCC0->COUNT.reg
  #define FUEL5_COUNTER TCC0->COUNT.reg
  #define FUEL6_COUNTER TCC0->COUNT.reg
  #define FUEL7_COUNTER TCC4->COUNT.reg
  #define FUEL8_COUNTER TCC4->COUNT.reg
  
  #define FUEL1_COMPARE TCC0->CC[0].bit.CC
  #define FUEL2_COMPARE TCC0->CC[1].bit.CC
  #define FUEL3_COMPARE TCC0->CC[2].bit.CC
  #define FUEL4_COMPARE TCC0->CC[3].bit.CC
  #define FUEL5_COMPARE TCC0->CC[4].bit.CC
  #define FUEL6_COMPARE TCC0->CC[5].bit.CC
  #define FUEL7_COMPARE TCC4->CC[0].bit.CC
  #define FUEL8_COMPARE TCC4->CC[1].bit.CC

/*
* Now Ignitions...
*/

  #define IGN1_COUNTER  TCC1->COUNT.reg
  #define IGN2_COUNTER  TCC1->COUNT.reg
  #define IGN3_COUNTER  TCC1->COUNT.reg
  #define IGN4_COUNTER  TCC1->COUNT.reg
  #define IGN5_COUNTER  TCC2->COUNT.reg
  #define IGN6_COUNTER  TCC2->COUNT.reg
  #define IGN7_COUNTER  TCC3->COUNT.reg
  #define IGN8_COUNTER  TCC3->COUNT.reg

  #define IGN1_COMPARE  TCC1->CC[0].bit.CC
  #define IGN2_COMPARE  TCC1->CC[1].bit.CC
  #define IGN3_COMPARE  TCC1->CC[2].bit.CC
  #define IGN4_COMPARE  TCC1->CC[3].bit.CC
  #define IGN5_COMPARE  TCC2->CC[0].bit.CC
  #define IGN6_COMPARE  TCC2->CC[1].bit.CC
  #define IGN7_COMPARE  TCC3->CC[0].bit.CC
  #define IGN8_COMPARE  TCC3->CC[1].bit.CC

  static inline void FUEL1_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL2_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL3_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL4_TIMER_ENABLE(void)  {<macro here>;}
  //The below are optional, but recommended if there are sufficient timers/compares
  static inline void FUEL5_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL6_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL7_TIMER_ENABLE(void)  {<macro here>;}
  static inline void FUEL8_TIMER_ENABLE(void)  {<macro here>;}

  static inline void FUEL1_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL2_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL3_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL4_TIMER_DISABLE(void)  { <macro here>;}
  //The below are optional, but recommended if there are sufficient timers/compares
  static inline void FUEL5_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL6_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL7_TIMER_DISABLE(void)  { <macro here>;}
  static inline void FUEL8_TIMER_DISABLE(void)  { <macro here>;}

    static inline void IGN1_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN2_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN3_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN4_TIMER_ENABLE(void)  {<macro here>;}
  //The below are optional, but recommended if there are sufficient timers/compares
    static inline void IGN5_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN6_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN7_TIMER_ENABLE(void)  {<macro here>;}
    static inline void IGN8_TIMER_ENABLE(void)  {<macro here>;}

    static inline void IGN1_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN2_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN3_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN4_TIMER_DISABLE(void)  {<macro here>;}
  //The below are optional, but recommended if there are suffici;}ent timers/compares
    static inline void IGN5_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN6_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN7_TIMER_DISABLE(void)  {<macro here>;}
    static inline void IGN8_TIMER_DISABLE(void)  {<macro here>;}

  
  #define MAX_TIMER_PERIOD 139808 //This is the maximum time, in uS, that the compare channels can run before overflowing. It is typically 65535 * <how long each tick represents>
  #define uS_TO_TIMER_COMPARE(uS) ((uS * 15) >> 5) //Converts a given number of uS into the required number of timer ticks until that time has passed.

/*
***********************************************************************************************************
* Auxiliaries
*/
  //macro functions for enabling and disabling timer interrupts for the boost and vvt functions
  #define ENABLE_BOOST_TIMER()  <macro here>
  #define DISABLE_BOOST_TIMER(void)  { <macro here>

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


#endif //CORE_TEMPLATE
#endif //TEMPLATE_H
