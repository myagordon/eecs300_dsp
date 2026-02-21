/*
  ******************************************************************************
  * @file    HAL.cpp
  * @author  EECS 300 STAFF
  * @brief   HAL driver for EECS300 peripherals. For more information on espressif functions called, 
  *          refer to https://docs.espressif.com/projects/esp-idf/en/v3.2.5/api-reference/peripherals/index.html
  ****************************************************************************** 
*/

#include "HAL.h"


void blockInterrupts()
{
    xt_ints_off(~(0b1 << 6));
}

void unblockInterrupts()
{
    xt_ints_on(0xFFFFFFFF);
}

void attachPwm(uint8_t pin, uint32_t freq, uint8_t chan){ // new function
    if(freq < 1 || 312500 < freq || chan > 15 || pin > 39)  return;
    ledcAttachChannel(pin, freq, DUTY_RESOLUTION, chan);
}

void setPwmDuty(uint8_t chan, uint16_t duty)
{
    if(chan > 15)  return;
    ledcWriteChannel(chan, duty);
}

void pwmDetach(uint8_t pin)
{
    if(pin > 39)  return;
    ledcDetach(pin);
}

hw_timer_t* setUpTimer(){
    return timerBegin(TIMER_DEFAULT); 
}

void startTimer(hw_timer_t *timer, void (*f)(), uint64_t period) { // new function
    timerAttachInterrupt(timer, f);
    timerAlarm(timer, period, true, 0); // repeat the alarm (third param) until period (second param) and reset back to 0 (fourth param)
}

void stopTimer(hw_timer_t *timer) {
    timerEnd(timer);
}

void writeToDAC(uint8_t pin, uint8_t value) {
    if (pin<25 || pin>26) return;
    dacWrite(pin, value);
}

void stopDAC(uint8_t pin) {
    dacDisable(pin);
}