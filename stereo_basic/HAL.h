/*
  ******************************************************************************
  * @file    HAL.h
  * @author  EECS 300 STAFF
  * @brief   HAL driver function prototypes for EECS300 peripherals. 
  ****************************************************************************** 
*/

#ifndef HAL_H
#define HAL_H

#include <esp32-hal-ledc.h>
#include <esp32-hal-timer.h>
#include <esp32-hal-dac.h>
#include <Wire.h>


#define PUSH_BUTTON_PIN 0
#define LED_PIN 2
#define DUTY_RESOLUTION 8
#define TIMER_DEFAULT 1000000

/*
 * Function:  blockInterrupts()
 * --------------------
 * temporarily blocks all interrupts from firing
 * interrupts can be unblocked with unblockInterrupts()
 *
 *  returns: void
 */
void blockInterrupts();

/*
 * Function:  unblockInterrupts()
 * --------------------
 * unblocks enabled interrupts from firing
 *
 * returns: void
 */
void unblockInterrupts();

/*
 * Function:  attachPwm
 * --------------------
 * configures a pin to output a PWM signal at a desired frequency, resolution, and channel
 * and starts the pwm generation
 * More info: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
 *
 *  pin:    Specifies the pin we wish to output the indicated channel to
 *          This corresponds to the GPIO pin number (range 0 - 39)
 * 
 *  freq:   Specifies the PWM frequency in hertz (range 4 - 312500). 
 *          Since each period of the PWM signal is broken into 256 parts (8 bit counter), 
 *          the "counting frequency" [Hz] will be freq*256.
 * 
 *  chan:   Specifies which of the 16 available channels we want to setup (range 0 - 15)
 *          
 *
 *  returns: void
 */
void attachPwm(uint8_t pin, uint32_t freq, uint8_t chan); // new function


/*
 * Function:  setPwmDuty
 * --------------------
 *  sets the duty cycle of a given pwm channel
 *
 *  chan:   Specifies which of the 16 available channels we want to setup (range 0 - 15)
 *
 *  duty:   Specifies the duty cycle, where 0 is 0% and 255 is 100% (range 0 - 255)
 *
 *  returns: void
 */
void setPwmDuty(uint8_t chan, uint16_t duty);

/*
 * Function:  pwmDetach
 * --------------------
 *  detaches the indicate pin form the pwm channel
 *
 *  pin:      Specifies the pin we wish to output the indicated channel to
 *            This corresponds to the GPIO pin number (range 0 - 39)
 *
 *  returns:  void
 */
void pwmDetach(uint8_t pin);

/*
 * Function:  setUpTimer
 * --------------------
 *  configures and starts a timer at 1 MHz (up to 4 can be initialized)
 * 
 *  returns:  hw_timer_t* (timer struct, NULL if error)
 */
hw_timer_t* setUpTimer();

/*
 * Function:  startTimer
 * --------------------
 *  configures and starts a timed interrupt with the desired period and callback function
 *  the counting frequency is 1 MHz
 * 
 *  timer:          a hw_timer_t pointer which points to the desired timer struct 
 *  
 *  void (*f)():    a function pointer (which points to the desired callback function)
 *                  to use this, simply write the name of the callback function
 *
 *  period:         indicates the desired period (in microseconds) of the timed interrupt 
 *
 *  returns:        void
 */

void startTimer(hw_timer_t *timer, void (*f)(), uint64_t period);

/*
 * Function:  stopTimer
 * --------------------
 *  stops the indicated interrupt generation timer
 *
 *  timer:          a hw_timer_t pointer which points to the desired timer struct 
 *
 *  returns:        void
 */
void stopTimer(hw_timer_t *timer);

/*
 * Function:  writeToDAC
 * --------------------
 *  Writes a Voltage for the indicated DAC to output
 *
 *  pin:        indicates which of the 2 DAC pins we want to write to (GPIO25 or GPIO26)
 *  
 *  value:      determines voltage to output, where 0 = 0V and 255 = 3.3V 
 *
 *  returns:    void
 */
void writeToDAC(uint8_t pin, uint8_t value);

/*
 * Function:  stopDAC
 * --------------------
 *  Stop DAC output on given pin
 *
 *  pin:        indicates which of the 2 DAC pins we want to write to (GPIO25 or GPIO26)
 * 
 *  returns:    void
 */
void stopDAC(uint8_t pin);

#endif
