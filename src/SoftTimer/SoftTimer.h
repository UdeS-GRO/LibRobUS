/*
Projet S1 2018
Class to start a function after a certain delay or at certain frequency
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 15/08/2018
*/

#ifndef SoftTimer_H_
#define SoftTimer_H_

#include <Arduino.h>

class SoftTimer
{
  public:
    SoftTimer();

    /** Function to set a delay between callback of a timer

    @param delay
    delay in millisecond between call
    */
    void setDelay(unsigned long delay){ delay_ = delay; };

    /** Function to a number of repetition of the callback before desabling it

    @param nrep
    number of repetition
    @note if nrep is negative or is not set, the timer will run forever (default);
    */
    void setRepetition(int32_t nrep){ nRep_ = nrep; };

    /** Method to set a callback to a timer

    @param function to set
    A function that returns void with no parameters
    */
    void setCallback(void (*func)()){f_ = func;};

    /** Method to call callback if callTime_ is passed

    @note the next callTime_ and nRep_ is computed if necessary
    */
    void update();
    /** Function to a enable timer
    */
    void enable();

    /** Function to a disable timer
    */
    void disable();

  private:
    void (*f_)(); // void, parameterless, function pointer
    unsigned long delay_; // Delay (ms) between callback
    unsigned long callTime_; // Time at which to call function
    bool enable_; // Timer state
    int32_t nRep_; // Number of iteration (-1 is infinite)
    int32_t currRep_;// Number of iteration left
};
#endif //SoftTimer
