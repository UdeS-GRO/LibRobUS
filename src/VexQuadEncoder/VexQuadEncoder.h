/*
Projet S1 2018
Class to interface the Vex Quadrature encoder
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 04/06/2018
*/

#ifndef VexQuadEncoder_H_
#define VexQuadEncoder_H_
using callback_t = void(*)();

#include <Arduino.h>

class VexQuadEncoder
{
  public:
    /** Method to initialize pins for the Vex Quadrature encoder

    @param pin_ch1
    Digital pin number for channel 1

    @param pin_ch2
    Digital pin number for channel 2
    */
    void init(uint8_t pin_ch1, uint8_t pin_ch2);

    /** Interrupt Service Routine for the object
    */
    void isr();

    /** Method to return the counter variable

    @return counter
    Signed integer 32bits

    @note There are 90 slits per rotation
    */
    int32_t getCount(){return counter_;};

    /** Method to return the interrupt pin (Channel 1)

    @return interrupt pin
    uint8_t interrupt pin
    */
    uint8_t getPinInt(){return digitalPinToInterrupt(PIN_CH1_);};

    /** Method to return Channel 1 pin

    @return  pin
    uint8_t pin number
    */
    uint8_t getPinCh1(){return PIN_CH1_;};

    /** Method to return Channel 1 pin

    @return  pin
    uint8_t pin number
    */
    uint8_t getPinCh2(){return PIN_CH2_;};

    /** Method to reset counter_
    */
    void reset(){counter_=0;};

  private:
    uint8_t PIN_CH1_;
    uint8_t PIN_CH2_;
    volatile int32_t counter_;
};
#endif // VexQuadEncoder
/*
// Construction example
VexQuadEncoder vex;

// Initialisation example
void setupVex(){
  vex.init(2,3);
  attachInterrupt(vex.getPinInt(), []{vex.isr();}, FALLING);
}
*/
