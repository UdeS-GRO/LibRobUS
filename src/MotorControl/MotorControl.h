/*
Projet S1 2018
Class to control the PWM sent to the motors drives
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/
#ifndef MotorControl_H_
#define MotorControl_H_

#include <Arduino.h>
class MotorControl
{
  public:
    /** Method to initialize the pins for motors control
    */
    void init(uint8_t pwm_pin, uint8_t dir_pin);

    /** Method to set the speed and direction of a DC motor

    @param speed [-1.0, 1.0]
    float to specify the PWM value [0, 255] and direction for the drive
    */
    void setSpeed(float speed);

  private:
    // Pins for PWM and DIRECTION
    uint8_t PWM_PIN_ ;// {5, 6};    // PWM pins
    uint8_t DIR_PIN_ ;// {30, 31};  // direction pins
};
#endif //MotorControl
