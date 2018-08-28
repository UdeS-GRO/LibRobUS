/*
Projet S1 2018
Class to control the PWM sent to the motors drives
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/
#include "MotorControl.h"


void MotorControl::init(uint8_t pwm_pin, uint8_t dir_pin) {
  // For each defined motor
  PWM_PIN_ = pwm_pin;
  DIR_PIN_ = dir_pin;
  //setup PWM and direction pins
  pinMode(PWM_PIN_, OUTPUT);
  pinMode(DIR_PIN_, OUTPUT);

}

void MotorControl::setSpeed(float speed) {
  int pwm = abs(speed*255.0f); // Change speed to unsigned integer
  // Switching polarity on DIR_PIN to change motor direction
  if (speed > 0) {
    digitalWrite(DIR_PIN_, LOW);
  } else {
    digitalWrite(DIR_PIN_, HIGH);
  }
  // SetPWM value
  analogWrite(PWM_PIN_, (pwm > 255) ? 255 : static_cast<uint8_t>(pwm));
}
