/*
Projet S1 2018
Class to interface Robus board functionnalities
@author Jean-Samuel Lauzon, Serges Caron
@version 1.0 28/09/2018
*/

#ifndef Robus_H_
#define Robus_H_

#include <Arduino.h>
#include <MegaServo/MegaServo.h>
//#include <Servo.h>
#include <LS7366Counter/LS7366Counter.h>
#include <SRF04Sonar/SRF04Sonar.h>

#define SERVO_1 0
#define SERVO_2 1

#define SONAR_1 0
#define SONAR_2 1

#define FRONT 2
#define REAR 3

class Robus
{
  public:
    /** Method to initialize pins and objects
    */
    void init();

    /** Method to verify if a certain bumper is pressed

    @param id
    the id of the disired bumper (0: left, 1:rigth, 2:front, 3:rear)

    @return true, if bumper is pressed.
    */
    bool isBumper(uint8_t id);

    /** Method to read the analog value comming from the IR module

    @param id
    the id of the disired bumper (0: left, 1:rigth, 2:front, 3:rear)

    @return IR_value [0 1023] (in practice it stalls at around 512).
    */
    uint16_t readIR(uint8_t id);

    /** Method to enable a Servomotor that was disable
    @param id
    the id of the disired servo [0, 1]

    @return void.
    */
    void enableServo(uint8_t id);

    /** Method to disable a Servomotor
    @param id
    the id of the disired servo [0, 1]

    @return void.
    */
    void disableServo(uint8_t id);

    /** Method to set an angle of a Servomotor
    @param id
    the id of the disired servo [0, 1]
    
    @param angle
    the angle desired. The range is defined by __SERVO_RANGE__
    @return void.
    */
    void setAngleServo(uint8_t id, uint8_t angle);

    /** Method to get range in cm with a sonar
    @param id
    the id of the disired sonar [0, 1]
    
    @return estimated distance in cm
    */
    float getRangeSonar(uint8_t id);


  private:
    const uint8_t IR_PIN[4] =  {A0, A1, A2, A3};
    const uint8_t BUMPER_PIN[4] =  {27, 29, 26, 28};// (0: left, 1:rigth, 2:front, 3:rear)
    const uint8_t __SERVO_PINS__[2] = {4, 7};
    const uint8_t __SERVO_RANGE__[2] = {0, 180}; // 0 to 180 degree
    const uint8_t __SONAR_ECHO_PINS__[2] = {22, 24};
    const uint8_t __SONAR_TRIG_PINS__[2] = {23, 25};
    //Servo __servo__[2];
    MegaServo __servo__[2];
    SRF04Sonar __sonar__[2];
};
#endif //Robus_H_
