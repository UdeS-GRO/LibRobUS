/*
Projet S1 2018
Class to interface some of the ArduinoX "on board" functionnalities
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/

#ifndef ArduinoX_H_
#define ArduinoX_H_

#include <Arduino.h>
#include <Adafruit_INA219.h> // For power usage statistics

class ArduinoX
{
  public:
    /** Method to initialize pins and objects
    */
    void init();

    /** Method to turn on the buzzer
    */
    void buzzerOn();

    /** Method to turn on the buzzer periodically for a certain duration

    @param freq
    Frequency at which the buzzer is triggered (50% duty cycle)

    @param duration
    Duration of the pulses (in milliseconds)
    */
    void buzzerFreq(uint32_t freq, uint64_t duration);

    /** Method to turn on the buzzer periodically and indefinitely

    @param freq
    Frequency at which the buzzer is triggered (50% duty cycle)
    */
    void buzzerFreq(uint32_t freq);

    /** Method to turn off the buzzer
    */
    void buzzerOff();

    /** Method that returns current consumption

    @return current consumption in mA
    */
    float getCurrent();

    /** Method that returns the bus voltage

    @return voltage in Volts
    */
    float getBusVoltage();

    /** Method that returns the shunt resistor voltage

    @return voltage in mV
    */
    float getShuntVoltage();

    /** Method that returns the total input voltage

    @return voltage in Volts
    */
    float getVoltage();

    /** Method to verify if board is in low battery state

    @return true, if low battery. Else false.
    */
    bool isLowBat();

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

  private:
    const uint8_t LOWBAT_PIN =  12;
    const uint8_t BUZZER_PIN =  36;
    const uint8_t BUMPER_PIN[4] =  {27, 29, 26, 28};// (0: left, 1:rigth, 2:front, 3:rear)
    const uint8_t IR_PIN[4] =  {A0, A1, A2, A3};
    Adafruit_INA219 ina219;

};
#endif //ArduinoX
