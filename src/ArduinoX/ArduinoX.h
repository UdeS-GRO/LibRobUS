/*
Projet S1 2018
Class to interface some of the ArduinoX "on board" functionnalities
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/

#ifndef ArduinoX_H_
#define ArduinoX_H_

#include <Arduino.h>
#include <Adafruit_INA219/Adafruit_INA219.h> // For power usage statistics
#include <MotorControl/MotorControl.h>
#include <LS7366Counter/LS7366Counter.h>

#define LEFT 0
#define RIGHT 1
class ArduinoX
{
  public:
    /** Method to initialize pins and objects
    */
    void init();

    /** Method to turn on the buzzer
    */
    void buzzerOn();

    /** Method to turn on the buzzer at a frequency
    */
    void buzzerOn(uint32_t freq);

    /** Method to turn on the buzzer at a frequency for
     * a certain duration
    */
    void buzzerOn(uint32_t freq, uint64_t duration);

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

    /** Method to set speed (direction and pwm) to a motor drive
    
    @param id
    identification of motor [0,1]
    
    @param speed
    speed to send to the drive [-1.0, 1.0]
    */
    void setSpeedMotor(uint8_t id, float speed);

    /** Method read the count of pulses from a quadrature encoder
    
    @param id
    identification of encoder [0,1]
    
    @return number of pulses
    */
    int32_t readEncoder(uint8_t id);

    /** Method read the count of pulses from a quadrature encoder
     * then reset the counter.
    
    @param id
    identification of encoder [0,1]
    
    @return number of pulses
    */
    int32_t readResetEncoder(uint8_t id);
    
    /** Method read reset the count of pulses to zero
    
    @param id
    identification of encoder [0,1]
    */
    void resetEncoder(uint8_t id);

  private:
    const uint8_t LOWBAT_PIN =  12;
    const uint8_t BUZZER_PIN =  36;
    const uint8_t MOTOR_PWM_PIN[2] =  {6, 5};
    const uint8_t MOTOR_DIR_PIN[2] =  {31, 30};
    const uint8_t COUNTER_SLAVE_PIN[2] =  {35, 34};
    const uint8_t COUNTER_FLAG_PIN[2] =  {A15, A14};
    Adafruit_INA219 ina219;
    MotorControl __motor__[2];
    LS7366Counter __encoder__[2];

};
#endif //ArduinoX
