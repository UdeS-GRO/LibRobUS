/*
Projet S1 2018
Class to interface some of the ArduinoX "on board" functionnalities
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/

#include "ArduinoX.h"

void ArduinoX::init(){
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LOWBAT_PIN, INPUT);
  ina219.begin();
  for(uint8_t id = 0; id < 2; id++){
    __motor__[id].init(MOTOR_PWM_PIN[id], MOTOR_DIR_PIN[id]);
    __encoder__[id].init(COUNTER_SLAVE_PIN[id], COUNTER_FLAG_PIN[id]);
  }
}

void ArduinoX::buzzerOn(){
  digitalWrite(BUZZER_PIN, HIGH);
}

void ArduinoX::buzzerOn(uint32_t freq){
  tone(BUZZER_PIN, freq);
}

void ArduinoX::buzzerOn(uint32_t freq, uint64_t duration){
  tone(BUZZER_PIN, freq, duration);
}

void ArduinoX::buzzerOff(){
  digitalWrite(BUZZER_PIN, LOW);
}

float ArduinoX::getCurrent(){
  return ina219.getCurrent_mA()/1000.0;
}

float ArduinoX::getBusVoltage(){
  return ina219.getBusVoltage_V();
}

float ArduinoX::getShuntVoltage(){
  return ina219.getShuntVoltage_mV()/1000.0;
}

float ArduinoX::getVoltage(){
  return getBusVoltage() + getShuntVoltage();
}

bool ArduinoX::isLowBat(){
  return !digitalRead(LOWBAT_PIN);
}

void ArduinoX::setSpeedMotor(uint8_t id, float speed){
  if(id<0 || id>1){
    Serial.println("Invalid motor id!");
    return;
  }
  if(id==1){
    speed *= -1; // left motor is inverted
  }
  __motor__[id].setSpeed(speed);
}

int32_t ArduinoX::readEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return 0;
  }
  if(id == 0){
    return -__encoder__[id].read();// Left motor is inverted
  }else{
    return __encoder__[id].read();
  }
}

int32_t ArduinoX::readResetEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return 0;
  }
  if(id == 0){
    return -__encoder__[id].readReset();// Left motor is inverted
  }else{
    return __encoder__[id].readReset();
  }
}

void ArduinoX::resetEncoder(uint8_t id){
  if(id<0 || id>1){
    Serial.println("Invalid encoder id!");
    return;
  }
  __encoder__[id].reset();// Reset counter
}
