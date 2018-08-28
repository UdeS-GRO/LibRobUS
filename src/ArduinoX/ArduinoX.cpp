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
  for(uint8_t i = 0; i < 4; i++){
    pinMode(BUMPER_PIN[i], INPUT);
  }
}

void ArduinoX::buzzerOn(){
  digitalWrite(BUZZER_PIN, HIGH);
}

void ArduinoX::buzzerFreq(uint32_t freq, uint64_t duration){
  tone(BUZZER_PIN, freq, duration);
}

void ArduinoX::buzzerFreq(uint32_t freq){
  tone(BUZZER_PIN, freq);
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

bool ArduinoX::isBumper(uint8_t id){
  if(id<0 || id>4){
    Serial.println("Invalid Bumper id!");
    return false;
  }
  return digitalRead(BUMPER_PIN[id]);
}

uint16_t ArduinoX::readIR(uint8_t id){
  if(id<0 || id>4){
    Serial.println("Invalid IR id!");
    return 0;
  }
  return analogRead(IR_PIN[id]);
}
