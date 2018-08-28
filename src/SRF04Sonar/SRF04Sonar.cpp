/*
Projet S1 2018
Class to interface the sonar module SRF04
@author Jean-Samuel Lauzon, Jonathan Vincent, Serge Caron
@version 1.0 26/05/2018
*/
#include "SRF04Sonar.h"

void SRF04Sonar::init(uint8_t echoPin, uint8_t trigPin){
  ECHO_PIN = echoPin;
  TRIG_PIN = trigPin;
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW); // Set the trigger pin to low
}

float SRF04Sonar::getRange(){
  digitalWrite(TRIG_PIN, LOW);    // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);   // Send a 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);    // Send pin low again
  float range = pulseIn(ECHO_PIN, HIGH)/58.0;
  return range; // Read in times pulse
};
