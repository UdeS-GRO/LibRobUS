/*
Projet S1 2018
Class to interface the sonar module SRF04
@author Jean-Samuel Lauzon, Jonathan Vincent, Serge Caron
@version 1.0 26/05/2018
*/

#ifndef SRF04Sonar_H_
#define SRF04Sonar_H_

#include <Arduino.h>

class SRF04Sonar
{
  public:
    /** Method to initialize pins for sonar module

    @param echoPin
    Digital pin number to enable echoing

    @param trigPin
    Digital pin number to enable triggering
    */
    void init(uint8_t echoPin, uint8_t trigPin);

    /** Method to pulse sonar and compute the range according to delay

    @return estimated distance
    */
    float getRange();

  private:
    uint8_t ECHO_PIN; // Pin number for pulsing
    uint8_t TRIG_PIN; // Pin number for trigering
};
#endif //SRF04Sonar
