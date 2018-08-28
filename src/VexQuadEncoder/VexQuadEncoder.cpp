/*
Projet S1 2018
Class to interface the Vex Quadrature encoder
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau 
@version 1.0 04/06/2018
*/
#include "VexQuadEncoder.h"

void VexQuadEncoder::init(uint8_t pin_ch1, uint8_t pin_ch2){
  PIN_CH1_ = pin_ch1;
  PIN_CH2_ = pin_ch2;
  pinMode(PIN_CH1_, INPUT_PULLUP);
  pinMode(PIN_CH2_, INPUT_PULLUP);
}

void VexQuadEncoder::isr(){
  if(digitalRead(PIN_CH2_)){
    counter_ += 1;
  }else{
    counter_ -= 1;
  }
}
