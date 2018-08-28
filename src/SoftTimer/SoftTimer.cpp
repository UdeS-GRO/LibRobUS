/*
Projet S1 2018
Class to start a function after a certain delay or at certain frequency
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 15/08/2018
*/

#include "SoftTimer.h"

SoftTimer::SoftTimer(){
  delay_ = 0;
  callTime_ = 0;
  nRep_ = -1;
  currRep_ = -1;
  enable_ = false;
  f_ = NULL;
};

void SoftTimer::update(){
  if(millis()>callTime_ && enable_ && nRep_){
    f_();
    if(nRep_ > 0){
      currRep_--;
      if(currRep_ == 0){
        disable();
      }
    }
    callTime_ = millis() + delay_;
  }
};

void SoftTimer::enable(){
  if(f_ != NULL){
    enable_ = true;
    callTime_ = millis() + delay_;
    currRep_ = nRep_;
  }
};

void SoftTimer::disable(){
  enable_ = false;
};
