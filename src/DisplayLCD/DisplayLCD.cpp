/*
Projet S1 2018
General librairies for displaying LCD like a terminal
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 11/06/2018
*/

#include "DisplayLCD.h"
void DisplayLCD::init(){
  display_->init();
  display_->backlight();
  display_->setCursor(0, 0);
  display_->blink_on();
};

void DisplayLCD::setCursor(uint8_t column, uint8_t row){
  cur_col_ = column % N_COL;
  cur_row_ = row % N_ROW;
  display_->setCursor(cur_col_, cur_row_);
};

void DisplayLCD::print(String msg){
  //display_->setCursor(0, row);
  uint8_t c = 0; // counter
  uint8_t msg_len = msg.length();
  String sub_msg = "";
  while(msg_len > N_COL - cur_col_){
    sub_msg = msg.substring(c, c + N_COL- cur_col_);
    display_->print(sub_msg);
    c += N_COL- cur_col_;
    cur_row_ ++;
    cur_row_ %= N_ROW;
    msg_len -= N_COL - cur_col_;
    display_->setCursor(0, cur_row_);
    cur_col_ = 0;
  }
  sub_msg = msg.substring(c);
  display_->print(sub_msg);
  cur_col_ += sub_msg.length();
  clearLine(); // clear rest of the line
};


void DisplayLCD::newLine(){
  cur_row_ ++;
  cur_row_ %= N_ROW;
  cur_col_ = 0;
  display_->setCursor(cur_col_, cur_row_);
  clearLine();
};

void DisplayLCD::clear(){display_->clear();};


void DisplayLCD::clearLine(){
  // Put spaces for the rest of the line
  String buff = "";
  for(uint8_t c = cur_col_; c < N_COL; c++){
    buff += " ";
  }
  display_->print(buff);
  display_->setCursor(cur_col_, cur_row_);
};
