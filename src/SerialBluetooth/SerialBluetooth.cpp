/*
Projet S1 2018
Class to communicate with Sunfounder Bluetooth module
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/

#include "SerialBluetooth.h"

void SerialBluetooth::init(HardwareSerial& serialCon){
  stream_ptr = &serialCon;
  serialCon.begin(BAUD_RATE_BLUETOOTH);
  while(!serialCon); // Attente d'ouverture du port serie
}

void SerialBluetooth::init(SoftwareSerial& serialCon){
  stream_ptr = &serialCon;
  serialCon.begin(BAUD_RATE_BLUETOOTH);
  while(!serialCon); // Attente d'ouverture du port serie
}

bool SerialBluetooth::read(String& msg){
  if(stream_ptr-> available()){
    msg = stream_ptr->readString();
    return true;
  }else{
    return false;
  }
}

void SerialBluetooth::println(String msg){
  stream_ptr->println(msg);
}
void SerialBluetooth::println(){
  stream_ptr->println();
}
void SerialBluetooth::print(String msg){
  stream_ptr->print(msg);
}
