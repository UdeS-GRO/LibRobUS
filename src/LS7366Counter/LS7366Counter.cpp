/*
Projet S1 2018
Class to communicate to LS7366 encoder module
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/
#include "LS7366Counter.h"

void LS7366Counter::init(uint8_t slave_pin, uint8_t flag_pin)
{
  SLAVE_PIN_ = slave_pin;
  FLAG_PIN_ = flag_pin;

  pinMode(SLAVE_PIN_, OUTPUT);    // Set slave pin as output
  pinMode(FLAG_PIN_, INPUT);      // Set flag pin as inpout
  digitalWrite(SLAVE_PIN_,HIGH);  // set slave pin to HIGH

  SPI.begin();                        // Start SPI communication
  delayMicroseconds(500);
  // Set encoder configuration

    // Communication when slave pin is low.
    digitalWrite(SLAVE_PIN_, LOW);
    SPI.transfer(0x88);                // Write to MDR0
    SPI.transfer(0x03);                // 4 bytes mode
    digitalWrite(SLAVE_PIN_, HIGH);
    delayMicroseconds(500);
    reset(); // Reset encoder to zero

}

int32_t LS7366Counter::read() {
  uint32_t count[4];
  int32_t count_value = 0;
  // If id is out of range
  digitalWrite(SLAVE_PIN_,LOW);        // Start communication
  SPI.transfer(0x60);                     // Read command

  // Read 4 bytes
  for(uint8_t i = 0; i < 4; i++) {
    count[i] = SPI.transfer(0x00);
  }
  digitalWrite(SLAVE_PIN_, HIGH);     // End of communication

  // Concatenate the four bytes
  for(uint8_t i = 0; i < 4; i++) {
    count_value = (count_value << 8) + count[i];
  }
  return -count_value;
}

void LS7366Counter::reset() {
  digitalWrite(SLAVE_PIN_, LOW); // Start communication
  // write to DTR
  SPI.transfer(0x98);
  for(uint8_t i = 0; i < 4; i++) {
    SPI.transfer(0x00); // Set register to zero
  }
  digitalWrite(SLAVE_PIN_, HIGH); // end communication
  delayMicroseconds(100);

  digitalWrite(SLAVE_PIN_, LOW);  // Start communication
  SPI.transfer(0xE0);                // Set Data to center
  digitalWrite(SLAVE_PIN_, HIGH); // end communication
}

int32_t LS7366Counter::readReset() {
  int32_t buffer = read();
  reset();
  return buffer;
}
