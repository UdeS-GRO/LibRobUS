/*
Projet S1 2018
Class to communicate to LS7366 encoder module
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/
#ifndef LS7366Counter_h
#define LS7366Counter_h

#include "Arduino.h"
#include <SPI.h>

class LS7366Counter
{
  public:
    /** Methode to initialize communication between arduino and encoder modules

    @param slave_pin
    Pin select for SPI communication

    @param flag_pin
    Pin for flag
    */
    void init(uint8_t slave_pin, uint8_t flag_pin);

    /** Method to read the number of steps on an encoder module

    @return number of steps [–2147483648, 2147483647]
    */
    int32_t read();

    /** Method to reset the number of steps on an encoder
    */
    void reset();   // Reinitialise le nombre de pulses

    /** Method to read the number of steps then reset
    @return number of steps [–2147483648, 2147483647]
    */
    int32_t readReset();

  private:
    uint8_t SLAVE_PIN_;// {34, 35}; // Slave select pins
    uint8_t FLAG_PIN_ ;// {A14, A15};
};
#endif // LS7366Counter


/*
		LS7366 Breakout    -------------   Arduino
		-----------------                  -------
            MOSI   -------------------   MOSI (D51)
            MISO   -------------------   MISO (D50)
            SCK    -------------------   SCK (D52)
            SS1    -------------------   SS1 (D34)
            SS2    -------------------   SS2 (D35)
            GND    -------------------   GND
            VDD    -------------------   VCC (5.0V)
*/
