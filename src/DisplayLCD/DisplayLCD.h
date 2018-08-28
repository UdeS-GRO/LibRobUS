/*
Projet S1 2018
General librairies for displaying LCD like a terminal
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 11/06/2018
*/

#ifndef DisplayLCD_H_
#define DisplayLCD_H_

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>



class DisplayLCD
{
  public:
    /** Method to initialize the LCD
    */
    void init();

    /** Method to set cursor

    @param column
    specifies the column between [0, N_COL]

    @param row
    specifies the row between [0, N_ROW]
    */
    void setCursor(uint8_t column, uint8_t row);

    /** Method to print to LCD

    @param msg
    String message
    */
    void print(String msg);

    /** Method to switch to next row
    */
    void newLine();

    /** Method to clear LCD
    */
    void clear();

  private:

    /** Method to clear a line (from current column)
    */
    void clearLine();
    constexpr static uint8_t ADDRESS = 0x27;
    constexpr static uint8_t N_COL = 20;
    constexpr static uint8_t N_ROW = 4;
    LiquidCrystal_I2C* display_ = new LiquidCrystal_I2C(ADDRESS, N_COL ,N_ROW);
    uint8_t cur_row_ = 0;
    uint8_t cur_col_ = 0;

};
#endif // DisplayLCD
