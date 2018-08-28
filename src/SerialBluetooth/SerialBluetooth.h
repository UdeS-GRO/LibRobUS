/*
Projet S1 2018
Class to communicate with Sunfounder Bluetooth module
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/

#ifndef SerialBluetooth_H_
#define SerialBluetooth_H_

#include <Arduino.h>
#include <Stream.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define BAUD_RATE_BLUETOOTH 115200

class SerialBluetooth
{
  public:

    /** Method to initialize communication between arduino
    and the Sunfounder bluetooth device.

    @param serialCon
    A HardwareSerial object (ex. Serial1).
    */
    void init(HardwareSerial& serialCon);

    /** Method to initialize communication between arduino
    and the Sunfounder bluetooth device.

    @param serialCon
    A SoftwareSerial object.
    */
    void init(SoftwareSerial& serialCon);

    /** Method to read new available messages on serial port
    @param    msg
    A String variable to store the message.

    @return   false if message is empty (no new message available). Else true.
    */
    bool read(String& msg);

    /** Method to print message (plus a newline) to serial port

    @param msg
    A String message to print
    */

    void println(String msg);
    /** Method to print message (plus a newline) to serial port

    @param msg
    A String message to print
    */
    void println();

    /** Method to print to serial port

    @param msg
    A String message to print
    */
    void print(String msg);

  private:
    Stream* stream_ptr;
};
#endif //SerialBluetooth_H_
