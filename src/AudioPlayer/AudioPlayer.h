/*
Projet S1 2018
Class to communicate with SeeedStudio audioplayer module
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 26/05/2018
*/

#ifndef AudioPlayer_H_
#define AudioPlayer_H_

#include <Arduino.h>
#include <Stream.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#define START_BYTE      0x7E
#define END_BYTE        0xEF
#define VERSION_BYTE    0xFF
#define NO_REPLY_BYTE   0x00
#define REPLY_BYTE      0x01

//COMMANDS
#define CMD_NEXT        0X01
#define CMD_PREV        0X02
#define CMD_SET_TRACK   0X03
#define CMD_VOL_UP      0X04
#define CMD_VOL_DOWN    0X05
#define CMD_SET_VOL     0X06

#define CMD_SET_EQ      0X07
#define CMD_REPEAT      0X08
#define CMD_SET_DEVICE  0X09
#define CMD_STANDBY     0X0A
#define CMD_RESET       0X0C
#define CMD_PLAY        0X0D
#define CMD_PAUSE       0X0E
#define CMD_STOP        0X16

// others
#define BAUD_RATE_LECTEURAUDIO    9600
#define MSG_SIZE     8

class AudioPlayer
{
  public:
    /** Method to initialize communication between arduino
    and the SeeedStudio audioplayer device.

    @param serialCon
    A predefined HardwareSerial object (ex. Serial1).
    */
    void init(HardwareSerial&);

    /** Method to initialize communication between arduino
    and the SeeedStudio audioplayer device.

    @param serialCon
    A SoftwareSerial object.
    */
    void init(SoftwareSerial&);

    /** Method to select and play a specific track

    @param track
    Index of the disired track (starts at 0).
    */
    void play(uint16_t track);

    /** Method to select and play a specific track
    (this fonction is blocking until track is over)

    @param track
    Index of the disired track (starts at 0).
    */
    void playBlocking(uint16_t track);

    /** Method to pause the current playing track

    */
    void pause();

    /** Method to resume the current paused track
    */
    void resume();

    /** Method to pause the current playing track
    */
    void stop();

    /** Method to play next track
    */
    void next();

    /** Method to play previous track
    */
    void previous();

    /** Method to set the volume level

    @param volume
    A floating value between [0.0, 1.0].
    */
    void setVolume(float volume);

    /** Method to poll the state of the audioplayer to block code

    @return state of the audio player
    false: Song is not finished, true: Song is finished

    @note The module returns a message (only once!) after a song is finished
    example of use: While(~obj.isFinished());
    */
    bool isFinished();
  private:

    /** Method to select the storage device

    @param device [0, 1]
    An usigned integer (0: for Ucard, 1:for SDcard).
    */
    void selectDevice(uint8_t device);

    /** Method to write to the audioplayer device

    @param array
    A byte array pointer containing instructions.

    @param len
    the length of the array.
    */
    void writeMsg(uint8_t * array, uint8_t len);

    /** Method to print to debug console

    @param debug
    A HardwareSerial to print to.

    @param array
    A byte array pointer containing instructions.

    @param len
    the length of the array.
    */
    void print_HEX(HardwareSerial& debug, uint8_t * array, uint8_t len);
    Stream *player;
};
#endif //AudioPlayer
