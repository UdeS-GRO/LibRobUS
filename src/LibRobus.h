/*
Projet S1 2018
General librairies for Robus robot
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 09/06/2018
*/
#ifndef LibRobus_H_
#define LibRobus_H_

// Includes
// Custom libraries
#include <Robus/Robus.h>
#include <ArduinoX/ArduinoX.h>
#include <AudioPlayer/AudioPlayer.h>
#include <DisplayLCD/DisplayLCD.h>
#include <VexQuadEncoder/VexQuadEncoder.h>
#include <SoftTimer/SoftTimer.h>

// Third party libraries
#include <IRremote/IRremote.h>


// Defines
#define LEFT 0
#define RIGHT 1
#define FRONT 2
#define REAR 3

#define MAX_N_TIMER 10 // Nombre maximum de Chronometres
#define BAUD_RATE_SERIAL0 9600
#define BAUD_RATE_BLUETOOTH 115200
#define SerialBT Serial2
#define SerialAudio Serial3
#define IR_RECV_PIN 37


/** Function to initialize most variables to use in code
*/
void BoardInit();


/** Function to initialize audio variables
*/
void AudioInit();

/** Function to initialize Display variables
*/
void DisplayInit();


/** Function to initialize Bluetooth variables
*/
void BluetoothInit();



/** Function to control the two DC motors on robots

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@param speed, reprensents direction and amplitude of PWM
floating value between [-1.0, 1.0]
*/
void MOTOR_SetSpeed(uint8_t id, float speed);


/** Function to read the number of pulses from the encoder counter

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@return number of pulses on a int32 [–2147483648, 2147483647]
*/
int32_t ENCODER_Read(uint8_t id);

/** Function to reinitialize the number of pulses on counter

@param id
identification of the motor (LEFT(0) or RIGHT(1))
*/
void ENCODER_Reset(uint8_t id);

/** Function to read the number of pulses from a counter, then reset its value

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@return number of pulses on a int32 [–2147483648, 2147483647]
*/
int32_t ENCODER_ReadReset(uint8_t id);

/** Function to play an audio track on mp3 player
This function is non-blocking

@param track
the index of the track (in last modify order (starts from 1))
*/
void AUDIO_Play(uint16_t track);

/** Function to play an audio track on mp3 player
This function is blocking(code will stay in this function until track is over)

@param track
the index of the track (in last modify order (starts from 1))
*/
void AUDIO_PlayBlocking(uint16_t track);

/** Function to play next audio track on mp3 player
*/
void AUDIO_Next();

/** Function to play previous audio track on mp3 player
*/
void AUDIO_Previous();

/** Function to pause currently playing audio track on mp3 player
*/
void AUDIO_Pause();

/** Function to resume currently paused audio track on mp3 player
*/
void AUDIO_Resume();

/** Function to stop current audio track on mp3 player
*/
void AUDIO_Stop();

/** Function to poll the the state of the current track

@return true if song is finished, else false
*/
bool AUDIO_IsFinish();

/** Function set the audio volume of the mp3 player

@param Volume
floating value between [0.0, 1.0]
*/
void AUDIO_SetVolume(float volume);

/** Function to estimate the range with the sonar
@param id
identification of the sonar
@return range
Estimation of the range in meters.
*/
float SONAR_GetRange(uint8_t id);

/** Function to set cursor position
@note For I2C 4x20 LCD display

@param row
row index [0, 3]

@param column
column index [0, 19]
*/
void DISPLAY_SetCursor(uint8_t row, uint8_t column);

/** Function to print message at the cursor position
@note For I2C 4x20 LCD display

@param msg
Alphanumeric string message
*/
void DISPLAY_Printf(String msg);

/** Function to move cursor to beginning of next row
@note For I2C 4x20 LCD display
*/
void DISPLAY_NewLine();

/** Function to clear the display
@note For I2C 4x20 LCD display
*/
void DISPLAY_Clear();

/** Function that return the voltage input of ArduinoX
@return volatge in V
*/
float AX_GetVoltage();

/** Function that return the current input of ArduinoX
@return current in mA
*/
float AX_GetCurrent();

/** Function turn on the onboard buzzer
*/
void AX_BuzzerON();
/** Function turn on the onboard buzzer
@param freq
frequency of a 50% dutycycle square wave
*/
void AX_BuzzerON(uint32_t freq);

/** Function turn on the onboard buzzer
@param freq
frequency of a 50% dutycycle square wave
@param duration
time (ms) buzzer is active
*/
void AX_BuzzerON(uint32_t freq, uint64_t duration);

/** Function turn off the onboard buzzer
*/
void AX_BuzzerOFF();

/** Function to inquire the battery level
return true if batterie is low, else false
*/
bool AX_IsLowBat();

/** Function to inquire a bumber state
@param id
index of the desired bumper (LEFT:0, RIGHT:1, FRONT:2, REAR:3)

return true if bumper is pressed, else false
*/
bool ROBUS_IsBumper(uint8_t id);

/** Function to raw imput fromt infrared captor
@param id
index of the desired IR module [0, 3]

return number on 16bits
*/
uint16_t ROBUS_ReadIR(uint8_t id);

/** Function to enable a servomotor
@param id
index of the desired servomotor [0, 1]
*/
void SERVO_Enable(uint8_t id);

/** Function to disable a servomotor
@param id
index of the desired servomotor [0, 1]
*/
void SERVO_Disable(uint8_t id);

/** Function to set angle to a Servomotor
@note The servo must be enable before

@param id
index of the desired servomotor [0, 1]

@param angle
An angle value in the range defined in global variable
*/
void SERVO_SetAngle(uint8_t id, uint8_t angle);

/** Function to set a callback to a timer

@param id
index of the timer [0, MAX_N_TIMER-1]

@param function to set
A function that returns void with no parameters
*/
void SOFT_TIMER_SetCallback(uint8_t id, void (*func)());

/** Function to set a delay between callback of a timer

@param id
index of the timer [0, MAX_N_TIMER-1]

@param delay
delay in millisecond between call
*/
void SOFT_TIMER_SetDelay(uint8_t id, unsigned long delay);

/** Function to a number of repetition of the callback before desabling it

@param id
index of the timer [0, MAX_N_TIMER-1]

@param nrep
number of repetition
@note if nrep is negative or is not set, the timer will run forever (default);
*/
void SOFT_TIMER_SetRepetition(uint8_t id, int32_t nrep);

/** Function to a enable a timer

@param id
index of the timer [0, MAX_N_TIMER-1]
*/
void SOFT_TIMER_Enable(uint8_t id);

/** Function to a disable a timer

@param id
index of the timer [0, MAX_N_TIMER-1]
*/
void SOFT_TIMER_Disable(uint8_t id);


/** Function to call a callback if callTime_ is passed for a timer

@note the next callTime_ and nRep_ is computed if necessary
*/
void SOFT_TIMER_Update();

/** Function to write a message to Bluetooth module
@note for Sunfounder Serial Bluetooth

@param msg
a string message to be sent via bluetooth to paired
*/
void BLUETOOTH_print(String msg);

/** Function to write a message to Bluetooth module with a end line
@note for Sunfounder Serial Bluetooth, will send right away

@param msg
a string message to be sent via bluetooth to paired
*/
void BLUETOOTH_println(String msg);

/** Function to set callback when data from BlueTooth
@note for Sunfounder Serial Bluetooth

@param f
a void fonction without parameters
*/
void BLUETOOTH_setCallback(void (*f)());


/** Function called when data on Serial2
*/
void serialEvent2();

/** Function that defines the default callback when data is receive from bluetooth
@note this can be used as a callback

*/
void BLUETOOTH_readCallback();

/** Function that defines the default callback when data is receive from bluetooth

@return the received string
*/
String BLUETOOTH_read();

/** Function to return decoded ir message
@return 0 if no message

@return code on a uint32_t

*/
uint32_t REMOTE_read();

/** allow inline printing (c++ style);
*/
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
#define endl "\r\n"

#endif //LibRobus
