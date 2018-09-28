/*
Projet S1 2018
General librairies for Robus robot
@author Jean-Samuel Lauzon, Jonathan Vincent
@version 1.0 09/06/2018
*/
#ifndef LibRobus_H_
#define LibRobus_H_

// Includes
  // Custom librairies
  #include <Robus/Robus.h>
  #include <ArduinoX/ArduinoX.h>
  #include <AudioPlayer/AudioPlayer.h>
  #include <DisplayLCD/DisplayLCD.h>
  #include <VexQuadEncoder/VexQuadEncoder.h>
  #include <SoftTimer/SoftTimer.h>


// Defines
#define LEFT 0
#define RIGHT 1
#define FRONT 2
#define REAR 3

#define MAX_N_TIMER 5 // Nombre maximum de Chronometres
#define BAUD_RATE_SERIAL0 9600
#define BAUD_RATE_BLUETOOTH 115200
#define SerialBT Serial2
#define SerialAudio Serial3

// Objects creation
  Robus __Robus__;
  ArduinoX __AX__;
  SoftTimer __timer__[MAX_N_TIMER];
  AudioPlayer __audio__;
  DisplayLCD __display__;
  VexQuadEncoder __vex__;

// Global variables
  // Bluetooth
  void (*BT_func)() = NULL;
  String BlUETOOTH_MSG = "";


/** Function to initialize most variables to use in code
*/
void BoardInit(){
  // Initialize debug communication on Serial0
  Serial.begin(BAUD_RATE_SERIAL0);
  
  // Init ArduinoX
  __AX__.init();

  // Init Robus
  __Robus__.init();
};

/** Function to initialize audio variables
*/
void AudioInit(){
  __audio__.init(SerialAudio);
}

/** Function to initialize Display variables
*/
void DisplayInit(){
  __display__.init();
}

/** Function to initialize Bluetooth variables
*/
void BluetoothInit(){
  // Init serial bluetooth
  SerialBT.begin(BAUD_RATE_BLUETOOTH);
}



/** Function to control the two DC motors on robots

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@param speed, reprensents direction and amplitude of PWM
floating value between [-1.0, 1.0]
*/
void MOTOR_SetSpeed(uint8_t id, float speed){
  __AX__.setSpeedMotor(id, speed);
};


/** Function to read the number of pulses from the encoder counter

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@return number of pulses on a int32 [–2147483648, 2147483647]
*/
int32_t ENCODER_Read(uint8_t id){
  return __AX__.readEncoder(id);
};

/** Function to reinitialize the number of pulses on counter

@param id
identification of the motor (LEFT(0) or RIGHT(1))
*/
void ENCODER_Reset(uint8_t id){
  __AX__.resetEncoder(id);
};

/** Function to read the number of pulses from a counter, then reset its value

@param id
identification of the motor (LEFT(0) or RIGHT(1))

@return number of pulses on a int32 [–2147483648, 2147483647]
*/
int32_t ENCODER_ReadReset(uint8_t id){
  return __AX__.readResetEncoder(id);
};

/** Function to play an audio track on mp3 player
This function is non-blocking

@param track
the index of the track (in last modify order (starts from 1))
*/
void AUDIO_Play(uint16_t track){
  __audio__.play(track);
};

/** Function to play an audio track on mp3 player
This function is blocking(code will stay in this function until track is over)

@param track
the index of the track (in last modify order (starts from 1))
*/
void AUDIO_PlayBlocking(uint16_t track){
  __audio__.playBlocking(track);
};

/** Function to play next audio track on mp3 player
*/
void AUDIO_Next(){
  __audio__.next();
};

/** Function to play previous audio track on mp3 player
*/
void AUDIO_Previous(){
  __audio__.previous();
};

/** Function to pause currently playing audio track on mp3 player
*/
void AUDIO_Pause(){
  __audio__.pause();
};

/** Function to resume currently paused audio track on mp3 player
*/
void AUDIO_Resume(){
  __audio__.resume();
};

/** Function to stop current audio track on mp3 player
*/
void AUDIO_Stop(){
  __audio__.stop();
};

/** Function to poll the the state of the current track

@return true if song is finished, else false
*/
bool AUDIO_IsFinish(){
  return __audio__.isFinished();
};

/** Function set the audio volume of the mp3 player

@param Volume
floating value between [0.0, 1.0]
*/
void AUDIO_SetVolume(float volume){
  __audio__.setVolume(volume);
};

/** Function to estimate the range with the sonar
@param id
identification of the sonar
@return range
Estimation of the range in meters.
*/
float SONAR_GetRange(uint8_t id){
  return __Robus__.getRangeSonar(id);
};

/** Function to set cursor position
@note For I2C 4x20 LCD display

@param row
row index [0, 3]

@param column
column index [0, 19]
*/
void DISPLAY_SetCursor(uint8_t row, uint8_t column){
  __display__.setCursor(column,row);
};

/** Function to print message at the cursor position
@note For I2C 4x20 LCD display

@param msg
Alphanumeric string message
*/
void DISPLAY_Printf(String msg){
  __display__.print(msg);
};

/** Function to move cursor to beginning of next row
@note For I2C 4x20 LCD display
*/
void DISPLAY_NewLine(){
  __display__.newLine();
};

/** Function to clear the display
@note For I2C 4x20 LCD display
*/
void DISPLAY_Clear(){
  __display__.clear();
};

/** Function that return the voltage input of ArduinoX
@return volatge in V
*/
float AX_GetVoltage(){
  return __AX__.getVoltage();
};

/** Function that return the current input of ArduinoX
@return current in mA
*/
float AX_GetCurrent(){
  return __AX__.getCurrent();
};

/** Function turn on the onboard buzzer
*/
void AX_BuzzerON(){
  __AX__.buzzerOn();
};

/** Function turn on the onboard buzzer
@param freq
frequency of a 50% dutycycle square wave
*/
void AX_BuzzerON(uint32_t freq){
  __AX__.buzzerFreq(freq);
};

/** Function turn on the onboard buzzer
@param freq
frequency of a 50% dutycycle square wave

@param duration
time (ms) buzzer is active
*/
void AX_BuzzerON(uint32_t freq, uint64_t duration){
  __AX__.buzzerFreq(freq, duration);
};


/** Function turn off the onboard buzzer
*/
void AX_BuzzerOFF(){
  __AX__.buzzerOff();
};

/** Function to inquire the battery level
return true if batterie is low, else false
*/
bool AX_IsLowBat(){
  return __AX__.isLowBat();
};

/** Function to inquire a bumber state
@param id
index of the desired bumper (LEFT:0, RIGHT:1, FRONT:2, REAR:3)

return true if bumper is pressed, else false
*/
bool ROBUS_IsBumper(uint8_t id){
  return __Robus__.isBumper(id);
};

/** Function to raw imput fromt infrared captor
@param id
index of the desired IR module [0, 3]

return number on 16bits
*/
uint16_t ROBUS_ReadIR(uint8_t id){
  return __Robus__.readIR(id);
};


/** Function to enable a servomotor
@param id
index of the desired servomotor [0, 1]
*/
void SERVO_Enable(uint8_t id){
  __Robus__.enableServo(id);
}

/** Function to disable a servomotor
@param id
index of the desired servomotor [0, 1]
*/
void SERVO_Disable(uint8_t id){
  __Robus__.disableServo(id);
}

/** Function to set angle to a Servomotor
@note The servo must be enable before

@param id
index of the desired servomotor [0, 1]

@param angle
An angle value in the range defined in global variable
*/
void SERVO_SetAngle(uint8_t id, uint8_t angle){
  __Robus__.setAngleServo(id, angle);
}

/** Function to set a callback to a timer

@param id
index of the timer [0, MAX_N_TIMER-1]

@param function to set
A function that returns void with no parameters
*/
void SOFT_TIMER_SetCallback(uint8_t id, void (*func)()){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setCallback(func);
};

/** Function to set a delay between callback of a timer

@param id
index of the timer [0, MAX_N_TIMER-1]

@param delay
delay in millisecond between call
*/
void SOFT_TIMER_SetDelay(uint8_t id, unsigned long delay){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setDelay(delay);
};

/** Function to a number of repetition of the callback before desabling it

@param id
index of the timer [0, MAX_N_TIMER-1]

@param nrep
number of repetition
@note if nrep is negative or is not set, the timer will run forever (default);
*/
void SOFT_TIMER_SetRepetition(uint8_t id, int32_t nrep){
  if(id<0){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setRepetition(nrep);
};

/** Function to a enable a timer

@param id
index of the timer [0, MAX_N_TIMER-1]
*/
void SOFT_TIMER_Enable(uint8_t id){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].enable();
};

/** Function to a disable a timer

@param id
index of the timer [0, MAX_N_TIMER-1]
*/
void SOFT_TIMER_Disable(uint8_t id){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].disable();
};


/** Function to call a callback if callTime_ is passed for a timer

@note the next callTime_ and nRep_ is computed if necessary
*/
void SOFT_TIMER_Update(){
  for(uint8_t id = 0; id<MAX_N_TIMER; id++){
    __timer__[id].update();
  };
};

/** Function to write a message to Bluetooth module
@note for Sunfounder Serial Bluetooth

@param msg
a string message to be sent via bluetooth to paired
*/
void BLUETOOTH_print(String msg){
  SerialBT.print(msg);
};

/** Function to write a message to Bluetooth module with a end line
@note for Sunfounder Serial Bluetooth, will send right away

@param msg
a string message to be sent via bluetooth to paired
*/
void BLUETOOTH_println(String msg){
  SerialBT.println(msg);
};

/** Function to set callback when data from BlueTooth
@note for Sunfounder Serial Bluetooth

@param f
a void fonction without parameters
*/
void BLUETOOTH_setCallback(void (*f)()){
  BT_func = f;
};


/** Function called when data on Serial2
*/
void serialEvent2(){
  BT_func();
};

/** Function that defines the default callback when data is receive from bluetooth
@note this can be used as a callback

*/
void BLUETOOTH_readCallback(){
  String inputString;
  while (SerialBT.available()) {
    // get the new byte:
    char inChar = (char)SerialBT.read();
    // add it to the inputString:
    inputString += inChar;
    }
  BlUETOOTH_MSG = inputString;
}

/** Function that defines the default callback when data is receive from bluetooth

@return the received string
*/
String BLUETOOTH_read(){
  String inputString;
  while (SerialBT.available()) {
    // get the new byte:
    char inChar = (char)SerialBT.read();
    // add it to the inputString:
    inputString += inChar;
    }
  return inputString;
}

/** allow inline printing (c++ style);
*/
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
#define endl "\r\n"

#endif //LibRobus
