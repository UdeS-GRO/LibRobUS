/*
Projet S1 2018
Class to interface some of the ArduinoX "on board" functionnalities
@author Jean-Samuel Lauzon, Jonathan Vincent, Marc Rousseau
@version 1.0 26/05/2018
*/

#include <LibRobus.h>

// Objects creation
  Robus __Robus__;
  ArduinoX __AX__;
  SoftTimer __timer__[MAX_N_TIMER];
  AudioPlayer __audio__;
  DisplayLCD __display__;
  VexQuadEncoder __vex__;
  IRrecv __irrecv__(IR_RECV_PIN);

// Global variables
  // Bluetooth
  void (*BT_func)() = NULL;
  String BlUETOOTH_MSG = "";
  decode_results IR_MSG;


void BoardInit(){
  // Initialize debug communication on Serial0
  Serial.begin(BAUD_RATE_SERIAL0);
  
  // Init ArduinoX
  __AX__.init();

  // Init Robus
  __Robus__.init();

  // init telecommande
  __irrecv__.enableIRIn(); // Start the receiver
};

void AudioInit(){
  __audio__.init(SerialAudio);
}

void DisplayInit(){
  __display__.init();
}

void BluetoothInit(){
  // Init serial bluetooth
  SerialBT.begin(BAUD_RATE_BLUETOOTH);
}

void MOTOR_SetSpeed(uint8_t id, float speed){
  __AX__.setSpeedMotor(id, speed);
};

int32_t ENCODER_Read(uint8_t id){
  return __AX__.readEncoder(id);
};

void ENCODER_Reset(uint8_t id){
  __AX__.resetEncoder(id);
};

int32_t ENCODER_ReadReset(uint8_t id){
  return __AX__.readResetEncoder(id);
};

void AUDIO_Play(uint16_t track){
  __audio__.play(track);
};

void AUDIO_PlayBlocking(uint16_t track){
  __audio__.playBlocking(track);
};

void AUDIO_Next(){
  __audio__.next();
};

void AUDIO_Previous(){
  __audio__.previous();
};

void AUDIO_Pause(){
  __audio__.pause();
};

void AUDIO_Resume(){
  __audio__.resume();
};

void AUDIO_Stop(){
  __audio__.stop();
};

bool AUDIO_IsFinish(){
  return __audio__.isFinished();
};

void AUDIO_SetVolume(float volume){
  __audio__.setVolume(volume);
};

float SONAR_GetRange(uint8_t id){
  return __Robus__.getRangeSonar(id);
};

void DISPLAY_SetCursor(uint8_t row, uint8_t column){
  __display__.setCursor(column,row);
};

void DISPLAY_Printf(String msg){
  __display__.print(msg);
};

void DISPLAY_NewLine(){
  __display__.newLine();
};

void DISPLAY_Clear(){
  __display__.clear();
};

float AX_GetVoltage(){
  return __AX__.getVoltage();
};

float AX_GetCurrent(){
  return __AX__.getCurrent();
};

void AX_BuzzerON(){
  __AX__.buzzerOn();
};

void AX_BuzzerON(uint32_t freq){
  __AX__.buzzerOn(freq);
};

void AX_BuzzerON(uint32_t freq, uint64_t duration){
  __AX__.buzzerOn(freq, duration);
};

void AX_BuzzerOFF(){
  __AX__.buzzerOff();
};

bool AX_IsLowBat(){
  return __AX__.isLowBat();
};

bool ROBUS_IsBumper(uint8_t id){
  return __Robus__.isBumper(id);
};

uint16_t ROBUS_ReadIR(uint8_t id){
  return __Robus__.readIR(id);
};

void SERVO_Enable(uint8_t id){
  __Robus__.enableServo(id);
}

void SERVO_Disable(uint8_t id){
  __Robus__.disableServo(id);
}

void SERVO_SetAngle(uint8_t id, uint8_t angle){
  __Robus__.setAngleServo(id, angle);
}

void SOFT_TIMER_SetCallback(uint8_t id, void (*func)()){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setCallback(func);
};

void SOFT_TIMER_SetDelay(uint8_t id, unsigned long delay){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setDelay(delay);
};

void SOFT_TIMER_SetRepetition(uint8_t id, int32_t nrep){
  if(id<0){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].setRepetition(nrep);
};

void SOFT_TIMER_Enable(uint8_t id){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].enable();
};

void SOFT_TIMER_Disable(uint8_t id){
  if(id<0 || id>MAX_N_TIMER){
    Serial.println("Invalid timer id!");
    return;
  }
  __timer__[id].disable();
};

void SOFT_TIMER_Update(){
  for(uint8_t id = 0; id<MAX_N_TIMER; id++){
    __timer__[id].update();
  };
};

void BLUETOOTH_print(String msg){
  SerialBT.print(msg);
};

void BLUETOOTH_println(String msg){
  SerialBT.println(msg);
};

void BLUETOOTH_setCallback(void (*f)()){
  BT_func = f;
};

void serialEvent2(){
  BT_func();
};

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

uint32_t REMOTE_read(){
  if (__irrecv__.decode(&IR_MSG)) {
    __irrecv__.resume(); // Receive the next value
    // Serial.println(IR_MSG.value);
    return IR_MSG.value;
  }
  return 0;
};
