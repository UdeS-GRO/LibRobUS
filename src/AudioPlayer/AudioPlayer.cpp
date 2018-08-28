/* Fonctions de control du lecteur mp3 Seeed Studio
Auteur: Jean-Samuel Lauzon
Projet S1 2018
*/

#include "AudioPlayer.h"

void AudioPlayer::init(HardwareSerial& serialCon){
  player = &serialCon;
  serialCon.begin(BAUD_RATE_LECTEURAUDIO);
  while(!serialCon); // Wait for connection
  selectDevice(uint8_t(2)); // Selection of SDcard
  delay(20);
}

// initialisation de la communication de la classe
void AudioPlayer::init(SoftwareSerial& serialCon){
  player = &serialCon;
  serialCon.begin(BAUD_RATE_LECTEURAUDIO);
  while(!serialCon); // Attente d'ouverture du port seri
  selectDevice(uint8_t(2)); // Selection de la carte SD (2)
  delay(20);
}

void AudioPlayer::selectDevice(uint8_t device){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2; // Size without header and tail
  msg[3] = CMD_SET_DEVICE;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = device;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(200);
}

void AudioPlayer::play(uint16_t track){
  track ++; // AudioPlayer expect track to start at 0
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_SET_TRACK;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = track / 256;
  msg[6] = track % 256;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::playBlocking(uint16_t track){
  play(track);
  while(!isFinished());
}

void AudioPlayer::setVolume(float volume){
  uint8_t msg[MSG_SIZE];
  uint8_t vol = uint8_t(round(volume*0x1E));
  // Volume between 0x00 et 0x1E
  if(volume > 0x1E){
    volume = 0x1E;
  }
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_SET_VOL;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = vol;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::pause(){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_PAUSE;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = 0x00;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::resume(){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_PLAY;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = 0x00;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::stop(){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_STOP;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = 0x00;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::next(){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_NEXT;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = 0x00;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}

void AudioPlayer::previous(){
  uint8_t msg[MSG_SIZE];
  msg[0] = START_BYTE;
  msg[1] = VERSION_BYTE;
  msg[2] = MSG_SIZE - 2;
  msg[3] = CMD_PREV;
  msg[4] = NO_REPLY_BYTE;
  msg[5] = 0x00;
  msg[6] = 0x00;
  msg[7] = END_BYTE;
  writeMsg(msg, MSG_SIZE);
  delay(20);
}


bool AudioPlayer::isFinished(){
  delay(10);
  unsigned char c[10] = {0};
  uint8_t i = 0;
  while(player->available()){
    c[i] = player->read();
    i++;
    delay(10); // To be sure not to skip any byte
  }
  // print_HEX(Serial, c , i);

  if(c[3]>>4 == 3){
    // Serial.println("Song finished");
    return true;
  }else{
    return false;
  }
}

void AudioPlayer::writeMsg(uint8_t * message, uint8_t size){
  for (uint8_t i =0 ;i < size; i++){
	   player->write((message[i]));
	};
}

void AudioPlayer::print_HEX(HardwareSerial& debug,uint8_t* msg, uint8_t size){
  for(int i = 0; i < size; i++)
  {
    debug.print(msg[i],HEX);
    debug.print(' ');
  }
  if(size > 0){
    debug.println();
  }
}
