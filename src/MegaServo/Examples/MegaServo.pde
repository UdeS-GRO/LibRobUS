#include <MegaServo.h>

// test sketch for MegaServo library
// this will sweep all servos back and forth once, then position according to voltage on potPin


#define FIRST_SERVO_PIN  22  

MegaServo Servos[MAX_SERVOS] ; // max servos is 32 for mega, 8 for other boards

int pos = 0;      // variable to store the servo position 
int potPin = 0;   // connect a pot to this pin.

void setup()
{
  for( int i =0; i < MAX_SERVOS; i++)
    Servos[i].attach( FIRST_SERVO_PIN +i, 800, 2200);

  sweep(0,180,2); // sweep once    
}

void sweep(int min, int max, int step)
{
  for(pos = min; pos < max; pos += step)  // goes from 0 degrees to 180 degrees    
  {                                  // in steps of 1 degree 
    for( int i =0; i < MAX_SERVOS; i++){ 
      Servos[i].write( pos);     // tell servo to go to position  
    }
    delay(15);                  // waits 15ms for the servo to move 
  } 
  for(pos = max; pos>=min; pos-=step)     // goes from 180 degrees to 0 degrees 
  {                                
    for( int i =0; i < MAX_SERVOS; i++){ 
      Servos[i].write( pos);     // tell servo to go to position  
    }
    delay(15);                  // waits 15ms for the servo to move 
  }   
}

void loop()
{ 
  pos = analogRead(potPin);   // read a value from 0 to 1023
  for( int i =0; i < MAX_SERVOS; i++) 
    Servos[i].write( map(pos, 0,1023,0,180));   
  delay(15);   
}
