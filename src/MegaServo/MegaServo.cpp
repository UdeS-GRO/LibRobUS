/*
  MegaServo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 0.1
  Copyright (c) 2009 Michael Margolis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* 
  This library uses 16 bit timers to drive up to 12 servos on each timer.
  It uses interrupts so no explicit refresh activity is required. 
  The usage and method naming is similar to the Arduino servo library http://www.arduino.cc/en/Reference/Servo
  except that it support up to 48 servos on a Mega (12 on a standard Arduino).
  Also pulse widths can be in microseconds or degrees -
  write() treats parameters less than 200 as degrees, larger values are treated as milliseconds.
 
  
  A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently written using the write() method

  Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
  Timers are siezed as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.

  The methods are:

   MegaServo - Class for manipulating servo motors connected to Arduino pins.

   attach(pin )  - Attaches a servo motor to an i/o pin.
   attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400  
 
   write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
   writeMicroseconds() - Sets the servo pulse width in microseconds 
   read()      - Gets the last written servo pulse width as an angle between 0 and 180. 
   readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
   attached()  - Returns true if there is a servo attached. 
   detach()    - Stops an attached servos from pulsing its i/o pin. 
  
   March 31 2009: initial release 
   Jun 10 2009   : release 2 - added writeMicroseconds method, renamed read_us to readMicroseconds. Added 8MHz clock support
                   fixes to read method and ISR handler when less than maximum number of servo instances
   
 */
#include <avr/interrupt.h>
//#include <WProgram.h> 


#include "MegaServo.h"

#define TICKS_PER_uS     (clockCyclesPerMicrosecond() / 8)  // number of timer ticks per microsecond with prescale of 8

#define SERVOS_PER_TIMER   12                               // the maximum number of servos controlled by one timer 
#define TRIM_DURATION     (SERVOS_PER_TIMER/2)             // compensation ticks to trim adjust for digitalWrite delays 

#define NBR_TIMERS        (MAX_SERVOS / SERVOS_PER_TIMER)

static servo_t servos[MAX_SERVOS];                         // static array of servo structures
static volatile int8_t Channel[NBR_TIMERS];                // counter for the servo being pulsed for each timer (or -1 if refresh interval)
#if defined(__AVR_ATmega1280__)
typedef enum {  _timer5, _timer1, _timer3, _timer4 } servoTimer_t; // this is the sequence for timer utilization 
#else
typedef enum { _timer1 } servoTimer_t;                     // this is the sequence for timer utilization 
#endif

//uint8_t siezedTimers = 0;                                // bitmap of initialized timers (replaced by isTimerActive method)
uint8_t ServoCount = 0;	                                   // the total number of attached servos

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((servoTimer_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN(_servo_nbr) (MIN_PULSE_WIDTH - servos[_servo_nbr].min * 4)  // minimum value in uS for this servo
#define SERVO_MAX(_servo_nbr) (MAX_PULSE_WIDTH - servos[_servo_nbr].max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/

static inline void handle_interrupts(servoTimer_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA)
{
  if( Channel[timer] < 0 )
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer 
  else{
    if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && SERVO(timer,Channel[timer]).Pin.isActive == true )  
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,LOW); // pulse this channel low if activated   
  }

  Channel[timer]++;    // increment to the next channel
  if( SERVO_INDEX(timer,Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
    *OCRnA = *TCNTn + SERVO(timer,Channel[timer]).ticks;
    if(SERVO(timer,Channel[timer]).Pin.isActive == true)	   // check if activated
      digitalWrite( SERVO(timer,Channel[timer]).Pin.nbr,HIGH); // its an active channel so pulse it high   
  }	
  else { 
    // finished all channels so wait for the refresh period to expire before starting over 
    if( (unsigned)*TCNTn < (((unsigned int)REFRESH_INTERVAL * TICKS_PER_uS) + 4) )	// allow a few ticks to ensure the next OCR1A not missed
      *OCRnA = (unsigned int)REFRESH_INTERVAL * TICKS_PER_uS;	
    else 
      *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed
	Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel
  }
}

SIGNAL (TIMER1_COMPA_vect) 
{ 
  handle_interrupts(_timer1, &TCNT1, &OCR1A); 
}

#if defined(__AVR_ATmega1280__)
SIGNAL (TIMER3_COMPA_vect) 
{ 
  handle_interrupts(_timer3, &TCNT3, &OCR3A); 
}
SIGNAL (TIMER4_COMPA_vect) 
{
  handle_interrupts(_timer4, &TCNT4, &OCR4A); 
}
SIGNAL (TIMER5_COMPA_vect) 
{
  handle_interrupts(_timer5, &TCNT5, &OCR5A); 
}
#endif

static void initISR(servoTimer_t timer)
{  
  if(timer == _timer1) {
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1 = 0;              // clear the timer count 
    TIFR1 = _BV(OCF1A);     // clear any pending interrupts; 
    TIMSK1 =  _BV(OCIE1A) ; // enable the output compare interrupt	  
  }
#if defined(__AVR_ATmega1280__)
  else if(timer == _timer3) {
    TCCR3A = 0;             // normal counting mode 
    TCCR3B = _BV(CS31);     // set prescaler of 8  
    TCNT3 = 0;              // clear the timer count 
    TIFR3 = _BV(OCF3A);     // clear any pending interrupts; 
    TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt		  
  }
  else if(timer == _timer4) {
    TCCR4A = 0;             // normal counting mode 
    TCCR4B = _BV(CS41);     // set prescaler of 8  
    TCNT4 = 0;              // clear the timer count 
    TIFR4 = _BV(OCF4A);     // clear any pending interrupts; 
    TIMSK4 =  _BV(OCIE4A) ; // enable the output compare interrupt		  
  }
  else if(timer == _timer5) {
    TCCR5A = 0;             // normal counting mode 
    TCCR5B = _BV(CS51);     // set prescaler of 8  
    TCNT5 = 0;              // clear the timer count 
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts; 
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt		  
  }
#endif
} 

static boolean isTimerActive(servoTimer_t timer)
{
  // returns true if any servo is active on this timer
  for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) {
	  if(SERVO(timer,channel).Pin.isActive == true)
		 return true;
  }
  return false;
}

//#define DEBUG_SHOW
#ifdef DEBUG_SHOW
#include <HardwareSerial.h> 
void debugShow(int index, char *label)
{
  Serial.print(label);
  Serial.print(" for servo: ");
  Serial.print(index,DEC);
  Serial.print(", pin= ");
  Serial.print(servos[index].Pin.nbr,DEC);
  Serial.print(", ticks=");
  Serial.print(servos[index].ticks,DEC);
  Serial.print(", min=");
  Serial.print(SERVO_MIN(index),DEC);
  Serial.print(", max=");
  Serial.println(SERVO_MAX(index),DEC);
}
#endif


/****************** end of static functions ******************************/

MegaServo::MegaServo()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	servos[this->servoIndex].ticks = DEFAULT_PULSE_WIDTH * TICKS_PER_uS;   // store default values	
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos 
}

uint8_t MegaServo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t MegaServo::attach(int pin, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;  
	// todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128 
	servos[this->servoIndex].min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    servos[this->servoIndex].max  = (MAX_PULSE_WIDTH - max)/4; 
	// initialize the timer if it has not already been initialized 
	servoTimer_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);    
	servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  } 
  return this->servoIndex ;
}

void MegaServo::detach()  
{
  servos[this->servoIndex].Pin.isActive = false;  

#ifdef FREE_TIMERS
  if(isTimerActive(SERVO_INDEX_TO_TIMER(servoIndex)) == false) {
     ;// call to unimplimented function in wiring.c to re-init timer (set timer back to PWM mode) TODO? 
  }
#endif
}

void MegaServo::write(int value)
{	
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  if( (channel >= 0) && (channel < MAX_SERVOS) )   // ensure channel is valid
  {  
    if(value < 200) // convert degrees if value < 200
       value = map(value, 0, 180, SERVO_MIN(channel),  SERVO_MAX(channel));      
    else {
      if( value < SERVO_MIN(channel) )		       // ensure pulse width is valid
        value = SERVO_MIN(channel);
      else if( value > SERVO_MAX(channel) )
        value = SERVO_MAX(channel);	 
    }
    value = (value-TRIM_DURATION) * TICKS_PER_uS;  // convert to ticks after compensating for interrupt overhead
	uint8_t oldSREG = SREG;
	cli();
	servos[channel].ticks = value;  
	SREG = oldSREG; 
  } 
}

void MegaServo::writeMicroseconds(int value)
{
   this->write(value);
}

int MegaServo::read() // return the value as degrees
{
  return  map( this->readMicroseconds(), SERVO_MIN(this->servoIndex), SERVO_MAX(this->servoIndex), 0, 180);     
}

int MegaServo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = (servos[this->servoIndex].ticks /  TICKS_PER_uS) + TRIM_DURATION +1 ;
  else 
    pulsewidth  = 0;

  return pulsewidth;   
}

bool MegaServo::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}