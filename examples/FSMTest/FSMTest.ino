/*
@file FSMTest.ino
@version 2.1
@author Alex Palmese
@contact alex.palmese@gmail.com
@description
"TEST" FINITE STATE MACHINE SAMPLE
Connect two LEDs (with required 220 Ohm resistors) to pins 12 and 13, and a button to 
pin 7 (with pull-down 1k+ resistor).

This Finite State Machine will let Arduino act as follows:
1) At startup, all LEDs are off
2) When button is pressed, LED 1 starts blinking with 200ms cycle
3) When button is pressed again, LED 1 is steady
4) When button is pressed again, LED 1 is turned off, and LED 2 is on
5) If button is pressed within 5 seconds, both LEDs will be on, and the machine 
goes to a complete stop, no further actions possible.
6) If no button is pressed, afer 5 seconds the second LED will turn off and start
again at the beginning
*/
#include <EasyFSM.h>
#include <SimpleTimer.h>

// ***********************************************
// FINITE STATE MACHINE
// ***********************************************
// States
#define S_START 1
#define S_FLASH 2
#define S_STEADY 3
#define S_LED2 4
// Conditions
#define C_BUTTON 0
#define C_LED2_OFF 1
// Actions
#define A_LED_OFF 0
#define A_LED_ON 1
#define A_FLASH 2
#define A_LED2_ON 3
#define A_ALL_LED_ON 4

// ***********************************************
// SYMBOLS
// ***********************************************
// Pins
#define LED 13
#define LED2 12
#define BUTTON 7

// ***********************************************
// VARIABLES/GLOBALS
// ***********************************************
EasyFSM fsm;
// the timer object
SimpleTimer timer;
int blink = 0;
int led = LOW;
int tmrBlink = -1;
int tmrSingle = -1;

// ***********************************************
void setup() {  
  Serial.begin(115200);

  // State START, Event BUTTON, Action FLASH, Parameter 200, Next State FLASH
  fsm.Write(S_START, C_BUTTON, A_FLASH, 200, S_FLASH);
  // State FLASH, Event BUTTON, Action LED_ON, Parameter 0, Next State STEADY
  fsm.Write(S_FLASH, C_BUTTON, A_LED_ON, 0, S_STEADY);
  // State STEADY, Event BUTTON, Action LED2_ON, Parameter 5000, Next State LED2
  fsm.Write(S_STEADY, C_BUTTON, A_LED2_ON, 5000, S_LED2);
  // State LED2, Event LED2_OFF, Action LED_OFF, Parameter 0, Next State START
  fsm.Write(S_LED2, C_LED2_OFF, A_LED_OFF, 0, S_START); 
  // State LED2, Event BUTTON, Action LED_OFF, Parameter 0, Next State HALT
  fsm.Write(S_LED2, C_BUTTON, A_ALL_LED_ON, 0, S_HALT); 

  // Set the callback functions
  fsm.SetFunctions(&TestCondition, &DoAction);

  // Set the machine to first state
  fsm.Set(S_START);
  Serial.print("First state=");
  Serial.println(fsm.State);
  
  pinMode(LED, OUTPUT);  
  pinMode(LED2, OUTPUT);  
  pinMode(BUTTON, INPUT);
  
}

// ***********************************************
void loop() {
  // State execution
  fsm.Execute();
  // Timers update
  DoWork();
}

boolean TestCondition(int condition) {
  static int val0 = 0;
  int btn;
  
  switch ( condition ) {
  case C_BUTTON:
    btn = digitalRead(BUTTON);
    if ((btn == HIGH) && (val0 == LOW)) {
      val0 = btn;
      // Just a bit of basic debouncing...
      delay(100);
      while (digitalRead(BUTTON) == HIGH)
      delay(100);
      Serial.println("BUTTON");
      return true;
    }
    val0 = btn;
    return false;
  case C_LED2_OFF:
      return (digitalRead(LED2) == LOW);
  }
  return false;
}

void DoAction(int action) {
  switch ( action ) {
    case A_LED_OFF:
      blink = 0;
      digitalWrite(LED, LOW);
      return;
    case A_LED_ON:
      if ( tmrBlink != -1 ) {
        timer.deleteTimer(tmrBlink);
        tmrBlink = -1;
      }
      digitalWrite(LED, HIGH);
      digitalWrite(LED2, LOW);
      return;
    case A_FLASH:
      blink = fsm.Param();
      digitalWrite(LED, HIGH);
      digitalWrite(LED2, LOW);
      if ( tmrSingle != -1 ) {
        timer.deleteTimer(tmrSingle);
        tmrSingle = -1;
      }
      tmrBlink = timer.setInterval(blink, BlinkLed);
      return;
    case A_LED2_ON:
      blink = fsm.Param();
      digitalWrite(LED, LOW);
      digitalWrite(LED2, HIGH);
      if ( tmrBlink != -1 ) {
        timer.deleteTimer(tmrBlink);
        tmrBlink = -1;
      }
      tmrSingle = timer.setTimeout(blink, Led2Off);
      return;
    case A_ALL_LED_ON:
      digitalWrite(LED, HIGH);
      digitalWrite(LED2, HIGH);
      timer.disable(tmrBlink);
      timer.disable(tmrSingle);
  }
  return;
}

void DoWork() {
  timer.run();
}

void BlinkLed() {
  digitalWrite(LED, !digitalRead(LED));
}

void Led2Off() {
  digitalWrite(LED2, LOW);
}
