/*
@file FSMSingleBlink.ino
@version 2.1
@author Alex Palmese
@contact alex.palmese@gmail.com
@description
"SINGLE BLINK" FINITE STATE MACHINE SAMPLE
This basic Finite State Machine will simply flash the builtin LED for 
a fixed time when the button is pressed.
Connect a button to pin 7, with a pulldown resistor (1k+).

It has only two states: 
* START turns the LED off and when the button is pressed, turns the led ON and switches to LED state
* LED turns the LED on and after the amount of time contained in the parameter, switches back to START 
*/
#include <EasyFSM.h>

// ***********************************************
// FINITE STATE MACHINE
// ***********************************************
EasyFSM fsm;
// States
#define S_START 1
#define S_LED 2
// Conditions
#define C_BUTTON 1
#define C_TIMER 2
// Actions
#define A_LED_OFF 0
#define A_LED_ON 1

// ***********************************************
// SYMBOLS
// ***********************************************
#define LED 13
#define BUTTON 7

// ***********************************************
// GLOBALS
// ***********************************************
long millis0;
bool btnPressed = false;
int prevState = 0;

// ***********************************************
void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);

  // State definitions
  fsm.Write(S_START, C_BUTTON, A_LED_ON, 0, S_LED);
  fsm.Write(S_LED, C_TIMER, A_LED_OFF, 1000, S_START);
  // Set the callback functions
  fsm.SetFunctions(&TestCondition, &DoAction);
  // Set the machine to first state
  fsm.Set(S_START);
}

// ***********************************************
void loop() {
  // State execution
  fsm.Execute();
}

boolean TestCondition(int condition) {
  switch ( condition ) {
  case C_TIMER: // Check if timer ends
    if ( millis() - millis0 > fsm.Param() ) {
      // End of time, reset start time
      millis0 = millis();
      return true;
    }
    return false;
  case C_BUTTON:
    return ( digitalRead(BUTTON) == HIGH );
  case C_ELSE:
    return true;
  }
  return false;
}

void DoAction(int action) {
  switch ( action ) {
  case A_LED_OFF:
    digitalWrite(LED, LOW);
    break;
  case A_LED_ON:
    digitalWrite(LED, HIGH);
    break;
  }
  return;
}
