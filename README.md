# EasyFSM
Easy FiniteState machine for Arduino
## Definition
This library lets you easily implement Finite State Machines (FSM) over Arduino boards.
## Installation
1) Download the code as a ZIP file from the "Clone or download" green button
2) Extract the zip file
3) Move the "EasyFSM-master" folder that has been extracted to your libraries directory
4) Rename the folder to "EasyFSM"
## Usage
For a full functions reference please refer to [Library reference Wiki](https://github.com/dotto59/EasyFSM/wiki/Library-Reference)

First, always draw your **state diagram** using circles for the states, and arcs representing the "transitions". Each transition is a combination of three items: a "**condition**" (any event or test result) an "**action**" (what to do when the event happens) and the "**next state**" (the new state after the action is done). One of the states will be the "starting" state. Don't start working without a complete and logically tested state diagram, as a good chart makes the implementation pretty straightforward instead of trial-and-error method.

You now need just a few coding! First of all, you must define all the symbols for States, Conditions, and Actions. I suggest you to start names with "S_" for States, Conditions with "C_" and Actions with "A_", and assign then some integer values (a sequence starting from 1).

For example, let's assume you want to create a simple machine with just two states to blink a LED, like in FSMBlink example project. The starting state is called START, then the condition will be a timer fired (e.g. 1 second) and the actions are turn the LED on or off:
```
// State
#define S_START 1
#define S_LED 2
// Conditions
#define C_TIMER 0
// Actions
#define A_LED_OFF 0
#define A_LED_ON 1
```
After creating a global variable for the finite state machine, "``FiniteState fsm;``", our machine will be easily defined on setup() function as follows, together with the "initial state" definition:
```
  // State definitions
  fsm.Write(S_START, C_TIMER, A_LED_ON, 1000, S_LED);
  fsm.Write(S_LED, ELSE, A_LED_OFF, 1000, S_START);
  // Set the callback functions
  fsm.SetFunctions(&TestCondition, &DoAction);
  // Set the machine to first state
  fsm.Set(S_START);
```
With the first statement we define a transition from START state to LED state, based on a TIMER of 1000 ms. Obviously, the second represents the opposite transition, from state LED back to START after another second. The third one is the reference to local functions we'll see next. The last statement sets the starting state to START.
As you can see, Write() has 5 parameters:
1) Current state code
2) Condition required to change state
3) Action required if Condition is met
4) Optional integer parameter (if not used, simply put a zero)
5) Next state at the end of action

After this initialization, you need to create two functions: one to check the conditions or "TestCondition()", and one to perform the required actions, called "DoAction". A reference to those functions is passed to FiniteState object by calling the "SetFunctions()" method we saw before.
The function "TestConditions()" is called back by FiniteState library passing the condition code (one of the C_* symbols) and must  return a boolean value representing the test result: true if the condition is met (so the corresponding action will be performed), or false if not. You just need to implement a "switch..case" with all the condition symbols you have defined before.
```
boolean TestCondition(int condition) {
  switch ( condition ) {
  case C_TIMER: // Check if timer ends
    if ( millis() - millis0 > fsm.Param() ) {
      // End of time, reset start time
      millis0 = millis();
      return true;
    }
    return false;
  case C_ELSE:
    return true;
  }
  return false;
}
```
In the function "DoAction" you just need to check the required action,passed by the library as its parameter, and then use it in another "switch..case" block with all the action symbols defined and what each action should perform.
In our example, the code will be as follows:
```
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
```
Finally, your loop() cycle will be pretty simple and identical for almost any project using FSM:
```
void loop() {
  // State execution
  fsm.Execute();
}
```
The "Execute" method automatically cycles every condition of the current state, then if matches, calls DoAction function to run the corresponding action.

## Contributing
If you want to contribute to this project:
* Report bugs and errors
* Ask for enhancements
* Create issues and pull requests
* Tell other people about this library
## Copyright
(c) 2020 Alex Palmese
## License
This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation.
This library is distributed WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details. 
