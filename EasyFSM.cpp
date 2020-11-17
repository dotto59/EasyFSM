/*
@file EasyFSM.cpp
@version 2.1
@author Alex Palmese
@contact alex.palmese@gmail.com
@description
| Finite state machine implementation (for robot programming)
@license
| This library is free software; you can redistribute it and/or
| modify it under the terms of the GNU Lesser General Public
| License as published by the Free Software Foundation.
| This library is distributed WITHOUT ANY WARRANTY; without even
| the implied warranty of MERCHANTABILITY or FITNESS FOR A
| PARTICULAR PURPOSE.  See the GNU Lesser General Public License
| for more details.
*/
#include "Arduino.h"
#include "EasyFSM.h"

int _State;

// ------------------------------------------------
// METHODS
// ------------------------------------------------
EasyFSM::EasyFSM() {
  Reset();
}

void EasyFSM::Reset() {
  id = -1;
  // Create first "null" record
  Write(S_HALT,C_ELSE,A_NONE,0,S_HALT);
  id = -1;
  Max = -1;
  // Set current state to HALT
  Set(S_HALT);
}

void EasyFSM::SetFunctions(bool (*condCheck)(int), void (*doAction)(int)) {
  _CondCheck = condCheck;
  _DoAction = doAction;
}

int State() {
  return _State;
}
void EasyFSM::Write(int idState, int idCond, int idAction, int idParam, int idNext) {
    ++id;
    _CurState[id] = idState;
    _Condition[id] = idCond;
    _Action[id] = idAction;
    _Param[id] = idParam;
    _NextState[id] = idNext;
    if (id > Max) Max = id;
}

// Search next current state condition. 
// Returns C_BREAK if no more conditions are found
int EasyFSM::Next() {
  for (int i=++id; i<=Max; ++i)
    if ( _CurState[i] == _State ) {
      id = i;
      return _Condition[i];
    }
  id = -1;
  return C_BREAK;
}

// Sets current state
void EasyFSM::Set(int newState) {
  _State = S_HALT;
  for(id=0; id<=Max; ++id)
    if ( _CurState[id] == newState ) {
      _State = newState;
    }
  id = -1;
}

// Sets next state
void EasyFSM::SetNext() {
  Set(NextState());
}

// Returns current state condition
int EasyFSM::Condition() {
  if ( id > -1 )
      return _Condition[id];
  else
      return C_BREAK;
}

// Returns current state action
int EasyFSM::Action() {
  if ( id > -1 )
      return _Action[id];
  else
      return A_NONE;
}

// Returns current state interger parameter
int EasyFSM::Param() {
  if ( id > -1 )
      return _Param[id];
  else
      return 0;
}

// Returns the next state for current condition
int EasyFSM::NextState() {
  if ( id > -1 )
      return _NextState[id];
  else
      return S_HALT;
}

void EasyFSM::Execute() {
  // State execution cycle
  while ( _State != S_HALT && Next() != C_BREAK )
    if ( _CondCheck(Condition()) ) {
      // Execute required action
      _DoAction(Action());
      // Jump to next state
      SetNext();
      break;
    }
}
