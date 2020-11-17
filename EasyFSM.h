/*
@file EasyFSM.h
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
#ifndef FiniteState_h
#define FiniteState_h

#include "Arduino.h"

#define MAX_COND 32    // Max conditions

// Null state (start), do nothing
#define S_HALT -1
// Default condition (ELSE)
#define C_ELSE -1
// Null condition returned from Next to signal end of conditions
#define C_BREAK -2
// No actions
#define A_NONE -1

class EasyFSM {
public:
  EasyFSM();
  void Reset();
  void Write(int idState, int idCond, int idAction, int idParam, int idNext);
  void SetFunctions(bool (*condCheck)(int), void (*doAction)(int));
  int State();
  int Next();
  void Set(int newState);
  void SetNext();
  int Condition();
  int Action();
  int Param();
  int NextState();
  void Execute();
  
private:
  int _State;
  int _CurState[MAX_COND];
  int _Condition[MAX_COND];
  int _Action[MAX_COND];
  int _Param[MAX_COND];
  int _NextState[MAX_COND];
  int id;    
  int Max;
  bool (*_CondCheck)(int);
  void (*_DoAction)(int);
};

#endif
