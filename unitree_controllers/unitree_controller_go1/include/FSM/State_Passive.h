/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    FSMStateName checkChange(FSMStateName targetState);
    FSMStateName getNextState(FSMStateName nextState);
    float timePassed = 0.0;
    float waitingTime = 0.8; // s
};

#endif  // PASSIVE_H