/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"


class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    virtual FSMStateName checkChange(FSMStateName targetState);
    virtual FSMStateName getNextState(FSMStateName nextState);
protected:
    virtual bool isReached(){
        return _percent == 1.f;
    }

private:
    float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    float _startPos[12];
    float _duration = 1000;   //steps
    float _percent = 0;       // [0,1]

};

#endif  // FIXEDSTAND_H