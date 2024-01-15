/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();
}

void State_FixedStand::run(){
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange() {
    FSMStateName userState = getStateFromUser();
    return getNextState(userState);
}

FSMStateName State_FixedStand::checkChange(FSMStateName targetState) {
    return getNextState(targetState);
}

FSMStateName State_FixedStand::getNextState(FSMStateName nextState) {
    switch (nextState) {
        case FSMStateName::PASSIVE:
            return FSMStateName::PASSIVE;
        case FSMStateName::FREESTAND:
            return FSMStateName::FREESTAND;
        case FSMStateName::TROTTING:
            return FSMStateName::TROTTING;
        case FSMStateName::BALANCETEST:
            return FSMStateName::BALANCETEST;
        case FSMStateName::SWINGTEST:
            return FSMStateName::SWINGTEST;
        case FSMStateName::STEPTEST:
            return FSMStateName::STEPTEST;
        #ifdef COMPILE_WITH_MOVE_BASE
        case FSMStateName::MOVE_BASE:
            if (isReached())
                return FSMStateName::MOVE_BASE;
            else
                return FSMStateName::FIXEDSTAND;
        #endif  // COMPILE_WITH_MOVE_BASE
        default:
            return FSMStateName::FIXEDSTAND;
    }
}

