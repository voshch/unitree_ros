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
    if(_lowState->userCmd == UserCommand::L2_B){
        timeStepF2M = 0;
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        timeStepF2M = 0;
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        timeStepF2M = 0;
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        timeStepF2M = 0;
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        timeStepF2M = 0;
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        timeStepF2M = 0;
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return checkTime4Change();
        // return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else{
        timeStepF2M = 0;
        return FSMStateName::FIXEDSTAND;
    }
}

FSMStateName State_FixedStand::checkChange(FSMStateName targetState) {
    if(targetState == FSMStateName::PASSIVE) {
        timeStepF2M = 0;
        return FSMStateName::PASSIVE;
    }
    else if(targetState == FSMStateName::FREESTAND){
        timeStepF2M = 0;
        return FSMStateName::FREESTAND;
    }
    else if(targetState == FSMStateName::TROTTING){
        timeStepF2M = 0;
        return FSMStateName::TROTTING;
    }
    else if(targetState == FSMStateName::BALANCETEST){
        timeStepF2M = 0;
        return FSMStateName::BALANCETEST;
    }
    else if(targetState == FSMStateName::SWINGTEST){
        timeStepF2M = 0;
        return FSMStateName::SWINGTEST;
    }
    else if(targetState == FSMStateName::STEPTEST){
        timeStepF2M = 0;
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(targetState == FSMStateName::MOVE_BASE){
        return checkTime4Change();
        // return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else{
        timeStepF2M = 0;
        return FSMStateName::FIXEDSTAND;
    }
}


FSMStateName State_FixedStand::checkTime4Change() {
    if (timeStepF2M > duration) {
        timeStepF2M = 0;
        return FSMStateName::MOVE_BASE;
    }
    else {
        timeStepF2M ++;
        return FSMStateName::FIXEDSTAND;
    }
}