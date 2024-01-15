/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 8;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 3;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }

    _ctrlComp->setAllSwing();
}

void State_Passive::run(){
    
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    FSMStateName userState = getStateFromUser();
    return getNextState(userState);
}

FSMStateName State_Passive::checkChange(FSMStateName targetState){
    // return getNextState(targetState); 
    if(targetState == FSMStateName::FIXEDSTAND){
        return FSMStateName::FIXEDSTAND;
    }
    #ifdef COMPILE_WITH_MOVE_BASE
    else if (targetState == FSMStateName::MOVE_BASE) {
        if (timePassed >= waitingTime) {
            timePassed = 0.0;
            return FSMStateName::FIXEDSTAND;
        }
        else
            timePassed += _ctrlComp->dt;
            return FSMStateName::PASSIVE;
    }
    #endif
    else 
        return FSMStateName::PASSIVE;
}

FSMStateName State_Passive::getNextState(FSMStateName nextState) {
    if(nextState == FSMStateName::FIXEDSTAND){
        return FSMStateName::FIXEDSTAND;
    }
    #ifdef COMPILE_WITH_MOVE_BASE
    else if (nextState == FSMStateName::MOVE_BASE) {
            return FSMStateName::FIXEDSTAND;
    }
    #endif
    else 
        return FSMStateName::PASSIVE;
}