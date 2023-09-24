/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/ControlFrame.h"

ControlFrame::ControlFrame(CtrlComponents *ctrlComp, int FSM_targetState):_ctrlComp(ctrlComp){
    _FSMController = new FSM(_ctrlComp);
    _FSMController->initialize((FSMStateName) FSM_targetState);
}

void ControlFrame::run(){
    _FSMController->run();
}