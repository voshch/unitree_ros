/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}
    virtual FSMStateName checkChange(FSMStateName targetState) {return FSMStateName::INVALID;}
    FSMStateName checkChangeOverride(FSMStateName targetState) {

        if (targetState == FSMStateName::INVALID) 
            return checkChange();
        #ifdef COMPILE_WITH_MOVE_BASE
        else if (_stateName == FSMStateName::PASSIVE || _stateName == FSMStateName::FIXEDSTAND || 
                _stateName == FSMStateName::TROTTING ||_stateName == FSMStateName::MOVE_BASE) {
            return checkChange(targetState);
        }
        #endif  // COMPILE_WITH_MOVE_BASE    

        // if(_stateName == targetState)
        //     return targetState;

        // if(_stateName == FSMStateName::FIXEDSTAND && isReached())
        //     return targetState;
        
        return FSMStateName::FIXEDSTAND;
    }
    
    FSMStateName _stateName;
    std::string _stateNameString;
protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;
    int duration = 800;
    virtual bool isReached() {return false;}
};

#endif  // FSMSTATE_H