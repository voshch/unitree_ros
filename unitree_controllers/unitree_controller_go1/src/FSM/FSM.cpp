/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>
#include "FSM.h"

Vec3 eulerAngles(RotMat rotMat);

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    if(ctrlComp->params){
        _targetState = (FSMStateName) ctrlComp->params->targetState;
    }

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
    _stateList.danger = new State_Danger(_ctrlComp);
    

    #ifdef COMPILE_WITH_MOVE_BASE
        _stateList.moveBase = new State_move_base(_ctrlComp);
    #endif  // COMPILE_WITH_MOVE_BASE

    initialize();
    
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    _resetTask.data = false;
    // _resetTaskPub = _nh.advertise<std_msgs::Bool>("reset_task", 10);
    _resetTaskSub = _nh.subscribe("reset_task", 10, &FSM::reachedTargetCallback, this);
    
    _dangerMode.data = false;
    _dangerModePub = _nh.advertise<std_msgs::Bool>("danger_mode", 10);
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();

    if(!checkSafety()){
        _nextState = _stateList.danger;
        _mode = FSMMode::CHANGE;
        if (!_dangerMode.data) {
            _ctrlComp->estimator->callInitSystem();
            _dangerMode.data = true;
            _dangerModePub.publish(_dangerMode);
        }
    }
    else {
        if (_dangerMode.data) {
            _dangerMode.data = false;
            _dangerModePub.publish(_dangerMode);
        }
        if (_resetTask.data) {
            if (initSys)
                _ctrlComp->estimator->callInitSystem();
            _nextStateName =_currentState->checkChangeOverride(FSMStateName::PASSIVE);
            if (_nextStateName != _currentState->_stateName){
                _nextState = getNextState(_nextStateName);
            }
            _mode = FSMMode::CHANGE;
            initSys = false;
        }
        else
            initSys = true;

    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChangeOverride(_targetState);
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if(_mode == FSMMode::CHANGE){
        if(_nextState->_stateName != _currentState->_stateName){
            std::cout << "Switched from " << _currentState->_stateNameString << " to " << _nextState->_stateNameString << std::endl;
            _currentState->exit();
        }
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
    case FSMStateName::DANGER:
        return _stateList.danger;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafety(){
    currOrientation = eulerAngles(_ctrlComp->lowState->getRotMat())*180/M_PI;
    // std::cout << "Orientation:\n " << orientation << std::endl;
    // Check if angle to z-axis is greater than 60 degrees (in absolute value)
    if(fabs(currOrientation(0)) >= 60 || fabs(currOrientation(0)) >= 60){
        _safetyTimeout = getSystemTime() + 1e6; // 1s  
        std::cout << "DANGER!\n";
        return false;
    }
    //keep waiting
    else if(getSystemTime() < _safetyTimeout){
        // std::cout << "latent danger, remaining " << (_safetyTimeout - getSystemTime())/1e6 << "s" << std::endl;
        return false;
    }
    else{
        return true;
    }
}

void FSM::reachedTargetCallback(const std_msgs::Bool::ConstPtr& msg) {
    _resetTask.data = msg->data;
    if (_resetTask.data) {
        std::cout << "Target reached, resetting...\n";
    }
}

Vec3 eulerAngles(RotMat rotMat) {
    // float theta1, theta2, psi1, psi2, phi1, phi2, theta, psi, phi;
    float psi, theta, phi;
    Vec3 angles;
    if (fabs(rotMat(2,0)) != 1) {
        // theta1 = -std::asin(rotMat(2,0));
        // theta2 = M_PI - theta1;
        // psi1 = std::atan2(rotMat(2,1)/std::cos(theta1), rotMat(2,2)/std::cos(theta1));
        // psi2 = std::atan2(rotMat(2,1)/std::cos(theta2), rotMat(2,2)/std::cos(theta2));
        // phi1 = std::atan2(rotMat(1,0)/std::cos(theta1), rotMat(0,0)/std::cos(theta1));
        // phi2 = std::atan2(rotMat(1,0)/std::cos(theta2), rotMat(0,0)/std::cos(theta2));
        // angles << psi1, theta1, phi1;

        theta = -std::asin(rotMat(2,0));
        psi = std::atan2(rotMat(2,1)/std::cos(theta), rotMat(2,2)/std::cos(theta));
        phi = std::atan2(rotMat(1,0)/std::cos(theta), rotMat(0,0)/std::cos(theta));

    }
    else {
        phi = 0;
        if (rotMat(2,0) ==  -1) {
            theta = M_PI/2;
            psi = phi + std::atan2(rotMat(0,1),rotMat(0,2));
        }
        else {
            theta = -M_PI/2;
            psi = -phi + std::atan2(-rotMat(0,1),-rotMat(0,2));
        }
        // angles << psi, theta, phi;
    }
    angles << psi, theta, phi;
    return angles;
}