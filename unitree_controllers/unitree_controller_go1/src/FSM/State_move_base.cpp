/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"

#ifdef COMPILE_WITH_ROS
    #include "ros/ros.h"
#endif

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";

    if(ctrlComp->params){
        robotNamespace = ctrlComp->params->robotNamespace;
    }

    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
    ros::spinOnce();
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe(robotNamespace + "/cmd_vel", 1, &State_move_base::twistCallback, this);
    ROS_INFO("subscribing %s/cmd_vel", robotNamespace.c_str());
}

#endif  // COMPILE_WITH_MOVE_BASE