/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSMState.h"
#include <algorithm>

FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
}

long long FSMState::getRosTime()
{
    return _ctrlComp->ioInter->current_time;
}

long long  FSMState::getTime()
{
    if (real == false)
    {
        return getRosTime();
    }
    else
    {
        return getSystemTime();
    }
}

void FSMState::wait(long long startTime, long long waitTime){
    if (real == false)
    {
        rosAbsoluteWait(startTime,waitTime);
    }
    else
    {
        absoluteWait(startTime,waitTime);
    }
}

void FSMState::rosAbsoluteWait(long long startTime, long long waitTime){
    overtime = 0;
    if(getRosTime() - startTime > waitTime){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while((getRosTime() - startTime < waitTime) && (overtime < OVERTIME)){

        // std::cout << "getRosTime()" << getRosTime() << std::endl;
        // std::cout << "startTime" << startTime << std::endl;
        // std::cout << "wait" << std::endl;
        // std::cout << "overtime" <<  overtime << std::endl;
        usleep(50);
        overtime += 50;
    }

}

void FSMState::setVelocityCommand(const CmdVel &cmd){
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_cmd_vel_ = cmd;
}

void FSMState::setVelocityCommand(double linear_x,
                                  double linear_y,
                                  double angular_z,
                                  const std::string &source,
                                  const ros::Time &stamp){
    CmdVel cmd;
    cmd.linear_x = linear_x;
    cmd.linear_y = linear_y;
    cmd.angular_z = angular_z;
    cmd.valid = true;
    cmd.stamp = stamp;
    cmd.source = source;
    setVelocityCommand(cmd);
}

FSMState::CmdVel FSMState::getVelocityCommand(){
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    CmdVel cmd = current_cmd_vel_;
    if(!cmd.valid){
        return cmd;
    }

    const double raw_linear_x = cmd.linear_x;
    const double raw_linear_y = cmd.linear_y;
    const double raw_angular_z = cmd.angular_z;
    cmd.linear_x = std::max(-cmd_vel_max_linear_x_, std::min(cmd_vel_max_linear_x_, cmd.linear_x));
    cmd.linear_y = std::max(-cmd_vel_max_linear_y_, std::min(cmd_vel_max_linear_y_, cmd.linear_y));
    cmd.angular_z = std::max(-cmd_vel_max_angular_z_, std::min(cmd_vel_max_angular_z_, cmd.angular_z));
    cmd.clamped = (cmd.linear_x != raw_linear_x) ||
                  (cmd.linear_y != raw_linear_y) ||
                  (cmd.angular_z != raw_angular_z);

    if(cmd_vel_timeout_sec_ > 0.0 && !cmd.stamp.isZero()){
        const ros::Duration age = ros::Time::now() - cmd.stamp;
        if(age.toSec() > cmd_vel_timeout_sec_){
            cmd.linear_x = 0.0;
            cmd.linear_y = 0.0;
            cmd.angular_z = 0.0;
            cmd.timed_out = true;
        }
    }
    return cmd;
}

void FSMState::configureVelocityCommandLimits(double max_linear_x,
                                              double max_linear_y,
                                              double max_angular_z,
                                              double timeout_sec){
    cmd_vel_max_linear_x_ = std::max(0.0, max_linear_x);
    cmd_vel_max_linear_y_ = std::max(0.0, max_linear_y);
    cmd_vel_max_angular_z_ = std::max(0.0, max_angular_z);
    cmd_vel_timeout_sec_ = std::max(0.0, timeout_sec);
}

// 设置 /cmd_vel 回调，并统一写入速度命令缓存，便于后续接入其他命令源。
void FSMState::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
   if (msg) {
        setVelocityCommand(msg->linear.x,
                           msg->linear.y,
                           msg->angular.z,
                           "cmd_vel",
                           ros::Time::now());
    }
}

