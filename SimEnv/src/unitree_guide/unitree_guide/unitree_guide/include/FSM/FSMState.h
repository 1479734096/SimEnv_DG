/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

//添加ros节点，读取move_base
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#define  OVERTIME 200000
class FSMState{

public:
struct CmdVel {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
    bool valid = false;
    bool timed_out = false;
    bool clamped = false;
    ros::Time stamp;
    std::string source = "none";
};

    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;
    int real = false;
    long long getRosTime();
    long long getTime();
    void rosAbsoluteWait(long long startTime, long long waitTime);
    void wait(long long startTime, long long waitTime);
    void setVelocityCommand(const CmdVel &cmd);
    void setVelocityCommand(double linear_x,
                            double linear_y,
                            double angular_z,
                            const std::string &source,
                            const ros::Time &stamp = ros::Time::now());
    CmdVel getVelocityCommand();
    void configureVelocityCommandLimits(double max_linear_x,
                                        double max_linear_y,
                                        double max_angular_z,
                                        double timeout_sec);


protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;

    uint32_t overtime = 0;


/*ros节点的私类变量*/
public:
    //cmd_vel callback function
    CmdVel current_cmd_vel_;
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    ros::NodeHandle nh;
    //声明订阅节点与发布节点
    ros::Subscriber Sub_;
    ros::Publisher Pub_;
    std::mutex cmd_vel_mutex_;
    double cmd_vel_max_linear_x_ = 0.6;
    double cmd_vel_max_linear_y_ = 0.4;
    double cmd_vel_max_angular_z_ = 1.0;
    double cmd_vel_timeout_sec_ = 0.5;

};

#endif  // FSMSTATE_H
