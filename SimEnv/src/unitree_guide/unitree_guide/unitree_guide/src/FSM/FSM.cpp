/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

#ifdef COMPILE_WITH_ROS
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#endif

namespace {
const std::array<std::string, 12> kResetJointNames = {{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
}};

const std::array<double, 12> kResetJointPositions = {{
    -0.35, 1.36, -2.65,
     0.35, 1.36, -2.65,
     -0.50, 1.36, -2.65,
      0.50, 1.36, -2.65
}};

// a1.urdf calf 关节限位
const double kCalfUrdfLower = -2.69653369433;
const double kCalfUrdfUpper = -0.916297857297;
// 经启动链与 reset 实测验证的安全配置区间（-2.0 附近）
const double kCalfSafeLower = -2.4;
const double kCalfSafeUpper = -1.8;
// reset 输入重新武装所需的去抖帧数：250Hz 下约 0.5s，
// 必须覆盖 joy/键盘输入帧间隔，否则按住的输入会反复触发 reset
const int kResetRearmFrames = 125;

double getEnvDouble(const char *name, double fallback)
{
    const char *value = std::getenv(name);
    if(value == nullptr || value[0] == '\0'){
        return fallback;
    }

    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value, &end);
    if(errno != 0 || end == value || *end != '\0' || !std::isfinite(parsed)){
        return fallback;
    }
    return parsed;
}

// 恢复物理前写入模型的 calf 配置。calf 与启动链 STARTUP_CALF_POSITION 同源：
// 直接写成 -2.65 再 unpause 有重新触发关节角展开的风险（ENV-004），
// 启动链已验证“配置 -2.0 + 下趴命令 -2.65”的过渡可稳定收敛。
// 外部配置必须同时通过 URDF 限位和上述安全区间校验，否则回退 -2.0。
double getResetConfigCalfPosition()
{
    const double fallback = -2.0;
    const double calf = getEnvDouble("COMPETITION_RESET_CALF_POSITION", fallback);
    if(calf < kCalfUrdfLower || calf > kCalfUrdfUpper){
        std::cout << "[WARNING] COMPETITION_RESET_CALF_POSITION=" << calf
                  << " exceeds the URDF calf limit [" << kCalfUrdfLower << ", "
                  << kCalfUrdfUpper << "]. Falling back to " << fallback << " rad." << std::endl;
        return fallback;
    }
    if(calf < kCalfSafeLower || calf > kCalfSafeUpper){
        std::cout << "[WARNING] COMPETITION_RESET_CALF_POSITION=" << calf
                  << " is outside the validated safe range [" << kCalfSafeLower << ", "
                  << kCalfSafeUpper << "]. Falling back to " << fallback << " rad." << std::endl;
        return fallback;
    }
    return calf;
}

std::array<double, 12> getResetConfigJointPositions()
{
    std::array<double, 12> positions = kResetJointPositions;
    const double calf = getResetConfigCalfPosition();
    for(size_t i = 2; i < positions.size(); i += 3){
        positions[i] = calf;
    }
    return positions;
}

#ifdef COMPILE_WITH_ROS
bool callEmptyService(const std::string &service_name, double timeout_seconds)
{
    if(!ros::service::waitForService(service_name, ros::Duration(timeout_seconds))){
        std::cout << "[WARNING] " << service_name << " is not available." << std::endl;
        return false;
    }

    std_srvs::Empty srv;
    if(!ros::service::call(service_name, srv)){
        std::cout << "[WARNING] Failed to call " << service_name << "." << std::endl;
        return false;
    }
    return true;
}
#endif
}

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    _stateList.rl = new State_RL(_ctrlComp);
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->ioInterFreeDog->sendRecv();
    if(handleResetCommand()){
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    if(!_ctrlComp->ioInter->hasFullStateFeedback()){
        if(!_waitingForStateFeedback){
            std::cout << "[INFO] Waiting for Gazebo joint state feedback before accepting stand command." << std::endl;
            _waitingForStateFeedback = true;
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    if(_waitingForStateFeedback){
        std::cout << "[INFO] Gazebo joint state feedback is ready." << std::endl;
        _waitingForStateFeedback = false;
    }
    if(!_ctrlComp->lowState->isFinite()){
        std::cout << "[WARNING] Gazebo state feedback is not finite; skipping control update." << std::endl;
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        if(!_fallSafetyLatched){
            std::cout << "[WARNING] Robot appears to have fallen. Switching to passive/down. Press 8 to reset pose." << std::endl;
            _fallSafetyLatched = true;
        }
        forcePassiveState();
        _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    _fallSafetyLatched = false;

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    applyResetTransition();

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
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    case FSMStateName::RL:
        return _stateList.rl;
    break;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}

bool FSM::handleResetCommand(){
    if(_ctrlComp->lowState->userCmd != UserCommand::RESET){
        if(_resetCommandLatched){
            // 释放去抖：forcePassiveState 会把面板命令改成 L2_B，而持续按住的
            // joy/键盘输入随后又会写回 RESET；必须连续足够帧观察到非 RESET
            // 才重新武装，保证“按住只触发一次，释放再按才触发第二次”。
            _resetReleaseFrames += 1;
            if(_resetReleaseFrames >= kResetRearmFrames){
                _resetCommandLatched = false;
                _resetReleaseFrames = 0;
            }
        }
        return false;
    }
    _resetReleaseFrames = 0;
    if(_resetCommandLatched){
        return true;
    }
    _resetCommandLatched = true;

    std::cout << "[INFO] Reset command received. Returning robot to the start pose." << std::endl;
    forcePassiveState();

    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        if(resetGazeboRobot()){
            std::cout << "[INFO] Gazebo robot reset complete. Press 2 to stand again." << std::endl;
        }else{
            std::cout << "[WARNING] Gazebo robot reset request failed. Physics stays paused; "
                      << "check Gazebo services, then unpause manually or restart the round."
                      << std::endl;
        }
    }else{
        std::cout << "[WARNING] Reset command is only supported in Gazebo simulation." << std::endl;
    }

    return true;
}

bool FSM::resetGazeboRobot(){
#ifndef COMPILE_WITH_ROS
    return false;
#else
    ros::NodeHandle nh;

    if(!callEmptyService("/gazebo/pause_physics", 1.0)){
        return false;
    }

    setResetDownCommand();
    _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
    usleep(30000);

    // 以下任一分支失败都必须保持物理暂停（fail-closed）：模型可能只完成了
    // 一部分 reset，直接 unpause 会在未知姿态下恢复物理。
    if(!ros::service::waitForService("/gazebo/set_model_configuration", ros::Duration(1.0))){
        std::cout << "[ERROR] /gazebo/set_model_configuration is not available. "
                  << "Reset aborted; physics stays paused." << std::endl;
        return false;
    }
    if(!ros::service::waitForService("/gazebo/set_model_state", ros::Duration(1.0))){
        std::cout << "[ERROR] /gazebo/set_model_state is not available. "
                  << "Reset aborted; physics stays paused." << std::endl;
        return false;
    }

    std::string robot_name = "a1";
    nh.getParam("/robot_name", robot_name);
    const std::string model_name = robot_name + "_gazebo";

    gazebo_msgs::SetModelState state_srv;
    state_srv.request.model_state.model_name = model_name;
    state_srv.request.model_state.reference_frame = "world";
    state_srv.request.model_state.pose.position.x = getEnvDouble("COMPETITION_ROBOT_X", 0.0);
    state_srv.request.model_state.pose.position.y = getEnvDouble("COMPETITION_ROBOT_Y", -3.2);
    state_srv.request.model_state.pose.position.z =
        getEnvDouble("COMPETITION_ROBOT_Z", 0.6) + getEnvDouble("COMPETITION_RESET_Z_OFFSET", 0.05);
    state_srv.request.model_state.pose.orientation =
        tf::createQuaternionMsgFromYaw(getEnvDouble("COMPETITION_ROBOT_YAW", 1.5708));
    state_srv.request.model_state.twist.linear.x = 0.0;
    state_srv.request.model_state.twist.linear.y = 0.0;
    state_srv.request.model_state.twist.linear.z = 0.0;
    state_srv.request.model_state.twist.angular.x = 0.0;
    state_srv.request.model_state.twist.angular.y = 0.0;
    state_srv.request.model_state.twist.angular.z = 0.0;

    ros::ServiceClient state_client =
        nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    if(!state_client.call(state_srv) || !state_srv.response.success){
        std::cout << "[ERROR] Failed to reset A1 model state: "
                  << state_srv.response.status_message
                  << ". Reset aborted; physics stays paused." << std::endl;
        return false;
    }

    gazebo_msgs::SetModelConfiguration config_srv;
    config_srv.request.model_name = model_name;
    config_srv.request.urdf_param_name = "robot_description";
    config_srv.request.joint_names.assign(kResetJointNames.begin(), kResetJointNames.end());
    const std::array<double, 12> resetConfigPositions = getResetConfigJointPositions();
    config_srv.request.joint_positions.assign(resetConfigPositions.begin(), resetConfigPositions.end());
    std::cout << "[INFO] Reset joint configuration calf position: "
              << resetConfigPositions[2] << " rad (command stays "
              << kResetJointPositions[2] << " rad)." << std::endl;

    ros::ServiceClient config_client =
        nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    if(!config_client.call(config_srv) || !config_srv.response.success){
        std::cout << "[ERROR] Failed to reset A1 joint configuration: "
                  << config_srv.response.status_message
                  << ". Reset aborted; physics stays paused." << std::endl;
        return false;
    }

    if(!state_client.call(state_srv) || !state_srv.response.success){
        std::cout << "[ERROR] Failed to apply A1 model state after joint reset: "
                  << state_srv.response.status_message
                  << ". Reset aborted; physics stays paused." << std::endl;
        return false;
    }

    // unpause 前最后一帧命令：与刚写入的配置零位置误差、Kd=0，
    // 消除 set_model_configuration 传送产生的速度伪差经 Kd 放大导致的力矩饱和。
    setResetTransitionInitialCommand(resetConfigPositions[2]);
    _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
    usleep(150000);

    if(!callEmptyService("/gazebo/unpause_physics", 1.0)){
        std::cout << "[ERROR] Failed to unpause physics after reset. "
                  << "Physics stays paused." << std::endl;
        return false;
    }

    startResetTransition(resetConfigPositions[2]);
    return true;
#endif
}

void FSM::forcePassiveState(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->ioInter->setPassive();
    _ctrlComp->lowState->userCmd = UserCommand::L2_B;
    _ctrlComp->lowState->userValue.setZero();

    if(_currentState != _stateList.passive){
        if(_currentState != nullptr){
            _currentState->exit();
        }
        _currentState = _stateList.passive;
        _currentState->enter();
    }

    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    setResetDownCommand();
}

void FSM::setResetDownCommand(){
    // 这是恢复物理后的位置命令（与 State_Passive 下趴保持一致），
    // 不是 unpause 前写入模型的关节配置；后者见 getResetConfigJointPositions()。
    for(int i = 0; i < 12; ++i){
        _ctrlComp->lowCmd->motorCmd[i].mode = 10;
        _ctrlComp->lowCmd->motorCmd[i].q = static_cast<float>(kResetJointPositions[i]);
        _ctrlComp->lowCmd->motorCmd[i].dq = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].tau = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kp = 12.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kd = 2.0f;
    }
    _ctrlComp->setAllSwing();
}

void FSM::setResetTransitionInitialCommand(double calfConfig){
    // unpause 前最后一帧命令：与刚写入的模型配置零位置误差、Kd=0。
    // 否则 set_model_configuration 传送产生的速度伪差（实测 dq≈-57 rad/s）
    // 会经 Kd=2 放大为 >100 Nm 的需求并顶到 33.5 Nm 限位。
    for(int i = 0; i < 12; ++i){
        _ctrlComp->lowCmd->motorCmd[i].mode = 10;
        _ctrlComp->lowCmd->motorCmd[i].q = static_cast<float>(
            (i % 3 == 2) ? calfConfig : kResetJointPositions[i]);
        _ctrlComp->lowCmd->motorCmd[i].dq = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].tau = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kp = 12.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kd = 0.0f;
    }
    _ctrlComp->setAllSwing();
}

void FSM::startResetTransition(double calfStart){
    _resetTransitionActive = true;
    _resetTransitionStartUs = getSystemTime();
    _resetTransitionCalfStart = calfStart;
    _resetSettleFrames = 0;
    _resetFlopSeen = false;
    std::cout << "[INFO] Reset transition started: calf command ramps from "
              << calfStart << " to " << kResetJointPositions[2]
              << " rad with bounded gains." << std::endl;
}

void FSM::applyResetTransition(){
    if(!_resetTransitionActive){
        return;
    }
    if(_currentState != _stateList.passive){
        // 用户已切换到其他状态，立即交还标准控制
        _resetTransitionActive = false;
        _resetSettleFrames = 0;
        return;
    }

    const long long elapsed = getSystemTime() - _resetTransitionStartUs;
    const long long rampUs = 1000000;    // calf 位置命令 1 s 斜坡
    const long long timeoutUs = 8000000; // 有界上限 8 s，不是无限等待
    double ratio = static_cast<double>(elapsed) / rampUs;
    if(ratio > 1.0){
        ratio = 1.0;
    }
    const double calfTarget = kResetJointPositions[2];
    const double calfCmd =
        _resetTransitionCalfStart + (calfTarget - _resetTransitionCalfStart) * ratio;
    // 阻尼从 0 渐升到 0.5 并保持到过渡结束：压住传送伪差与落地塌陷速度尖峰
    // （实测 dq 峰值约 24 rad/s）经 Kd 的放大；结束后由 State_Passive 恢复 Kd=2。
    const double kd = 0.5 * ratio;

    bool settled = true;
    for(int i = 0; i < 12; ++i){
        _ctrlComp->lowCmd->motorCmd[i].Kp = 12.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kd = static_cast<float>(kd);
        if(i % 3 == 2){
            _ctrlComp->lowCmd->motorCmd[i].q = static_cast<float>(calfCmd);
        }
        const double qErr =
            std::fabs(_ctrlComp->lowState->motorState[i].q - kResetJointPositions[i]);
        const double dqAbs = std::fabs(_ctrlComp->lowState->motorState[i].dq);
        // 塌陷只在关节接近目标折叠角后发生；以此区分 unpause 时的传送速度伪差，
        // 避免塌陷前误判“已见过冲击”而提前恢复 Kd=2。
        if(qErr < 0.2 && dqAbs > 3.0){
            _resetFlopSeen = true;
        }
        if(i % 3 == 2 && (qErr > 0.08 || dqAbs > 0.3)){
            settled = false;
        }
    }

    if(ratio >= 1.0 && settled && _resetFlopSeen){
        _resetSettleFrames += 1;
    }else{
        _resetSettleFrames = 0;
    }
    if(_resetSettleFrames >= 50){
        _resetTransitionActive = false;
        _resetSettleFrames = 0;
        std::cout << "[INFO] Reset transition complete. Standard passive hold restored." << std::endl;
    }else if(elapsed > timeoutUs){
        _resetTransitionActive = false;
        _resetSettleFrames = 0;
        std::cout << "[WARNING] Reset transition timed out; restoring standard passive hold." << std::endl;
    }
}
