/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
/*
 * junior_ctrl 控制器入口。
 *
 * auto.sh 最后启动本可执行文件。它负责初始化 Unitree 控制框架、选择仿真/真机 IO、
 * 设置控制周期 UNITREE_CTRL_DT，然后不断调用 ControlFrame::run()。
 *
 * 用户在前台终端输入 2/6 后，FSM 会切到站立/RL 状态；RL 状态中订阅 /cmd_vel，
 * 经过策略网络推理后写入 12 个关节命令，Gazebo 里的 ros_control 再执行这些命令。
 */
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

#include "interface/IOFREEDOGSDK.h"
bool running = true;

// 捕获 Ctrl+C，让主循环自然退出并释放控制组件。
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    // 尝试使用实时调度，降低控制循环抖动；没有权限时只打印错误，不阻止仿真启动。
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

double readControllerDt()
{
    // auto.sh 默认导出 UNITREE_CTRL_DT=0.004，把控制频率从 500 Hz 降到 250 Hz，
    // 目的是在大 Gazebo 场景中减少控制循环超时 warning。
    const double defaultDt = 0.002; // 500 Hz, Unitree upstream default.
    const char *envValue = std::getenv("UNITREE_CTRL_DT");
    if (envValue == nullptr || envValue[0] == '\0')
    {
        return defaultDt;
    }

    char *end = nullptr;
    errno = 0;
    double dt = std::strtod(envValue, &end);
    if (errno != 0 || end == envValue || *end != '\0' || !std::isfinite(dt) || dt < 0.001 || dt > 0.02)
    {
        std::cout << "[WARNING] Invalid UNITREE_CTRL_DT='" << envValue
                  << "', using default " << defaultDt << "s." << std::endl;
        return defaultDt;
    }
    return dt;
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

#ifdef RUN_ROS
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;

#ifdef COMPILE_WITH_SIMULATION
    // 仿真模式：IOROS 从 Gazebo/ROS 读取低层状态，并向 ros_control 写关节命令。
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT
    // 真机模式：使用 Unitree SDK。当前比赛仿真默认不走这条分支。
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT
    IOFREEDOGSDK *ioInter_freedog;
    ioInter_freedog = new IOFREEDOGSDK();
    // CtrlComponents 是 Unitree 控制框架的共享上下文，FSM 各状态都会读写它。
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter,ioInter_freedog);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = readControllerDt();
    std::cout << "controller dt: " << ctrlComp->dt
              << " s, frequency: " << 1.0 / ctrlComp->dt << " Hz" << std::endl;
    ctrlComp->running = &running;

#ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
#endif

    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();

    // ControlFrame 持有 FSM，run() 每个控制周期执行一次当前状态逻辑。
    ControlFrame ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);

    while (running)
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
