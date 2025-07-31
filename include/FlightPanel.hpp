#pragma once
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <flight_ctrl/SetFlightTask.h>
#include <flight_ctrl/SetDebugTarget.h>
#include <flight_ctrl/TriggerShutdown.h>   // ← 新增
#include <thread>
#include <atomic>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <sstream>

class FlightPanel
{
public:
    FlightPanel();
    ~FlightPanel();
    void run();
    void shutdown();

private:
    void print_menu();
    int  getch_nonblock();
    void kb_loop();

    void set_mode(const std::string& mode);
    void set_arm(bool arm);
    void set_flight_task(const std::string& task);
    void debug_interactive();

    ros::NodeHandle nh_;
    ros::ServiceClient set_mode_client_, arming_client_, set_task_client_;
    ros::ServiceClient set_trigger_client_;   // ← 用于发送关机请求
    ros::ServiceClient set_debug_target_client_;

    std::thread kb_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> shutdown_requested_{false};   // ← 主线程关机标志
};