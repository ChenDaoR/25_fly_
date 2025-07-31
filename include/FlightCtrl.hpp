#ifndef __FLIGHTCTRl_HPP__
#define __FLIGHTCTRL_HPP__

//头文件
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <flight_ctrl/SetFlightTask.h>
#include <flight_ctrl/SetFlightTask_Topic.h>
#include "pid.hpp"
#include <vector>
#include <cmath>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

enum SYS_STATE
{
    STATE_UNINIT,//未初始化
    STATE_BOOT,//启动
    STATE_CALIBRATIN,//校准
    STATE_STANDBY,//待机
    STATE_ACTIVE,//活跃
    STATE_EMERGENCY,//紧急
    STATE_TERMINATION //终止
};

enum FlightTask
{
    Standby,
    Takeoff,
    Land,
    Mission
};

class FlightCtrl
{

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    
    ros::Subscriber state_sub;//状态订阅
    ros::Subscriber reference_position_sub;//位置反馈 包含速度信息
    ros::Subscriber set_FlightTask_Topic_sub;//订阅状态切换话题
    ros::Subscriber vins_position_sub;//vins位置反馈
    
    //目标位置 目标速度 目标姿态发布
    ros::Publisher target_position_pub,target_velocity_pub,target_pose_pub;
    ros::Publisher set_FlightTask_Topic_pub;//状态切换发布者


    ros::ServiceClient arming_request_client;//解锁服务
    ros::ServiceClient mode_change_client;//模式切换服务
    ros::ServiceClient land_service;//降落服务
    ros::ServiceServer set_FlightTask_server;
    ros::ServiceClient set_FlightTask_client;//模式切换服务

    ros::Timer cmdloop_timer, statusloop_timer, trigger_timer;//控制定时器 反馈定时器 触发定时器
    ros::Time last_request,last_mission,land_time;//上次请求时间，上次任务时间

    double CmdRate,StatusRate;//控制频率 状态更新频率
    SYS_STATE state;//状态
    FlightTask FT;//飞行状态
    flight_ctrl::SetFlightTask::Request s;//切换飞行状态消息
    flight_ctrl::SetFlightTask_Topic s_Topic;

    mavros_msgs::State state_cb; //状态回调变量
    geometry_msgs::PoseStamped reference_position_cb; //目标位置回调变量
    nav_msgs::Odometry vins_position_cb;
    geometry_msgs::PoseStamped cache_position; //中间缓存位置
    geometry_msgs::Twist cache_vel;//中间缓存速度
    Eigen::Vector2d cache_TF;//坐标转换缓存
    double initial_yaw;//初始yaw
    geometry_msgs::PoseStamped initial_pose;//初始位姿
    Eigen::Rotation2Dd* ro_matrix;//旋转矩阵

    bool initial_flag;//初始化标志

    int ctrler;//控制器选择
    int mission_num;//飞行航点数目

    mavros_msgs::SetMode mode_change_msg;
    mavros_msgs::CommandBool arming_request_msgs;

    pid cx,cy,cz,c_yaw;//pid控制器
    int rate_count;//状态循环分频
    ros::Rate rate;
    bool land_switch;//降落标记位
    bool land_switch_;//
    double pos_err,yaw_err;//位置误差 旋转误差
    double vins_err;//vins的误差


    void state_Callback(const mavros_msgs::State::ConstPtr& msg);//状态回调
    void reference_position_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);//位置反馈回调
    void vins_position_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    // void cmdloop_Callback(const ros::TimerEvent& event);//控制循环回调
    // void statusloop_Callback(const ros::TimerEvent& event);//状态循环回调
    void trigger_Callback(const ros::TimerEvent& event);
    void cmdloop_Callback();//控制循环回调
    void statusloop_Callback();//状态循环回调
    bool set_FlightTask_Callback(flight_ctrl::SetFlightTask::Request& req,flight_ctrl::SetFlightTask::Response& res);//飞行状态切换回调
    void set_FlightTask_Topic_Callback(const flight_ctrl::SetFlightTask_Topic::ConstPtr& msg);

    //cmdloop中不同分支的对应函数
    void cmd_Task_Standby();
    void cmd_Task_Takeoff(double h);
    void cmd_Task_Land();
    void cmd_Task_Mission();

    void fly_to(double,double,double,double,double,bool); //封装飞定点函数
    void fly_to_(Eigen::Vector4d ,double,bool);
    double round_(double);


public:
    FlightCtrl(const ros::NodeHandle&,const ros::NodeHandle&);
    // FlightCtrl(const ros::NodeHandle&,const ros::NodeHandle&,double,double);
    ~FlightCtrl();
    void main_loop();
};

#endif