#ifndef __FLIGHTCTRlPRO_HPP__
#define __FLIGHTCTRLPRO_HPP__

//头文件
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <flight_ctrl/SetFlightTask.h>
#include <flight_ctrl/SetFlightTask_Topic.h>
#include <flight_ctrl/SetDebugTarget.h>
#include <flight_ctrl/TriggerShutdown.h>
#include "pid.hpp"
#include <bitset>
#include <boost/crc.hpp>
#include <boost/filesystem.hpp>
#include <dynamic_reconfigure/server.h>
#include <flight_ctrl/PidGainsConfig.h>


struct wayPoints
{
    //Eigen::Vector2d xy_grid;
    Eigen::Vector4d xyzy_map;
    double hover_time;
};

struct flyPath
{
    std::string hash;
    int steps;
    std::vector<wayPoints> waypoints_;
};

/*MissionManager类设计 主要关注任务的切换，扩展*/
class MissionManager 
{
private:
    flyPath fpath;
    int current_index;
    int current_mission_id;
    std::bitset<63> noFlyBitmap; //位图表示法：63位对应9x7网格（0=可飞，1=禁飞）
    //std::set<int> noFlyIndices; //网格索引集合：存储禁飞区AnBm对应的线性索引

public:
    MissionManager(); 
    ~MissionManager();
    bool loadMission(const std::string& hash);   
    const Eigen::Vector4d& getCurrentWaypoint();
    int getCurrentIndex();
    void nextWaypoint();
    bool isFinished();
    void reset();
    std::pair<int, int> ab2Grid(const std::string& ab);
    int grid2Idx(const std::pair<int, int>& g);
    Eigen::Vector2d grid2Map(int i, int j);
    std::pair<int, int> mapToGrid(const Eigen::Vector3d& map_pos);
    std::bitset<63>& setNoFlyZones(const std::vector<std::string>& zones);  //设计禁飞区并且返回位图
    std::string getBitmapHash(const std::bitset<63>& bs) const; //计算哈希值


};

/* MapMotion类的设计，构造map坐标系,负责处理坐标系转化问题，以及控制量计算*/
class MapMotion
{
private:
    Eigen::Vector3d origin_pos; //原点的位置
    double origin_yaw;  //原点偏航角    关于yaw，此处记录方便使用弧度，其余注意区分(-180 - 180)
    Eigen::Matrix3d R;  //旋转矩阵enu -> map 机头x 右手系
    Eigen::Matrix3d R_; //旋转矩阵map -> enu
    pid x, y, z, yaw; //四个控制器对象
    double timeout;  //超时时间
    ros::Time start_time;   //开始时间
    Eigen::Vector3d target_pos_map; // 目标位置
    double target_yaw_map;  //目标yaw 角度
    double pos_err , yaw_err;   //判断位置到达的容差 角度
    bool hover_flag = false;    //悬停状态标记
    ros::Time hover_start_Time; //悬停开始时间
    double hover_duration;  //悬停时间
    //reconfigure动态调参
    dynamic_reconfigure::Server<flight_ctrl::PidGainsConfig> server;
    dynamic_reconfigure::Server<flight_ctrl::PidGainsConfig>::CallbackType f;
    void dy_pid_Callback(flight_ctrl::PidGainsConfig &config,uint32_t level);

public:
    MapMotion();
    ~MapMotion();
    void init(const geometry_msgs::PoseStamped& ); //初始化零位姿 以及必要的量
    Eigen::Vector3d transform(const Eigen::Vector3d& );   //坐标转化外部接口
    geometry_msgs::Twist compute(const geometry_msgs::PoseStamped&);  //计算坐标输出
    void setTarget(const Eigen::Vector3d ,const double );   //设置目标值_
    void setTarget(const geometry_msgs::PoseStamped& );   //设置目标值_
    void setErr(const double& ,const double&);    //设置容差
    void setTimeout(const double& );
    void setStartTime(const ros::Time& time);
    bool isArrived(const geometry_msgs::PoseStamped& );   //是否到达目标点
    bool isTimeout(const ros::Time& );   //是否超时
    void startHover(const double& );    //开始悬停
    void stopHover();     //停止悬停
    bool isHover();     //是否处于悬停状态
    bool isHoverFinished(const ros::Time& ); 


    //坐标系转化
    Eigen::Vector3d enu2Map_pos(const Eigen::Vector3d& );  
    Eigen::Vector3d map2Enu_pos(const Eigen::Vector3d& );
    Eigen::Vector3d enu2Map_vel(const Eigen::Vector3d& ); 
    Eigen::Vector3d map2Enu_vel(const Eigen::Vector3d& );
    double enu2Map_yaw(const double& );  
    double map2Enu_yaw(const double& );
};

enum FlightTask
{
    Standby,
    Takeoff,
    Land,
    Mission,
    Interrupt,
    Hover,
    Emergency,
    Debug
};

/*FlightCore 框架*/
class FlightCore
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    
    ros::Subscriber state_sub;  //状态订阅
    ros::Subscriber reference_position_sub; //位置反馈 包含速度信息
    ros::Subscriber reference_velocity_sub; //速度反馈
    ros::Subscriber vins_position_sub;  //vins订阅

    //目标位置 目标速度 目标姿态发布
    ros::Publisher target_position_pub,target_velocity_pub,target_pose_pub;

    ros::ServiceClient arming_request_client;   //解锁服务
    ros::ServiceClient mode_change_client;  //模式切换服务
    ros::ServiceServer set_FlightTask_server;
    ros::ServiceClient set_FlightTask_client;//任务切换服务
    ros::ServiceServer set_DebugTarget_server;
    ros::ServiceClient set_DebugTarget_client;//debug发点服务
    ros::ServiceServer kill_trigger_server;
    
    ros::Time last_request,land_time;//上次操作请求时间，降落时间

    FlightTask FT;  //飞行状态
    flight_ctrl::SetFlightTask::Request s;//切换飞行状态消息

    mavros_msgs::State state_cb; //状态回调变量
    geometry_msgs::PoseStamped reference_position_cb; //参考位置回调变量
    geometry_msgs::TwistStamped reference_velocity_cb; //参考速度回调变量
    nav_msgs::Odometry vins_position_cb;    //vins位置
    geometry_msgs::Twist pub_vel;   //最终发布的速度
    geometry_msgs::PoseStamped last_pos; //上次位姿
    geometry_msgs::PoseStamped cache_pos; //缓存中间位置
    geometry_msgs::PoseStamped emergency_pos;   //vins漂移时位姿
    bool is_changed;    //状态机是否改变
    double vins_err;    //vins的误差
    bool vins_drifted;  //vins是否漂移
    double vins_err_pos_The;    //vins漂移判断的容忍误差_位置
    double vins_err_vel_The;    //vins漂移判断的容忍误差_速度
    

    mavros_msgs::SetMode mode_change_msg;
    mavros_msgs::CommandBool arming_request_msgs;

    int rate_count; //状态循环分频计数
    ros::Rate rate;
    bool initial_flag;  //初始化标志
    bool land_flag; // 降落标记
    bool use_vins;  //是否使用vins
    int mission_id; //任务id
    int last_index; //任务标记
    double last_pos_z;  //上次z坐标

    MapMotion move;
    MissionManager mission;

    void state_Callback(const mavros_msgs::State::ConstPtr& msg);   //状态回调
    void reference_position_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);  //位置反馈回调
    void reference_velocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void vins_position_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void cmdloop_Callback();    //控制循环回调
    void statusloop_Callback(); //状态循环回调
    bool set_FlightTask_Callback(flight_ctrl::SetFlightTask::Request& req,flight_ctrl::SetFlightTask::Response& res);   //飞行状态切换回调
    bool set_DebugTarget_Callback(flight_ctrl::SetDebugTarget::Request& req,flight_ctrl::SetDebugTarget::Response& res);
    bool kill_trigger_Callback(flight_ctrl::TriggerShutdown::Request&,flight_ctrl::TriggerShutdown::Response& res); //自杀服务

    //cmdloop中不同分支的对应函数
    void cmd_Task_Standby();
    void cmd_Task_Takeoff(double h);
    void cmd_Task_Land();
    void cmd_Task_Mission();
    void cmd_Task_Interupt();
    void cmd_Task_Debug();
    void cmd_Task_Emergency();

    double round_(double);  //约数函数
    geometry_msgs::PoseStamped getNowPose();    //获得当前位姿

public:
    FlightCore(const ros::NodeHandle&,const ros::NodeHandle&);
    ~FlightCore();
    void main_loop();
};
#endif