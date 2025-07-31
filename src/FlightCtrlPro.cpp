#include "FlightCtrlPro.hpp"

#define cmdRate 150
#define TAKEOFF_HEIGHT 1
#define POS_ERR 0.1
#define YAW_ERR 0.5
#define TIMEOUT 10
#define MISSION_ID 1
#define USE_VINS 1
#define VINS_ERR_THE_POS 1
#define VINS_ERR_THE_VEL 1

MapMotion::MapMotion()
{
}

MapMotion::~MapMotion()
{
}

void MapMotion::init(const geometry_msgs::PoseStamped& origin_)
{
    //此处应与FlightCore进行配合，更改，在flightCore初始化后把必要参数全部加载设置

    origin_pos(0) = origin_.pose.position.x;
    origin_pos(1) = origin_.pose.position.y;
    origin_pos(2) = origin_.pose.position.z;
    origin_yaw = tf2::getYaw(origin_.pose.orientation);

     R_ << cos(origin_yaw),  -sin(origin_yaw),0,
           sin(origin_yaw),  cos(origin_yaw), 0,
           0,                 0,              1;

     R << cos(origin_yaw),  sin(origin_yaw), 0,
         -sin(origin_yaw),  cos(origin_yaw), 0,
          0,                 0,              1;


    ROS_INFO("[MapMotion] init done. origin=(%.2f,%.2f,%.2f) yaw=%.2f",
             origin_pos(0), origin_pos(1), origin_pos(2), origin_yaw);

    x.setGains(1,0.05,0);
    x.setDt(1.0 / cmdRate);
    x.setIntegralLimit(0.1);
    x.setOutputLimit(0,0.2);
    x.setAdvancedFlag(true,false,false,false);
    x.setAdvancedThreshold(0.2,0,0);

    y.setGains(1,0.05,0);
    y.setDt(1.0 / cmdRate);
    y.setIntegralLimit(0.1);
    y.setOutputLimit(0,0.2);
    y.setAdvancedFlag(true,false,false,false);
    y.setAdvancedThreshold(0.2,0,0);

    z.setGains(1.2,0.1,0.3);
    z.setDt(1.0 / cmdRate);
    z.setIntegralLimit(0.5);
    z.setOutputLimit(0,0.8);
    z.setAdvancedFlag(false,false,true,true);
    z.setAdvancedThreshold(0,0,0.3);

    yaw.setGains(0.15,0,0);
    yaw.setDt(1.0 / cmdRate);
    yaw.setIntegralLimit(0);
    yaw.setOutputLimit(0,0.3);
    yaw.setAdvancedFlag(false,false,false,false);
    yaw.setAdvancedThreshold(0,0,0);

    f = boost::bind(&MapMotion::dy_pid_Callback,this,_1,_2);
    //server.setCallback(f);

}

Eigen::Vector3d MapMotion::enu2Map_pos(const Eigen::Vector3d& enu_pos)
{
    return R * (enu_pos - origin_pos);
}

Eigen::Vector3d MapMotion::map2Enu_pos(const Eigen::Vector3d& map_pos)
{
    return R_ * map_pos + origin_pos;
}

Eigen::Vector3d MapMotion::enu2Map_vel(const Eigen::Vector3d& enu_vel)
{
    return R * enu_vel;
}

Eigen::Vector3d MapMotion::map2Enu_vel(const Eigen::Vector3d& map_vins)
{
    return R_ *map_vins;
}

double MapMotion::enu2Map_yaw(const double& enu_yaw)   //弧度
{
    return (enu_yaw - origin_yaw) * 180.0 / M_PI;   //角度
}

double MapMotion::map2Enu_yaw(const double& map_yaw)   //角度
{
    return map_yaw * M_PI / 180.0 + origin_yaw;   //弧度
}

Eigen::Vector3d MapMotion::transform(const Eigen::Vector3d& enu_pos)
{
    return enu2Map_pos(enu_pos);
}

void MapMotion::setTarget(const Eigen::Vector3d pos,const double yaw_)  //角度
{
    //最终发的是enu坐标系下的速度命令，转到enu坐标系
    Eigen::Vector3d target_pos_enu = map2Enu_pos(pos);
    double target_yaw_enu = map2Enu_yaw(yaw_);

    target_pos_map = pos;
    target_yaw_map = yaw_;

    x.setTarget(target_pos_enu(0));
    y.setTarget(target_pos_enu(1));
    z.setTarget(target_pos_enu(2));
    yaw.setTarget(target_yaw_enu);
}

// void MapMotion::setTarget(const geometry_msgs::PoseStamped& target_pos)
// {
//     Eigen::Vector3d pos(target_pos.pose.position.x,
//                         target_pos.pose.position.y,
//                         target_pos.pose.position.z);
//     Eigen::Vector3d target_pos_enu = map2Enu_pos(pos);
//     double target_yaw_enu = map2Enu_yaw(tf2::getYaw(target_pos.pose.orientation) * 180.0 / M_PI);

//     x.setTarget(target_pos_enu(0));
//     y.setTarget(target_pos_enu(1));
//     z.setTarget(target_pos_enu(2));
//     yaw.setTarget(target_yaw_enu * 180.0 / M_PI);
// }

void MapMotion::setErr(const double& pos,const double& yaw_)
{
    pos_err = pos;
    yaw_err = yaw_;
}

void MapMotion::setTimeout(const double& time)
{
    timeout = time;
}

void MapMotion::setStartTime(const ros::Time& time)
{
    start_time = time;
}

bool MapMotion::isArrived(const geometry_msgs::PoseStamped& curr_pos_enu)
{
     if (hover_flag) return false;

    Eigen::Vector3d curr_enu(curr_pos_enu.pose.position.x,
                             curr_pos_enu.pose.position.y,
                             curr_pos_enu.pose.position.z);
    Eigen::Vector3d curr_map = enu2Map_pos(curr_enu);

    double curr_yaw_enu = tf2::getYaw(curr_pos_enu.pose.orientation);
    double curr_yaw_map = enu2Map_yaw(curr_yaw_enu);

    return (target_pos_map - curr_map).norm() < pos_err
    && std::abs(angles::shortest_angular_distance(curr_yaw_map * M_PI / 180.0, target_yaw_map * M_PI / 180.0)) * 180.0 / M_PI< yaw_err;

}

bool MapMotion::isTimeout(const ros::Time& curr_time)
{
    if(timeout)    return curr_time - start_time > ros::Duration(timeout);
    else    return false;
}

geometry_msgs::Twist MapMotion::compute(const geometry_msgs::PoseStamped& curr_pos_map)
{
    // ----------- 1. 当前 Map 位姿 -----------
    Eigen::Vector3d curr_map(curr_pos_map.pose.position.x,
                             curr_pos_map.pose.position.y,
                             curr_pos_map.pose.position.z);
    double curr_yaw_map = tf2::getYaw(curr_pos_map.pose.orientation);   //弧度

    // ----------- 2. 当前 Enu 位姿 -----------
    Eigen::Vector3d curr_enu = map2Enu_pos(curr_map);
    double curr_yaw_enu = map2Enu_yaw(curr_yaw_map * 180.0 / M_PI);    //输入角度 ， 输出弧度

    // ----------- 3. 目标 ENU 位姿 -----------
    Eigen::Vector3d target_enu = map2Enu_pos(target_pos_map);
    double target_yaw_enu = map2Enu_yaw(target_yaw_map);

    // ----------- 4. PID 计算 -----------
    geometry_msgs::Twist output;
    output.linear.x = x.compute(curr_enu(0));
    output.linear.y = y.compute(curr_enu(1));
    output.linear.z = z.compute(curr_enu(2));
    output.angular.z = yaw.compute_(angles::shortest_angular_distance(curr_yaw_enu,
                                                                      target_yaw_enu) * 180.0 / M_PI);

    //----------- 5. 调试输出 -----------
    ROS_INFO_THROTTLE(0.5,
        "[DEBUG]\n"
        "  Map  now  : [%.3f, %.3f, %.3f] yaw %.1f\n"
        "  ENU  now  : [%.3f, %.3f, %.3f] yaw %.1f\n"
        "  ENU target: [%.3f, %.3f, %.3f] yaw %.1f\n"
        "  PID vel   : [%.3f, %.3f, %.3f] yaw %.3f",
        curr_map(0),  curr_map(1),  curr_map(2),  curr_yaw_map * 180.0 / M_PI,
        curr_enu(0),  curr_enu(1),  curr_enu(2),  curr_yaw_enu * 180.0 / M_PI,
        target_enu(0), target_enu(1), target_enu(2), target_yaw_enu * 180.0 / M_PI,
        output.linear.x, output.linear.y, output.linear.z, output.angular.z);

    return output;
}

void MapMotion::startHover(const double& duration)
{
    hover_flag = true;
    hover_duration = duration;
    hover_start_Time = ros::Time::now();
    ROS_INFO("[Hover] Started for %.1f seconds", duration);
}

void MapMotion::stopHover()
{
    hover_flag = false;
    hover_duration = 0;
}

bool MapMotion::isHover()
{
    return hover_flag;
}

bool MapMotion::isHoverFinished(const ros::Time& curr_time)
{
    if (!hover_flag) return false;
    return (ros::Time::now() - hover_start_Time).toSec() >= hover_duration;


}

void MapMotion::dy_pid_Callback(flight_ctrl::PidGainsConfig &config,uint32_t level)
{
    // 更新x控制器参数
    x.setGains(config.x_p, config.x_i, config.x_d);
    x.setIntegralLimit(config.x_integral_lim);
    x.setOutputLimit(config.x_min_output, config.x_max_output);
    x.setAdvancedFlag(config.x_integral_sepration_flag, 
                     config.x_dead_zone_flag, 
                     config.x_d_filter_flag, 
                     config.x_d_first_flag);
    x.setAdvancedThreshold(config.x_integral_sepration_threshold, 
                          config.x_dead_zone_threshold, 
                          config.x_d_filter_value);

    // 更新y控制器参数
    y.setGains(config.y_p, config.y_i, config.y_d);
    y.setIntegralLimit(config.y_integral_lim);
    y.setOutputLimit(config.y_min_output, config.y_max_output);
    y.setAdvancedFlag(config.y_integral_sepration_flag, 
                     config.y_dead_zone_flag, 
                     config.y_d_filter_flag, 
                     config.y_d_first_flag);
    y.setAdvancedThreshold(config.y_integral_sepration_threshold, 
                          config.y_dead_zone_threshold, 
                          config.y_d_filter_value);

    // 更新z控制器参数
    z.setGains(config.z_p, config.z_i, config.z_d);
    z.setIntegralLimit(config.z_integral_lim);
    z.setOutputLimit(config.z_min_output, config.z_max_output);
    z.setAdvancedFlag(config.z_integral_sepration_flag, 
                     config.z_dead_zone_flag, 
                     config.z_d_filter_flag, 
                     config.z_d_first_flag);
    z.setAdvancedThreshold(config.z_integral_sepration_threshold, 
                          config.z_dead_zone_threshold, 
                          config.z_d_filter_value);

    // 更新yaw控制器参数
    yaw.setGains(config.yaw_p, config.yaw_i, config.yaw_d);
    yaw.setIntegralLimit(config.yaw_integral_lim);
    yaw.setOutputLimit(config.yaw_min_output, config.yaw_max_output);
    yaw.setAdvancedFlag(config.yaw_integral_sepration_flag, 
                       config.yaw_dead_zone_flag, 
                       config.yaw_d_filter_flag, 
                       config.yaw_d_first_flag);
    yaw.setAdvancedThreshold(config.yaw_integral_sepration_threshold, 
                            config.yaw_dead_zone_threshold, 
                            config.yaw_d_filter_value);
}

MissionManager::MissionManager(){}

MissionManager::~MissionManager(){}

bool MissionManager::loadMission(const int& mission_id)
{
    std::ifstream file("/home/feng/Myothers/config/missions.json");
    if (!file.is_open())
    {
        ROS_ERROR("[MISSION] Failed to open config/missions.json");
        return false;
    }

    nlohmann::json j;
    try
    {
        file >> j;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[MISSION] JSON parse error: %s", e.what());
        return false;
    }

    ROS_INFO("[MISSION] Loaded missions.json, found %lu missions", j["missions"].size());

    for (const auto& mission : j["missions"])
    {
        if (mission["id"].get<int>() == mission_id)
        {
            current_mission_id = mission_id;
            waypoints_.clear();
            ROS_INFO("[MISSION] Loading mission ID %d with %lu waypoints", 
                     mission_id, mission["waypoints"].size());
            for (const auto& wp : mission["waypoints"])
            {
                waypoints_.push_back(Eigen::Vector4d(wp[0], wp[1], wp[2], wp[3]));
                ROS_INFO("[MISSION] Waypoint %lu: [%.2f, %.2f, %.2f, %.2f]", waypoints_.size()-1,  wp[0].get<double>(),  wp[1].get<double>(),  wp[2].get<double>(),  wp[3].get<double>());
            }
            current_index = 0;
            return true;
        }
    }

    std::cerr << "Mission with id " << mission_id << " not found" << std::endl;
    return false;
}

const Eigen::Vector4d& MissionManager::getCurrentWaypoint()
{
    static Eigen::Vector4d null_waypoint(0.0, 0.0, 0.0, 0.0);
    if (current_index >= 0 && current_index < waypoints_.size())
    {
        return waypoints_[current_index];
    }
    return null_waypoint;
}

int MissionManager::getCurrentIndex()
{
    return current_index;
}

void MissionManager::nextWaypoint()
{
    if (current_index < waypoints_.size())
    {
        current_index++;
    }
}

bool MissionManager::isFinished()
{
    return current_index >= waypoints_.size();
}

void MissionManager::reset()
{
    current_index = 0;
}

FlightCore::FlightCore(const ros::NodeHandle& nh_,const ros::NodeHandle& nh_private)
    :nh(nh_),nh_private(nh_private),rate(150),initial_flag(false)
{
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",50,&FlightCore::state_Callback,this,ros::TransportHints().tcpNoDelay());
    reference_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",50,&FlightCore::reference_position_Callback,this,ros::TransportHints().tcpNoDelay()); 
    reference_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",50,&FlightCore::reference_velocity_Callback,this,ros::TransportHints().tcpNoDelay());    
    vins_position_sub = nh.subscribe<nav_msgs::Odometry>("vins_estimator/odometry",10,&FlightCore::vins_position_Callback,this,ros::TransportHints().tcpNoDelay());
    target_velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    
    arming_request_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_change_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    set_FlightTask_server = nh.advertiseService("FlightCtrl/SetFlightTask",&FlightCore::set_FlightTask_Callback,this);
    set_FlightTask_client = nh.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");
    set_DebugTarget_server = nh.advertiseService("FlightCtrl/SetDebugTarget",&FlightCore::set_DebugTarget_Callback,this);
    set_DebugTarget_client = nh.serviceClient<flight_ctrl::SetDebugTarget>("FlightCtrl/SetDebugTarget");
    kill_trigger_server = nh.advertiseService("FlightPanel/RequestShutdown",&FlightCore::kill_trigger_Callback, this);

    mode_change_msg.request.custom_mode = "OFFBOARD";
    arming_request_msgs.request.value = true;

    FT = Standby;
    last_index = -1;
    is_changed = true;
    land_flag = false;
    use_vins = USE_VINS;
    rate_count = 0;

    last_pos_z = 0;

    vins_err_pos_The = VINS_ERR_THE_POS;
    vins_err_vel_The = VINS_ERR_THE_VEL;
    vins_drifted = false;

    move.setTimeout(TIMEOUT);
    move.setErr(POS_ERR,YAW_ERR);

    mission_id = MISSION_ID;
    mission.loadMission(mission_id);

    last_request = ros::Time::now();
}

FlightCore::~FlightCore()
{

}

void FlightCore::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
    state_cb = *msg;
}

void FlightCore::reference_position_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    reference_position_cb = *msg;

    if(!initial_flag)
    {
        move.init(*msg);
        initial_flag = true;
        cache_pos = *msg;
    }
} 

void FlightCore::reference_velocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    reference_velocity_cb = *msg;
} 

void FlightCore::vins_position_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vins_position_cb = *msg;

    Eigen::Vector3d curr_enu(reference_position_cb.pose.position.x,
                             reference_position_cb.pose.position.y,
                             reference_position_cb.pose.position.z);
    Eigen::Vector3d curr_map = move.enu2Map_pos(curr_enu);
    Eigen::Vector2d ref(
    curr_map(0),
    curr_map(1)
    );
    Eigen::Vector2d vins(
        vins_position_cb.pose.pose.position.x,
        vins_position_cb.pose.pose.position.y
    );
    Eigen::Vector2d err = ref - vins;
    vins_err = err.norm();

    ROS_INFO_THROTTLE(0.5,"map_x_pos: %.3f , map_y_pos: %.3f , vins_x_pos: %.3f , vins_y_pos: %.3f",curr_map(0),curr_map(1),vins_position_cb.pose.pose.position.x,vins_position_cb.pose.pose.position.y);


    bool pos_ok = std::abs(err(0)) > vins_err_pos_The || std::abs(err(1)) > vins_err_pos_The || vins_err > vins_err_pos_The * 1.4;

    curr_enu = Eigen::Vector3d(reference_velocity_cb.twist.linear.x,
                            reference_velocity_cb.twist.linear.y,
                            reference_velocity_cb.twist.linear.z);
    curr_map = move.enu2Map_vel(curr_enu);
    ref = Eigen::Vector2d(
        curr_map(0),
        curr_map(1)
    );
    vins = Eigen::Vector2d(
        vins_position_cb.twist.twist.linear.x,
        vins_position_cb.twist.twist.linear.y
    );
    err = ref - vins;

    ROS_INFO_THROTTLE(0.5,"map_x_vel: %.3f , map_y_vel: %.3f , vins_x_vel: %.3f , vins_y_vel: %.3f",curr_map(0),curr_map(1),vins_position_cb.twist.twist.linear.x,vins_position_cb.twist.twist.linear.y);

    bool vel_ok = std::abs(err(0)) > vins_err_vel_The || std::abs(err(1)) > vins_err_vel_The || err.norm() > vins_err_vel_The * 1.4;

    // if(use_vins && (pos_ok||vel_ok))
    // {
    //     emergency_pos = reference_position_cb;
    //     vins_drifted = true;
    //     ROS_INFO("vins drifted...change to land");

    //     FT = Land;
    //     is_changed = true;
    //     cache_pos = getNowPose();
    // }

} 

bool FlightCore::set_FlightTask_Callback(flight_ctrl::SetFlightTask::Request& req,flight_ctrl::SetFlightTask::Response& res)
{
    s = req;
    if(s.value == "Standby")
    {
        ROS_INFO("now flight_task is Standby");
        FT = Standby;
        cache_pos = getNowPose();
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Takeoff")
    {
        ROS_INFO("now flight_task is Takeoff");
        FT = Takeoff;
        cache_pos = getNowPose();
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Mission")
    {
        ROS_INFO("now flight_task is Mission");
        FT = Mission;
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Land")
    {
        ROS_INFO("now flight_task is Land");
        FT = Land;
        cache_pos = getNowPose();
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Interupt")
    {
        ROS_INFO("now flight_task is Interupt");
        FT = Interrupt;
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Debug")
    {
        ROS_INFO("now flight_task is Debug");
        FT = Debug;
        cache_pos = getNowPose();
        res.success = true;
        land_flag = false;
    }
    else if(s.value == "Emergency")
    {
        ROS_INFO("now flight_task is Emergency");
        FT = Emergency;
        cache_pos = getNowPose();
        res.success = true;
        land_flag = false;
    }
    else    res.success = false;

    is_changed = true;
    return true;
}

bool FlightCore::set_DebugTarget_Callback(flight_ctrl::SetDebugTarget::Request& req,flight_ctrl::SetDebugTarget::Response& res)
{
    Eigen::Vector3d target_pos(req.x,req.y,req.z);
    move.setTarget(target_pos,req.yaw);
    ROS_INFO("[DEBUG] New target set: [%.2f, %.2f, %.2f, yaw=%.1f ]", req.x, req.y, req.z, req.yaw);
    res.success = true;
    return true;
}

bool FlightCore::kill_trigger_Callback(flight_ctrl::TriggerShutdown::Request&,flight_ctrl::TriggerShutdown::Response& res)
{
    ROS_INFO("Shutdown requested. Exiting...");
    ros::shutdown();   // Ctrl 节点自毁
    res.success = true;
    return true;
}

void FlightCore::main_loop()
{
    ROS_INFO("waiting for imu");
    while(ros::ok() && !state_cb.connected)
    {
        ros::spinOnce();
    }
    ROS_INFO("fcu has connected");

    while(ros::ok())
    {
        ros::spinOnce();
        cmdloop_Callback();
        if(rate_count == 15)
        {
            rate_count = 0;
            statusloop_Callback();
        } 
        rate_count++;
        rate.sleep();
    }
}

double FlightCore::round_(double x)
{
    return std::round(x * 10.0) / 10.0;
}

geometry_msgs::PoseStamped FlightCore::getNowPose()
{
    geometry_msgs::PoseStamped out;
    out.header.frame_id = "map";
    out.header.stamp = ros::Time::now();

    // if(std::abs(reference_position_cb.pose.position.z - last_pos_z) >= 0.5) reference_position_cb.pose.position.z = last_pos_z;
    // else last_pos_z = reference_position_cb.pose.position.z;

    if (use_vins && !vins_drifted)
    {
        ROS_INFO_THROTTLE(0.5,"vins");
        out.pose = vins_position_cb.pose.pose;
        out.pose.position.z = reference_position_cb.pose.position.z;
    }
    else
    {
        ROS_INFO_THROTTLE(0.5,"ref");
        geometry_msgs::PoseStamped t;
        if(use_vins && vins_drifted) t = emergency_pos;
        else    t = reference_position_cb;

        // 1. 把 ENU 坐标 → map 坐标
        Eigen::Vector3d enu(t.pose.position.x,
                            t.pose.position.y,
                            t.pose.position.z);
        Eigen::Vector3d map = move.enu2Map_pos(enu);   // 内部就是 R*(enu-origin_pos)

        out.pose.position.x = map(0);
        out.pose.position.y = map(1);
        out.pose.position.z = map(2);

        // 2. 把 ENU yaw → map yaw
        double enu_yaw = tf2::getYaw(t.pose.orientation);
        double map_yaw = move.enu2Map_yaw(enu_yaw); // 返回角度
        // 把角度再转回四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, map_yaw * M_PI / 180.0);
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
    }

    //ROS_INFO_THROTTLE(0.5,);

    return out;
}

void FlightCore::cmdloop_Callback()
{

    switch(FT)
    {
        case Standby: cmd_Task_Standby(); break;
        case Takeoff: cmd_Task_Takeoff(TAKEOFF_HEIGHT); break;
        case Mission: cmd_Task_Mission(); break;
        case Land: cmd_Task_Land(); break;
        case Interrupt: cmd_Task_Interupt(); break;
        case Debug: cmd_Task_Debug(); break;
        case Emergency: cmd_Task_Emergency();break;
    }

    if(!land_flag)  target_velocity_pub.publish(pub_vel);
    
}

void FlightCore::statusloop_Callback()
{

    if(state_cb.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(3.0))
    {
        ROS_INFO("trying to switch to OFFBOARD");
        if(mode_change_client.call(mode_change_msg) && mode_change_msg.response.mode_sent)
        {
            ROS_INFO("Offboard Enabled");
        }
        else ROS_INFO("Offboard Failed : %d",mode_change_msg.response.mode_sent);
        last_request = ros::Time::now();
    }
    else if(!state_cb.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
        ROS_INFO("trying to switch to Arm");
        if(arming_request_client.call(arming_request_msgs) && arming_request_msgs.response.success)
        {
            ROS_INFO("Armed");
        }
        else ROS_INFO("Arm Failed");
        last_request = ros::Time::now();
    }

    // 当前 ENU 坐标
    Eigen::Vector3d curr_enu(reference_position_cb.pose.position.x,
                             reference_position_cb.pose.position.y,
                             reference_position_cb.pose.position.z);
    // 转到 Map 坐标系
    Eigen::Vector3d curr_map = move.enu2Map_pos(curr_enu);
    double curr_yaw_enu = tf2::getYaw(reference_position_cb.pose.orientation);
    double curr_yaw_map = move.enu2Map_yaw(curr_yaw_enu);

    // ROS_INFO_THROTTLE(0.1,
    //     "[STATUS] Map Pose: x=%.3f  y=%.3f  err=%.2f ",
    //     vins_position_cb.pose.pose.position.x,vins_position_cb.pose.pose.position.y,vins_err);
}

void FlightCore::cmd_Task_Standby()
{
    if(is_changed)
    {
        Eigen::Vector3d target_pos(round_(cache_pos.pose.position.x),round_(cache_pos.pose.position.y),round_(cache_pos.pose.position.z));
        double target_yaw = tf2::getYaw(cache_pos.pose.orientation) * 180.0 / M_PI;
        move.setTarget(target_pos,target_yaw);
        is_changed = false;
    }
    pub_vel = move.compute(getNowPose());
}

void FlightCore::cmd_Task_Land()
{
        is_changed = false;
        Eigen::Vector3d target_pos(round_(cache_pos.pose.position.x),round_(cache_pos.pose.position.y),round_(getNowPose().pose.position.z-0.15));
        double target_yaw = tf2::getYaw(cache_pos.pose.orientation) * 180.0 / M_PI;
        move.setTarget(target_pos,target_yaw);
        pub_vel = move.compute(getNowPose());

    if(getNowPose().pose.position.z <= 0.25 && !land_flag)
    {
        land_time = ros::Time::now();
        mode_change_msg.request.custom_mode = "POSITION";
        mode_change_client.call(mode_change_msg);
        land_flag = true;
    }

    if(ros::Time::now() - land_time >= ros::Duration(1))
    {
        // mavros_msgs::CommandBool arming_request_msgs_;
        // arming_request_msgs_.request.value = false;
        // arming_request_client.call(arming_request_msgs_);
        mode_change_msg.request.custom_mode = "AUTO.LAND";
        mode_change_client.call(mode_change_msg);
    }
}

void FlightCore::cmd_Task_Takeoff(double h)
{
    if(is_changed)
    {
        Eigen::Vector3d target_pos(round_(cache_pos.pose.position.x),round_(cache_pos.pose.position.y),round_(h));
        double target_yaw = tf2::getYaw(cache_pos.pose.orientation) * 180.0 / M_PI;
        move.setTarget(target_pos,target_yaw);
        is_changed = false;
    }
    pub_vel = move.compute(getNowPose());
}

void FlightCore::cmd_Task_Mission()
{
    if(mission.isFinished())
    {
        ROS_INFO("[MISSION] All waypoints completed");
        FT = Standby;
        cache_pos = getNowPose();
        is_changed = true;
        mission.reset();
        last_index = -1;
        return;
    }

    if(mission.getCurrentIndex() != last_index)
    {
        Eigen::Vector4d tar = mission.getCurrentWaypoint();
        Eigen::Vector3d target_pos(tar(0),tar(1),tar(2));
        double target_yaw = tar(3);
        move.setTarget(target_pos,target_yaw);
        ROS_INFO("[MISSION] Setting new target: map_pos[%.2f, %.2f, %.2f], yaw %.2f ", 
                 target_pos(0), target_pos(1), target_pos(2), target_yaw);
        last_index = mission.getCurrentIndex();
        move.setStartTime(ros::Time::now());
    }

    if(move.isArrived(getNowPose()))
    {
        mission.nextWaypoint();
        ROS_INFO("[MISSION]Reached waypoint %d, Moving to next waypoint %d", mission.getCurrentIndex(), mission.getCurrentIndex() + 1);
    }
    else
    {
        if(move.isTimeout(ros::Time::now()))
        {
            ROS_WARN("[MISSION] Timeout at waypoint %d, switching to Standby", mission.getCurrentIndex());
            // FT = Standby;
            // cache_pos = getNowPose();
            // is_changed = true;
            mission.nextWaypoint();
        }
    }

    pub_vel = move.compute(getNowPose());

    if(!mission.isFinished())
    {
        Eigen::Vector4d current_wp = mission.getCurrentWaypoint();
        ROS_INFO_THROTTLE(1.0,"[MISSION] Current waypoint index: %d, target: [%.2f, %.2f, %.2f, %.2f ]", 
             mission.getCurrentIndex(), current_wp(0), current_wp(1), current_wp(2), current_wp(3));
    }

}

void FlightCore::cmd_Task_Interupt()
{

}

void FlightCore::cmd_Task_Debug()
{
    if(is_changed)
    {
        Eigen::Vector3d target_pos(round_(cache_pos.pose.position.x),round_(cache_pos.pose.position.y),round_(cache_pos.pose.position.z));
        double target_yaw = tf2::getYaw(cache_pos.pose.orientation) * 180.0 / M_PI;
        move.setTarget(target_pos,target_yaw);
        is_changed = false;
    }
    pub_vel = move.compute(getNowPose());
}

void FlightCore::cmd_Task_Emergency()
{
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "FlightCtrlPro");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ROS_INFO("[FlightCtrlPro] Node started, entering main loop ...");
    FlightCore FlightCorer(nh,nh_private);
    FlightCorer.main_loop();
    
    return 0;
}