/*********************************************************************
 *  serial.cpp  – 串口桥接节点
 *  1. 从串口收数据：解析 "msg:jfq:payload\r\n" 或 "cmd:CMD\r\n"
 *     • msg → 发布到 /serial_rx
 *     • cmd → 调用对应 Service
 *  2. 订阅 /serial_tx，把内容通过串口发出去
 *********************************************************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <flight_ctrl/SetFlightTask.h>
#include <flight_ctrl/TriggerShutdown.h> 

/*================ 前置声明 =================*/
void cbSerialTx(const std_msgs::String::ConstPtr& msg);
bool parseLine(const std::string& line,
               std::string& type,
               std::string& payload);

/*================ 全局对象 =================*/
serial::Serial ser;                    // 串口对象
std::string    rx_buffer;              // 字节流缓存

/*================ ROS 通信句柄 =================*/
ros::ServiceClient set_mode_client_;
ros::ServiceClient arming_client_;
ros::ServiceClient set_task_client_;
ros::ServiceClient kill_trigger_client_;
ros::ServiceClient go_trigger_client_;
ros::Publisher     pub_rx;
ros::Subscriber    sub_tx;

/*================ 业务函数 =================*/
void set_mode(const std::string& mode)
{
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;
    if (set_mode_client_.call(srv) && srv.response.mode_sent)
        ROS_INFO("Mode set to %s", mode.c_str());
    else
        ROS_WARN("Failed to set mode %s", mode.c_str());
}

void set_arm(bool arm)
{
    mavros_msgs::CommandBool srv;
    srv.request.value = arm;
    if (arming_client_.call(srv) && srv.response.success)
        ROS_INFO("Arming: %s", arm ? "ARMED" : "DISARMED");
    else
        ROS_WARN("Arming call failed");
}

void set_flight_task(const std::string& task)
{
    flight_ctrl::SetFlightTask srv;
    srv.request.value = task;
    if (set_task_client_.call(srv) && srv.response.success)
        ROS_INFO("Flight task set to %s", task.c_str());
    else
        ROS_WARN("Failed to set flight task %s", task.c_str());
}

/*================ 串口发送回调 =================*/
void cbSerialTx(const std_msgs::String::ConstPtr& msg)
{
    if (ser.isOpen())
    {
        ser.write(msg->data);
        // ser.write("\r\n");   // 可加可不加，视协议
    }
}

/*================ 帧解析 =================*/
bool parseLine(const std::string& line,
               std::string& type,
               std::string& payload)
{
    size_t colon = line.find(':');
    if (colon == std::string::npos) return false;
    type    = line.substr(0, colon);
    payload = line.substr(colon + 1);
    return true;
}

/*================ main =================*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_bridge_node");
    ros::NodeHandle nh;

    /* 1. 打开串口 */
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    ser.setTimeout(to);
    ser.setBytesize(serial::eightbits);    // 8位数据位
    ser.setParity(serial::parity_none);    // 无校验
    ser.setStopbits(serial::stopbits_one); // 1位停止位
    ser.setFlowcontrol(serial::flowcontrol_none); // 禁用硬件流控

    try { ser.open(); }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open port /dev/ttyUSB0");
        return -1;
    }
    if (!ser.isOpen()) return -1;
    ROS_INFO("/dev/ttyUSB0 opened");

    /* 2. ROS 通信 */
    set_mode_client_  = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client_    = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_task_client_  = nh.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");
    kill_trigger_client_ = nh.serviceClient<flight_ctrl::TriggerShutdown>("FlightPanel/RequestShutdown");
    go_trigger_client_ = nh.serviceClient<flight_ctrl::TriggerShutdown>("FlightPanel/GoTrigger");
    pub_rx            = nh.advertise<std_msgs::String>("Serial/RX", 100);
    sub_tx            = nh.subscribe("Serial/TX", 100, cbSerialTx);

    /* 3. 启动异步回调线程，确保 ServiceClient 可用 */
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* 4. 主循环：收数据 + 解析 + 执行 */
    ros::Rate loop_rate(100);   // 100 Hz
    while (ros::ok())
    {
        if (ser.available())
        {
            uint8_t buf[1024];
            size_t n = ser.read(buf, ser.available());
            // std::stringstream ss;
            // for (size_t i = 0; i < n; ++i)
            // {
            //     ss << std::hex << std::setw(2) << std::setfill('0') 
            //     << static_cast<int>(buf[i]) << " ";
            // }
            // ROS_INFO("Raw RX (%zu bytes): %s", n, ss.str().c_str());
            rx_buffer.append(reinterpret_cast<char*>(buf), n);
        }

        size_t pos;
        while ((pos = rx_buffer.find("#")) != std::string::npos)
        {
            std::string line = rx_buffer.substr(0, pos);
            ROS_INFO_STREAM(line);
            rx_buffer.erase(0, pos + 1);

            std::string type, payload;
            if (parseLine(line, type, payload))
            {
                if (type == "msg")
                {
                    std_msgs::String msg;
                    msg.data = payload;
                    pub_rx.publish(msg);
                }
                else if (type == "cmd")
                {
                    if      (payload == "o") set_mode("OFFBOARD");
                    else if (payload == "a")      set_arm(true);
                    else if (payload == "d")   set_arm(false);
                    else if (payload == "s")  set_flight_task("Standby");
                    else if (payload == "t")  set_flight_task("Takeoff");
                    else if (payload == "m")  set_flight_task("Mission");
                    else if (payload == "l")     set_flight_task("Land");
                    else if (payload == "M")     set_flight_task("M_Land");
                    else if (payload == "P")     set_mode("POSITION");
                    else if (payload == "L")     set_mode("AUTO.LAND");
                    else if (payload == "G")   
                    {
                        flight_ctrl::TriggerShutdown srv;
                        go_trigger_client_.call(srv);
                        ROS_INFO("GOOOOO!!");
                    }
                    else if (payload == "q")
                    {
                        flight_ctrl::TriggerShutdown srv;
                        kill_trigger_client_.call(srv);
                        ROS_INFO("Shutdown request sent. Ctrl & Panel both exiting.");
                    }
                }
            }
        }
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}