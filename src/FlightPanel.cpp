#include "FlightPanel.hpp"
#include <flight_ctrl/SetDebugTarget.h>   // 确保包含

FlightPanel::FlightPanel() : running_(true)
{
    set_mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client_     = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_task_client_   = nh_.serviceClient<flight_ctrl::SetFlightTask>("FlightCtrl/SetFlightTask");
    set_trigger_client_= nh_.serviceClient<flight_ctrl::TriggerShutdown>("FlightPanel/RequestShutdown");
    set_debug_target_client_ = nh_.serviceClient<flight_ctrl::SetDebugTarget>("FlightCtrl/SetDebugTarget");

    kb_thread_ = std::thread(&FlightPanel::kb_loop, this);
}

FlightPanel::~FlightPanel()
{
    shutdown();
}

void FlightPanel::shutdown()
{
    running_ = false;
    if (kb_thread_.joinable()) kb_thread_.join();
    ros::shutdown();
}

void FlightPanel::run()
{
    print_menu();
    while (ros::ok() && running_)
    {
        if (shutdown_requested_)
        {
            flight_ctrl::TriggerShutdown srv;
            set_trigger_client_.call(srv);   // 不再关心返回值
            ROS_INFO("Shutdown request sent. Ctrl & Panel both exiting.");
            break;
        }
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void FlightPanel::print_menu()
{
    system("clear");
    std::cout << "╔════════════════════════════════════╗\n";
    std::cout << "║         FlightPanel Terminal       ║\n";
    std::cout << "╠════════════════════════════════════╣\n";
    std::cout << "║  [o] OFFBOARD                      ║\n";
    std::cout << "║  [a] ARM                           ║\n";
    std::cout << "║  [z] DISARM                        ║\n";
    std::cout << "║  [1] Standby                       ║\n";
    std::cout << "║  [2] Takeoff                       ║\n";
    std::cout << "║  [3] Mission                       ║\n";
    std::cout << "║  [4] Land                          ║\n";
    std::cout << "║  [d] Debug (按 e 退出)             ║\n";
    std::cout << "║  [p] POSITION                      ║\n";
    std::cout << "║  [l] LAND                          ║\n";    
    std::cout << "║  [q] Quit (graceful)               ║\n";
    std::cout << "╚════════════════════════════════════╝\n";
    std::cout << "请输入命令: " << std::flush;
}

int FlightPanel::getch_nonblock()
{
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
    return ch;
}

void FlightPanel::kb_loop()
{
    while (running_)
    {
        int ch = getch_nonblock();
        if (ch == -1) continue;

        switch (ch)
        {
            case 'o': set_mode("OFFBOARD"); break;
            case 'a': set_arm(true);        break;
            case 'z': set_arm(false);       break;
            case '1': set_flight_task("Standby"); break;
            case '2': set_flight_task("Takeoff"); break;
            case '3': set_flight_task("Mission"); break;
            case '4': set_flight_task("Land");    break;
            case 'd': set_flight_task("Debug"); debug_interactive(); break;
            case 'p': set_mode("POSITION"); break;
            case 'l': set_mode("AUTO.LAND"); break;
            case 'q': shutdown_requested_ = true; break;
            default: break;
        }
    }
}

void FlightPanel::set_mode(const std::string& mode)
{
    mavros_msgs::SetMode srv; srv.request.custom_mode = mode;
    if (set_mode_client_.call(srv) && srv.response.mode_sent)
        ROS_INFO("Mode set to %s", mode.c_str());
    else
        ROS_WARN("Failed to set mode %s", mode.c_str());
}

void FlightPanel::set_arm(bool arm)
{
    mavros_msgs::CommandBool srv; srv.request.value = arm;
    if (arming_client_.call(srv) && srv.response.success)
        ROS_INFO("Arming: %s", arm ? "ARMED" : "DISARMED");
    else
        ROS_WARN("Arming call failed");
}

void FlightPanel::set_flight_task(const std::string& task)
{
    flight_ctrl::SetFlightTask srv; srv.request.value = task;
    if (set_task_client_.call(srv) && srv.response.success)
        ROS_INFO("Flight task set to %s", task.c_str());
    else
        ROS_WARN("Failed to set flight task %s", task.c_str());
}

void FlightPanel::debug_interactive()
{
    std::cout << "\n>>> DEBUG 模式 <<<\n";
    std::cout << "输入 x y z yaw  例: 1 2 1.5 90\n输入 e 退出 DEBUG\n";
    std::string line;
    while (running_ && std::getline(std::cin, line))
    {
        if (line == "e")
        {
            set_flight_task("Standby");
            std::cout << "已离开 DEBUG，回到 Standby。\n";
            return;
        }

        std::istringstream iss(line);
        double x, y, z, yaw;
        if (!(iss >> x >> y >> z >> yaw))
        {
            std::cout << "格式错误！重试\n";
            continue;
        }

        flight_ctrl::SetDebugTarget srv;
        srv.request.x = x; srv.request.y = y; srv.request.z = z; srv.request.yaw = yaw;
        if (set_debug_target_client_.call(srv))
            ROS_INFO("Debug target sent: [%.2f, %.2f, %.2f, %.1f]", x, y, z, yaw);
        else
            ROS_WARN("Send debug target failed");

        std::cout << "输入下一目标 (e 退出): ";
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FlightPanel");
    FlightPanel panel;
    panel.run();
    return 0;
}