#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// 全局变量，用于存储节点和发布者的指针
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;

// 回调函数，用于处理导航结果消息
void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    // 打印接收到的消息内容
    RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());

    // 使用静态变量跟踪到达次数
    static int arrival_count = 0;
    
    if(msg->data == "navi done")
    {
        arrival_count++;
        RCLCPP_INFO(node->get_logger(), "Arrived ! Count: %d", arrival_count);

        // 使用静态定时器指针确保定时器生命周期
        static rclcpp::TimerBase::SharedPtr timer;
        
        // 根据到达次数设置不同的目标点和延迟
        if (arrival_count == 1) { // 到达目标点1
            // 设置8秒定时器导航到目标点2
            timer = node->create_wall_timer(
                std::chrono::seconds(8),
                [](void) {
                    std_msgs::msg::String waypoint_msg;
                    waypoint_msg.data = "2";
                    navigation_pub->publish(waypoint_msg);
                    RCLCPP_INFO(node->get_logger(), "Delayed 8 seconds. Navigating to waypoint 2...");
                }
            );
        } 
        else if (arrival_count == 2) { // 到达目标点2
            timer->cancel();  // 取消之前的定时器
            // 设置8秒定时器导航到目标点3
            timer = node->create_wall_timer(
                std::chrono::seconds(8),
                [](void) {
                    std_msgs::msg::String waypoint_msg;
                    waypoint_msg.data = "3";
                    navigation_pub->publish(waypoint_msg);
                    RCLCPP_INFO(node->get_logger(), "Delayed 8 seconds. Navigating to waypoint 3...");
                }
            );
        }
        else if (arrival_count == 3) { // 到达目标点3
            timer->cancel();
            // 设置8秒定时器导航到目标点4
            timer = node->create_wall_timer(
                std::chrono::seconds(8),
                [](void) {
                    std_msgs::msg::String waypoint_msg;
                    waypoint_msg.data = "4";
                    navigation_pub->publish(waypoint_msg);
                    RCLCPP_INFO(node->get_logger(), "Delayed 8 seconds. Navigating to waypoint 4...");
                }
            );
        }
        else if (arrival_count == 4) { // 到达目标点4
            timer->cancel();
            // 设置8秒定时器导航到目标点5
            timer = node->create_wall_timer(
                std::chrono::seconds(8),
                [](void) {
                    std_msgs::msg::String waypoint_msg;
                    waypoint_msg.data = "5";
                    navigation_pub->publish(waypoint_msg);
                    RCLCPP_INFO(node->get_logger(), "Delayed 8 seconds. Navigating to waypoint 5...");
                }
            );
        }
        else if (arrival_count == 5) { // 到达目标点5
            timer->cancel();
            // 设置30秒定时器导航回目标点1
            timer = node->create_wall_timer(
                std::chrono::seconds(30),
                [](void) {
                    std_msgs::msg::String waypoint_msg;
                    waypoint_msg.data = "1";
                    navigation_pub->publish(waypoint_msg);
                    RCLCPP_INFO(node->get_logger(), "Delayed 30 seconds. Cycling back to waypoint 1...");
                    arrival_count = 0;  // 重置计数器
                }
            );
        }
    }
}

int main(int argc, char** argv)
{
    // 初始化 ROS 2 客户端库
    rclcpp::init(argc, argv);

    // 创建一个名为 "waypoint_navigation_node" 的节点
    node = std::make_shared<rclcpp::Node>("waypoint_navigation_node");

    // 创建发布者到全局变量
    navigation_pub = node->create_publisher<std_msgs::msg::String>(
        "/waterplus/navi_waypoint", 10);

    // 创建订阅者
    auto result_sub = node->create_subscription<std_msgs::msg::String>(
        "/waterplus/navi_result", 10, ResultCallback);

    // 暂停 1000 毫秒，确保订阅者和发布者都已初始化
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // 初始化导航到第一个目标点
    std_msgs::msg::String waypoint_msg;
    waypoint_msg.data = "1";  // 设置第一个目标点
    navigation_pub->publish(waypoint_msg);
    RCLCPP_INFO(node->get_logger(), "Initial navigation to waypoint 1...");

    // 启动节点的事件循环
    rclcpp::spin(node);
    
    // 关闭 ROS 2 客户端库
    rclcpp::shutdown();
    return 0;
}