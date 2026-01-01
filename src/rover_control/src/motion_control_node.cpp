#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using geometry_msgs::msg::Twist;
using std::placeholders::_1;

class MotionControlNode : public rclcpp::Node
{
public:
    explicit MotionControlNode() : Node("motion_control_node")
    {
        RCLCPP_INFO(this->get_logger(), "The motion control node is ready!");

        // 声明轮半径和轮间距两个参数
        this->declare_parameter<double>("wheel_radius", 0.09);
        this->declare_parameter<double>("wheel_separation", 0.32);
        this->get_parameter<double>("wheel_radius", wheel_radius_);
        this->get_parameter<double>("wheel_separation", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "wheel_radius:%.2fm ,wheel_separation:%.2fm", wheel_radius_, wheel_separation_);

        cmd_vel_subscriber_ = this->create_subscription<Twist>("/cmd_vel", 10, std::bind(&MotionControlNode::cmd_vel_callback, this, _1));
    }

private:
    void cmd_vel_callback(const Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // 第一步：转换到左轮和右轮的线速度
        double v_left = linear_x - (angular_z * wheel_separation_ / 2);
        double v_right = linear_x + (angular_z * wheel_separation_ / 2);

        // 第二步：转换到左轮和右轮的转速
        double omega_left_wheel = v_left / wheel_radius_;
        double omega_right_wheel = v_right / wheel_radius_;

        // 第三步：转换到电机的转速(角速度转电机RPM)
        double rpm_left = omega_left_wheel * 60 / (2 * M_PI);
        double rpm_right = omega_right_wheel * 60 / (2 * M_PI);

        // 打印验证
        RCLCPP_INFO(this->get_logger(),
                    "Cmd: [v=%.2f, w=%.2f] -> RPM Left: %.2f, RPM Right: %.2f",
                    linear_x, angular_z, rpm_left, rpm_right);
    }

    // 成员变量
    rclcpp::Subscription<Twist>::SharedPtr cmd_vel_subscriber_;
    double wheel_radius_;
    double wheel_separation_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControlNode>());
    rclcpp::shutdown();
    return 0;
}