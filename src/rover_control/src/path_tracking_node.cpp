#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "Eigen/Geometry"

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using std::placeholders::_1;

class PathTracking : public rclcpp::Node
{
public:
    PathTracking() : Node("path_tracking_node")
    {
        // 声明预瞄距离和目标速度
        this->declare_parameter("lookahead_dist", 0.5);
        this->declare_parameter("target_speed", 0.5);

        // 获取参数
        this->get_parameter("lookahead_dist", lookahead_dist_);
        this->get_parameter("target_speed", target_speed_);

        path_sub_ = this->create_subscription<Path>("/path", 10, std::bind(&PathTracking::pathCallback, this, _1));

        odom_sub_ = this->create_subscription<Odometry>("/odom", 10, std::bind(&PathTracking::odomCallback, this, _1));

        vel_pub_ = this->create_publisher<Twist>("/cmd_vel", 10);
    }

private:
    // 成员变量
    rclcpp::Subscription<Path>::SharedPtr path_sub_;
    nav_msgs::msg::Path global_path_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Twist>::SharedPtr vel_pub_;

    // 参数
    double lookahead_dist_;
    double target_speed_;

    // path 回调函数
    void pathCallback(const Path::SharedPtr msg)
    {
        global_path_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received Path with %zu points", global_path_.poses.size());
    }

    // odom 回调函数
    void odomCallback(const Odometry::SharedPtr msg)
    {
        Twist vel_msg;

        if (global_path_.poses.empty())
        {
            // RCLCPP_WARN(this->get_logger(), "全局路径无有效路径点，发布零速度");
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub_->publish(vel_msg);
            return;
        }

        // 有效路径点
        // 取出坐标
        double curr_x = msg->pose.pose.position.x;
        double curr_y = msg->pose.pose.position.y;
        // // 取出四元数
        // Eigen::Quaternion q(
        //     msg->pose.pose.orientation.w,
        //     msg->pose.pose.orientation.x,
        //     msg->pose.pose.orientation.y,
        //     msg->pose.pose.orientation.z);

        // Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
        // Eigen::Vector3d euler_angles;
        // // Z-Y-X 顺序
        // euler_angles = rotation_matrix.eulerAngles(2, 1, 0);

        // double curr_yaw = euler_angles[0];
        // 1. 取出四元数
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // 2. 手搓公式 (这是最稳的，绝对不会错)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double curr_yaw = std::atan2(siny_cosp, cosy_cosp);

        // 寻找预瞄点 (Lookahead Point)
        double goal_x = curr_x;
        double goal_y = curr_y;
        bool found_target = false;

        for (const auto &pose : global_path_.poses)
        {
            double dx = pose.pose.position.x - curr_x;
            double dy = pose.pose.position.y - curr_y;
            // hypoy 用于计算第三边的长度
            double dist = std::hypot(dx, dy);
            if (dist > lookahead_dist_)
            {
                goal_x = pose.pose.position.x;
                goal_y = pose.pose.position.y;
                found_target = true;
                break;
            }
        }

        if (!found_target)
        {
            auto last_pose = global_path_.poses.back();
            goal_x = last_pose.pose.position.x;
            goal_y = last_pose.pose.position.y;

            if (std::hypot(goal_x - curr_x, goal_y - curr_y) < 0.1)
            {
                stopCar();
                return;
            }
        }

        // ===================================================================
        // 4. Pure Pursuit 核心计算
        double angle_to_goal = std::atan2(goal_y - curr_y, goal_x - curr_x);
        double alpha = angle_to_goal - curr_yaw;

        // 角度归一化 (-PI ~ PI)
        while (alpha > M_PI)
            alpha -= 2.0 * M_PI;
        while (alpha < -M_PI)
            alpha += 2.0 * M_PI;

        double angular_z = (2.0 * target_speed_ * std::sin(alpha)) / lookahead_dist_;

        // 5. 发布指令
        vel_msg.linear.x = target_speed_;
        vel_msg.angular.z = angular_z;
        vel_pub_->publish(vel_msg);
        // ===================================================================
    }

    void stopCar()
    {
        Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_->publish(cmd_vel);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTracking>());
    rclcpp::shutdown();
    return 0;
}