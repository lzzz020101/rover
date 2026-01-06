#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "Eigen/Dense"
#include "rover_planner/astar.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;

class RoverPlanner : public rclcpp::Node
{
public:
    RoverPlanner() : Node("rover_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "rover planner node is ready!");

        astar_.setOccupancyCallback([this](const Eigen::Vector2i &idx) -> bool
                                    { return this->isOccupied(idx); });

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        map_sub_ = this->create_subscription<OccupancyGrid>("/map", qos, std::bind(&RoverPlanner::mapCallback, this, _1));

        odom_sub_ = this->create_subscription<Odometry>("/odom", 10, std::bind(&RoverPlanner::odomCallback, this, _1));

        goal_sub_ = this->create_subscription<PoseStamped>("/goal_pose", 10, std::bind(&RoverPlanner::goalCallback, this, _1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }

private:
    rover_planner::AStar astar_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
    bool map_received_ = false;
    OccupancyGrid stored_map_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    bool odom_received_ = false;
    Eigen::Vector2d current_pose_;
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // map 回调函数
    void mapCallback(const OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Map Received!");
        stored_map_ = *msg;
        map_received_ = true;

        // 提取分辨率、原点、尺寸
        double res = msg->info.resolution;

        geometry_msgs::msg::Pose origin_pose = msg->info.origin;
        Eigen::Vector2d origin_2d;
        origin_2d.x() = origin_pose.position.x;
        origin_2d.y() = origin_pose.position.y;

        Eigen::Vector2i map_size;
        map_size.x() = msg->info.width;
        map_size.y() = msg->info.height;

        // 初始化地图信息
        astar_.initMap(res, map_size, origin_2d);
    }

    // odom 回调函数
    void odomCallback(const Odometry::SharedPtr msg)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for Path... (Global Path is empty)");
        // RCLCPP_INFO(this->get_logger(), "Odometry Received!");
        current_pose_.x() = msg->pose.pose.position.x;
        current_pose_.y() = msg->pose.pose.position.y;
        odom_received_ = true;
    }

    // goal 回调函数
    void goalCallback(const PoseStamped::SharedPtr msg)
    {
        if ((!map_received_) || (!odom_received_))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Map or Odom..");
            return;
        }
        Eigen::Vector2d goal_pose(msg->pose.position.x, msg->pose.position.y);
        RCLCPP_INFO(this->get_logger(), "New Goal Received!");

        // A* 搜索
        if (astar_.search(current_pose_, goal_pose))
        {
            auto path_points = astar_.getPath();
            publishPath(path_points);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Failed to find path!");
        }
    }

    // 发布路径
    void publishPath(std::vector<Eigen::Vector2d> &points)
    {
        // 创建对象
        nav_msgs::msg::Path path_msg;
        // 表头
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (const auto &pt : points)
        {
            PoseStamped pose;
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published Path with %zu points", points.size());
    }

    // 障碍物判断
    bool isOccupied(const Eigen::Vector2i &idx)
    {
        if (!map_received_)
            return true;

        int width = stored_map_.info.width;
        int height = stored_map_.info.height;
        double resolution = stored_map_.info.resolution;

        // 1. 定义膨胀半径 (安全距离)
        double robot_radius = 0.35;

        // 计算要检查多少个格子 (半径 / 分辨率)
        int inflation_grid = std::ceil(robot_radius / resolution);

        // 2. 遍历周围的格子
        for (int x_offset = -inflation_grid; x_offset <= inflation_grid; x_offset++)
        {
            for (int y_offset = -inflation_grid; y_offset <= inflation_grid; y_offset++)
            {
                int check_x = idx.x() + x_offset;
                int check_y = idx.y() + y_offset;

                // 越界检查
                if (check_x < 0 || check_x >= width || check_y < 0 || check_y >= height)
                {
                    return true;
                }

                // 查表
                int index = check_y * width + check_x;
                int8_t data = stored_map_.data[index];

                // 只要膨胀范围内有一个障碍物，中心点就不能走！
                if (data > 50 || data == -1)
                {
                    return true;
                }
            }
        }

        return false; // 周围一圈都安全，可以通过
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPlanner>());
    rclcpp::shutdown();
    return 0;
}