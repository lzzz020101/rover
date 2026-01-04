/*
astar (路径搜索)
功能：纯算法实现。它不应该包含任何一个 ROS 2 的头文件。
核心逻辑：
维护 Open List 和 Closed List。
计算 G,H,F代价。
寻找从起点到终点的离散格子路径。
输出：一组原始的、折线状的 std::vector<Eigen::Vector2d>。
*/
#include "rover_planner/astar.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace rover_planner
{
    // 初始化地图
    void AStar::initMap(double res, Eigen::Vector2i map_size, Eigen::Vector2d origin)
    {
        resolution_ = res;
        inv_resolution_ = 1 / res;
        map_size_ = map_size;
        origin_ = origin;
        tie_breaker_ = 1.0 + 1.0 / 1000.0;
    }

    // 物理坐标转换为栅格索引
    Eigen::Vector2i AStar::posToIndex(const Eigen::Vector2d &pos)
    {
        Eigen::Vector2i idx;
        idx.x() = static_cast<int>((pos.x() - origin_.x()) * inv_resolution_);
        idx.y() = static_cast<int>((pos.y() - origin_.y()) * inv_resolution_);

        return idx;
    }

    // 栅格索引转换为物理坐标
    Eigen::Vector2d AStar::indexToPos(const Eigen::Vector2i &idx)
    {
        Eigen::Vector2d pos;
        pos.x() = origin_.x() + (idx.x() + 0.5) * resolution_;
        pos.y() = origin_.y() + (idx.y() + 0.5) * resolution_;

        return pos;
    }

    // 判断是否出界
    bool AStar::isOutside(const Eigen::Vector2i &idx)
    {
        if ((idx.x() < 0) || (idx.y() < 0) || (idx.x() >= map_size_.x()) || (idx.y() >= map_size_.y()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 计算启发式代价
    double AStar::getHeuristic(const Eigen::Vector2i &curr, const Eigen::Vector2i &target)
    {
        double h_score;
        double grid_dist = (curr - target).cast<double>().norm();
        h_score = grid_dist * resolution_ * tie_breaker_;
        return h_score;
    }

    // 判断是否是障碍物
    void AStar::setOccupancyCallback(std::function<bool(const Eigen::Vector2i &)> callback)
    {
        is_occupied_ = callback;
    }

    // 搜索函数
    int AStar::search(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &end_pos)
    {
        // 1. 清理上一次残留
        while (!open_set_.empty())
        {
            open_set_.pop();
        }
        node_map_.assign(map_size_.x() * map_size_.y(), nullptr);

        // 2. 将物理坐标转换成栅格索引
        Eigen::Vector2i start_index = posToIndex(start_pos);
        Eigen::Vector2i end_index = posToIndex(end_pos);

        // 3. 检查起点和终点的合法性
        if (isOutside(start_index) || isOutside(end_index))
        {
            std::cout << "[AStar] start or end outside map!" << std::endl;
            return 0;
        }

        // 4.创建起始节点并且放入open_set
        Node::Ptr start_node = std::make_shared<Node>(start_index, start_pos);
        start_node->g_score = 0;
        start_node->f_score = start_node->g_score + getHeuristic(start_index, end_index);
        start_node->state = 'a';

        int start_id = start_index.x() + start_index.y() * map_size_.x();

        AStar::node_map_[start_id] = start_node;
        open_set_.push(start_node);

        // 5. 主循环
        while (!open_set_.empty())
        {
            Node::Ptr curr = open_set_.top();
            open_set_.pop();

            // a. 判断当前节点是否在 close_set，防止重复处理
            if (curr->state == 'b')
                continue;
            // b. 终点判断
            if ((curr->index - end_index).norm() < 1.5)
            {
                goal_node_ = curr;
                std::cout << "[AStar] find path successfully!" << std::endl;
                return 1;
            }
            curr->state = 'b';
            // c. 邻居扩展
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0)
                        continue; // 跳过自己
                    Eigen::Vector2i neighbor_index = curr->index + Eigen::Vector2i(dx, dy);

                    if (isOutside(neighbor_index))
                        continue;

                    if (is_occupied_(neighbor_index))
                        continue;
                    int node_id = neighbor_index.x() + neighbor_index.y() * map_size_.x();

                    if (node_map_[node_id] != nullptr && node_map_[node_id]->state == 'b')
                    {
                        continue;
                    }
                    double edge_cost = (dx == 0 || dy == 0) ? resolution_ : resolution_ * 1.414;
                    double tmp_g_score = curr->g_score + edge_cost;
                    // 如果第一次访问该节点
                    if (AStar::node_map_[node_id] == nullptr)
                    {
                        Node::Ptr neighbor_node =
                            std::make_shared<Node>(
                                neighbor_index,
                                indexToPos(neighbor_index));
                        neighbor_node->parent = curr;
                        neighbor_node->g_score = tmp_g_score;
                        neighbor_node->f_score = tmp_g_score + getHeuristic(neighbor_index, end_index);
                        neighbor_node->state = 'a';

                        node_map_[node_id] = neighbor_node;
                        open_set_.push(neighbor_node);
                    }
                    // 该节点已经在openset中，但是有更好的路径
                    else
                    {
                        Node::Ptr neighbor_node = AStar::node_map_[node_id];

                        if (tmp_g_score < neighbor_node->g_score)
                        {
                            neighbor_node->g_score = tmp_g_score;
                            neighbor_node->f_score = tmp_g_score + getHeuristic(neighbor_index, end_index);
                            neighbor_node->parent = curr;
                            open_set_.push(neighbor_node);
                        }
                    }
                }
            }
        }
        std::cout << "[AStar] find path unsuccessfully!" << std::endl;
        return 0;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> AStar::getPath()
    {
        std::vector<Eigen::Vector2d> path;

        // 如果没有找到终点，或者终点节点为空，直接返回空路径
        if (goal_node_ == nullptr)
        {
            return path;
        }

        Node::Ptr curr = goal_node_;
        while (curr != nullptr)
        {
            // 将栅格索引转换为物理坐标
            path.push_back(indexToPos(curr->index));
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());
        path_ = path;
        return path;
    }

} // namespace rover_planner