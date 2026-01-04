#ifndef ROVER_PLANNER_ASTAR_HPP_
#define ROVER_PLANNER_ASTER_HPP_

#include <memory>
#include <Eigen/Dense>
#include <functional>
#include <queue>

namespace rover_planner
{

#define IN_OPEN_SET 'a'
#define IN_CLOSE_SET 'b'
#define NOT_EXPEND 'c'

    // 1. 定义搜索节点
    struct Node
    {
        using Ptr = std::shared_ptr<Node>;
        Eigen::Vector2i index;    // 栅格索引
        Eigen::Vector2d position; // 物理坐标

        double g_score; // 当前代价 g
        double f_score; // 总代价 f = g + h

        Ptr parent; // 父类指针，回溯

        char state; // 节点状态 open-a,close-b,not_expend-c

        // 当前代价代表从起始点到当前节点的代价，但是开始的时候还没走，先设置为无穷大
        Node(Eigen::Vector2i idx, Eigen::Vector2d pos)
            : index(idx), position(pos),
              g_score(std::numeric_limits<double>::max()),
              f_score(std::numeric_limits<double>::max()),
              parent(nullptr), state(0) {}
    };

    // 2. 定义比较规则
    // queue 会根据定义的比较规则找到最小值（大）排在首位，但是其他位置的元素并不能保证升序或降序
    struct NodeComparator
    {
        bool operator()(const Node::Ptr &a, const Node::Ptr &b)
        {
            return a->f_score > b->f_score;
        }
    };

    // 3.AStar
    class AStar
    {
    public:
        AStar() = default;
        ~AStar() = default;

        // 初始化地图
        void initMap(double res, Eigen::Vector2i map_size, Eigen::Vector2d origin);
        // 搜索路径，成功返回1，失败返回0
        int search(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &end_pos);
        // 障碍物回调函数
        void setOccupancyCallback(std::function<bool(const Eigen::Vector2i &)> callback);
        // 回溯路径
        std::vector<Eigen::Vector2d> getPath();

    private:
        // 计算启发式代价（欧几里得距离）
        double getHeuristic(const Eigen::Vector2i &curr, const Eigen::Vector2i &target);
        // 物理坐标转栅格索引
        Eigen::Vector2i posToIndex(const Eigen::Vector2d &pos);
        // 栅格索引转物理坐标
        Eigen::Vector2d indexToPos(const Eigen::Vector2i &idx);
        // 判断是否越界
        bool isOutside(const Eigen::Vector2i &idx);

        // 物理参数
        double resolution_;
        double inv_resolution_;
        Eigen::Vector2i map_size_;
        Eigen::Vector2d origin_;
        double tie_breaker_;

        // 搜索辅助数据
        std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodeComparator> open_set_;
        std::vector<Node::Ptr> node_map_;
        Node::Ptr goal_node_ = nullptr;
        std::function<bool(const Eigen::Vector2i &)> is_occupied_;
        std::vector<Eigen::Vector2d> path_;
    };
}; // namespace rover_planner

#endif // ROVER_PLANNER_ASTAR_HPP_