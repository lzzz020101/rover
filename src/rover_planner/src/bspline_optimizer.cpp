/*
bspline_optimizer (轨迹平滑)
功能：把 A* 的折线变成符合物理特性的曲线。
核心逻辑：
B 样条曲线生成（基函数计算）。
轨迹平滑优化（通常涉及最小化加速度、抖动和避障代价）。
输出：参数化的轨迹曲线数据。
*/
#include "rover_planner/bspline_optimizer.hpp"

namespace rover_planner
{
    bool BsplineOptimizer::optimize(const std::vector<Eigen::Vector2d> &raw_path, std::vector<Eigen::Vector2d> &smooth_path, int density)
    {
        smooth_path.clear();

        // 1. 如果点太少了，不能做3阶 B样条（至少需要四个点）
        if (raw_path.size() < 4)
        {
            smooth_path = raw_path;
            return true;
        }

        // 2. 初始化基函数矩阵
        base_matrix_
            << -1,
            3, -3, 1,
            3, -6, 3, 0,
            -3, 0, 3, 0,
            1, 4, 1, 0;
        base_matrix_ /= 6.0;

        // 3. 遍历控制点
        // 3阶 B样条是滑窗操作，每次取 4 个点 (p0, p1, p2, p3) 计算中间的一段曲线
        // 所以循环次数是 N - 3
        for (size_t i = 0; i < raw_path.size() - 3; ++i)
        {
            // 取出 4 个控制点，构建 4x2 矩阵
            // [x0, y0]
            // [x1, y1]
            // [x2, y2]
            // [x3, y3]
            Eigen::Matrix<double, 4, 2> control_points;
            control_points.row(0) = raw_path[i];
            control_points.row(1) = raw_path[i + 1];
            control_points.row(2) = raw_path[i + 2];
            control_points.row(3) = raw_path[i + 3];

            // 4. 插值计算
            // u 从 0 到 1，步长为 1/density
            for (int j = 0; j <= density; ++j)
            {
                double u = (double)j / (double)density;
                double u2 = u * u;
                double u3 = u2 * u;

                // u_vec = [u^3, u^2, u, 1]
                Eigen::Vector4d u_vec(u3, u2, u, 1.0);

                // 核心公式： Pos = U * M * P
                // U: 时间参数, M: 基矩阵, P: 控制点
                Eigen::Vector2d point = u_vec.transpose() * base_matrix_ * control_points;

                smooth_path.push_back(point);
            }
        }
        return true;
    }
}