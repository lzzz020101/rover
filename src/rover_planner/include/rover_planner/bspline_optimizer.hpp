#ifndef ROVER_PLANNER_BSPLINE_HPP_
#define ROVER_PLANNER_BSPLINE_HPP_

#include <vector>
#include <Eigen/Dense>

namespace rover_planner
{
    class BsplineOptimizer
    {
    public:
        BsplineOptimizer() = default;
        ~BsplineOptimizer() = default;

        /*
         * @brief B样条优化
         * @param raw_path A* 输出的原始路径点
         * @param smooth_path 输出的平滑路径点
         * @param density 插值密度, 默认20
         * @return true 成功 false 失败
         */
        bool optimize(const std::vector<Eigen::Vector2d> &raw_path, std::vector<Eigen::Vector2d> &smooth_path, int density = 20);

    private:
        // 3阶 B 样条的基函数矩阵
        // M = 1/6 * [ -1  3 -3  1 ]
        //           [  3 -6  3  0 ]
        //           [ -3  0  3  0 ]
        //           [  1  4  1  0 ]
        Eigen::Matrix4d base_matrix_;
    };
}; // namespace rover_planner

#endif // ROVER_PLANNER_BSPLINE_HPP_