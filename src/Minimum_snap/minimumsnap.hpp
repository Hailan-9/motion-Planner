/*
求解步骤：
    1.构建优化函数
    2.构建约束，等式约束和不等式约束
    3.上优化器求解，osqp库
*/

/*
引用深蓝学院《移动机器人运动规划》课程的结论：
• Minimum jerk：最小化角速度，有利于视觉跟踪
• Minimum snap：最小化差速推力，节省能源
*/
#pragma once

#include <iostream>
#include "cppTypes.h"
#include <math.h>
#include "math_function.hpp"
#include "qpSolver.hpp"
using namespace std;
using namespace Eigen;



class MinimumSnap
{
    public:
        explicit MinimumSnap();
        virtual ~MinimumSnap();

        //设轨迹分为k段，共有k+1个中间点
        //如果只考虑起点终点PVA，中间点P，中间点PVA连续，则共有4k+2个等式约束
        //TOdolist 可以考虑约束住中间没个插值点的速度和位置，使其在安全范围内
        int k;
        //多项式次数
        int n;
        //选择最小化的阶次 2:acc 3:jerk 4:snap 
        int order;
        //最大速度和最大加速度都是合速度和合加速度
        double max_vel_;
        double max_accel_;
        VecXd segment_time;

        void SetParas();
        /* 每维度数据用一列 order代表最小化阶次 3~jerk  4~snap */
        void SolveQp(const DMat<double> &waypoint,const int &_order,const double &total_Time,const double &max_vel = 0.0,const double &max_acc = 0.0 );
        void ResizeQpMats();
        VecXd AllocateTime(const DMat &waypoint);
        void PublishTrajectory()     

        /* 路径点 */
        DMat path;
        /** @brief 二次规划目标函数中二次项系数矩阵 */
        DMat<double> qp_H;
        /** @brief 二次规划目标函数中一次项系数矩阵 */
        DVec<double> qp_G;
        /** @brief 二次规划中不等式约束矩阵 */
        DMat<double> qp_A;
        /** @brief 二次规划中不等式约束上界 */
        DVec<double> qp_U;
        /** @brief 二次规划中不等式约束下界 */
	    DVec<double> qp_L;
        /* 被求解的变量，也就是决策变量 */
        DVec<double> decision_variables;
        //每一行代表一个维度的所有段的多项式系数
        DMat<double> coeff_All;

        // DMat<double> pos_List;
        // DMat<double> vel_List;
        // DMat<double> acc_List;
        vector<Vec3<double>> pos_List;
        vector<Vec3<double>> vel_List;
        vector<Vec3<double>> acc_List;

        Vec3d GetPosPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t); 
        Vec3d GetVelPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t); 
        Vec3d GetAccPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t); 



        /** @brief QpSolver类的实例化对象，二次规划求解器 */
        QpSolver* qpSolver;


}