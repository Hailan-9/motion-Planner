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
#include "commonFunction.hpp"
using namespace std;
using namespace Eigen;



class MinimumSnap
{
    public:
        explicit MinimumSnap();
        virtual ~MinimumSnap();

        //设轨迹分为k段，则共有k+1个点
        //如果只考虑起点终点PVA，中间点P，中间点PVA连续，则共有4k+2个等式约束

        //！！！TOdolist 可以考虑约束住中间每个插值点的速度和位置，使其在安全范围内,可能需要贝塞尔曲线
        int k;
        //多项式次数
        int n;
        //选择最小化的阶次 2:acc 3:jerk 4:snap 
        int order;
        //插值点时间间隔
        double dt;
        //最大速度和最大加速度都是合速度和合加速度
        double max_vel_;
        double max_accel_;
        //轨迹或者路径的维数 xy或者xyz
        int path_dimension;
        //每段轨迹的时间 绝对时间 k段轨迹 k+1个时间点t0~tk
        DVec<double> segment_time;
        DVec<double> relative_segment_Time;
        //start_end_State---每一行代表一维 首先是x维度起点p-v-a值，然后是终点p-v-a值，第二行 第三行同理
        DMat<double> start_end_State;
        void SetParas(const DMat<double> &start_end_State,const int &dimension_index);
        /* P-V-A-每维度数据用一列 order代表最小化阶次 3~jerk  4~snap */
        /* 引用传参比普通传参更加高效，传参时不需要重新调用相关的构造函数。而且为了防止形参被改变，可以前面加const修饰 */
        void SolveQp(const DMat<double> &waypoint,const DMat<double> _start_end_State,const int &_order,const double &total_Time,const double &max_vel = 0.0,const double &max_acc = 0.0 );
        void ResizeQpMats();
        VecXd AllocateTime(const DMat<double> &waypoint); 
        void PublishTrajectory();     

        /* 路径点 */
        DMat<double> path;
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
        // 时间归一化矩阵，贝塞尔曲线和普通多项式曲线转换时使用
        DMat<double> time_normalization_Matrix;
        // 轨迹规划中的全局---速度、加速度上下限 一行代表一维数据
        DMat<double> dynamic_Constraints;


        // DMat<double> pos_List;
        // DMat<double> vel_List;
        // DMat<double> acc_List;
        vector<Vec3<double>> pos_List;
        vector<Vec3<double>> vel_List;
        vector<Vec3<double>> acc_List;

        Vec3<double> GetPosPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t); 
        Vec3<double> GetVelPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t); 
        Vec3<double> GetAccPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t); 


        /** @brief QpSolver类的实例化对象，二次规划求解器 */
        QpSolver* qpSolver;


};