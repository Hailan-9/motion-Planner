/**
 * @copyright Copyright (c) 2023{
 * }..
 * 
 * @file    minimumsnap.cpp
 * @brief   描述
 * @author  hailan(2522082924@qq.com)
 * @version 0.1
 * @date    2023-07-16
 */


/**
 * @brief   描述
 * 
 * @param   waypoint    参数描述
 * @return  DVec<double> 
 * @author  hailan(2522082924@qq.com)
 * @date    2023-07-16
 */

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
https://blog.csdn.net/qq_15390133/article/details/106854420
https://blog.csdn.net/Travis_X/article/details/114435993

优点：通过路径搜索和轨迹优化的方式,算法高效，因为始终是在一个低维度的状态空间去考虑
缺点 ：不能保证优化产生路径是安全的。解决，中间插点。后面考虑安全走廊的问题。
*/

#include "minimumsnap.hpp"
#include <vector>
using namespace std;
using namespace Eigen;

/**
 * @brief   描述
 * 
 * @author  hailan(2522082924@qq.com)
 * @date    2023-07-16
 */
MinimumSnap::MinimumSnap()
{
    dt = (double)((double)((int)(0.1*1000))/1000.0);
    qpSolver = new QpSolver();
}

MinimumSnap::~MinimumSnap()
{
    delete qpSolver;
}

// 梯形时间分配方法 两个const 第一个const是不可以改变引用传递的参数 第二个代表常量成员函数，被修饰后 该函数不可以改变类中的成员变量 这个地方不能加第二个const 不然path无法被修改
// waypoint 一行代表一个维度的路径位置
// path.resize(rows,cols);这行代码之前一直出错，找不到原因，后来查找了很多资料 问了chatgpt后，发现是这个函数后面加了const 这样一来，就不能改变这个类中的任何成员变量
//  DVec<double> MinimumSnap::AllocateTime(const DMat<double> &waypoint) const //错误用法 应该去掉第二个const
/**
 * @brief   描述
 * 
 * @param   waypoint    参数描述
 * @return  DVec<double> 
 * @author  hailan(2522082924@qq.com)
 * @date    2023-07-16
 */
DVec<double> MinimumSnap::AllocateTime(const DMat<double> &waypoint)
{

    // position每维度数据用一行，一般二维xy或者三维xyz
    DVec<double> times = DVec<double>::Zero(waypoint.cols() - 1);
    const double t = max_vel_ / max_accel_;
    const double dist_threshold_1 = max_accel_ * t * t;

    double segment_t;
    for (unsigned int i = 1; i < waypoint.cols(); ++i)
    {
        double delta_dist = (waypoint.col(i) - waypoint.col(i - 1)).norm();
        if (delta_dist > dist_threshold_1)
        {
            segment_t = t * 2 + (delta_dist - dist_threshold_1) / max_vel_;
        }
        else
        {
            segment_t = std::sqrt(delta_dist / max_accel_);
        }

        times[i - 1] = segment_t;
    }
    // 临时测试
    times(0) = 1.0;
    times(1) = 2.0;
    times(2) = 2.0;


    return times;
}

void MinimumSnap::ResizeQpMats()
{
    decision_variables.resize(k * (n + 1), 1);

    qp_H.resize(k * (n + 1), k * (n + 1));
    qp_H.setZero();

    qp_G.resize(k * (n + 1), 1);
    qp_G.setZero();

    qp_A.resize(2*order + (k-1)*(order+1), k * (n + 1));
    qp_A.setZero();

    qp_L.resize(2*order + (k-1)*(order+1), 1);
    qp_L.setZero();

    qp_U.resize(2*order + (k-1)*(order+1), 1);
    qp_U.setZero();
    // 一般情况给定起点-终点的P-V-A-jerk值,高于ACC后的阶次一般都是给定0
    start_end_State.resize(path_dimension, 2*order);
    start_end_State.setZero();

    pos_List.clear();
    vel_List.clear();
    acc_List.clear();
}

void MinimumSnap::SetParas(const DMat<double> &start_end_State, const int &dimension_index)
{
    //每段时间，使用绝对时间，下面先求出每段的相对时间，然后转化为绝对时间
    DVec<double> relative_segment_Time = AllocateTime(path);
    // k段轨迹，有k个时间段，k+1个时间点 t0 = 0.0
    segment_time.resize(relative_segment_Time.size() + 1, 1);
    segment_time.setZero();

    for (unsigned int i = 1; i <= relative_segment_Time.size(); i++)
    {
        for (unsigned int j = 0; j < i; j++)
        {
            segment_time[i] += relative_segment_Time[j];
        }
    }
    cout << "segment_time--------------" << endl;
    for (int i(0); i < segment_time.size(); i++)
        cout << segment_time[i] << endl;


    /* 构建等式约束 */
    /* step1:构建第一组约束：起点终点PVA */
    DMat<double> Sub_A_start;
    Sub_A_start.resize(order, n + 1);
    Sub_A_start.setZero();

    DMat<double> Sub_A_end;
    Sub_A_end.resize(order, n + 1);
    Sub_A_end.setZero();

    for (unsigned int i = 0; i < order; i++)
    {
        for (unsigned int j = i; j < n + 1; j++)
        {
            Sub_A_start.block(i, j, 1, 1) << pow(segment_time[0], j - i) * Factorial(j) / Factorial(j - i);
        }
    }
    cout << "sub-----------" << endl;
    cout << Sub_A_start << endl;
    for (unsigned int i = 0; i < order; i++)
    {
        for (unsigned int j = i; j < n + 1; j++)
        {
            Sub_A_end.block(i, j, 1, 1) << pow(segment_time[k], j - i) * Factorial(j) / Factorial(j - i);
        }
    }
    cout << "sub-----------" << endl;
    cout << Sub_A_end << endl;
    qp_A.block(0, 0, order, n + 1) = Sub_A_start;
    qp_A.block(2*order + (k-1)*(order+1) - order, (k - 1) * (n + 1), order, n + 1) = Sub_A_end;


    /* step2:构建第二组约束：  一共k-1个中间点~~~中间点位置-速度-加速度~连续 中间点位置, */
    for (unsigned int i = 0; i < k - 1; i++)
    {
        DMat<double> Sub_Aa(order, n + 1);
        Sub_Aa.setZero();
        DMat<double> Sub_Ab(order, n + 1);
        Sub_Ab.setZero();
        for (unsigned int l = 0; l < order; l++)
        {
            for (unsigned int j = l; j < n + 1; j++)
            {
                Sub_Aa(l, j) = pow(segment_time[i + 1], j - l) * Factorial(j) / Factorial(j - l);
                Sub_Ab(l, j) = -Sub_Aa(l, j);
            }
        }
        qp_A.block(order + i * (order+1), i * (n + 1), order, n + 1) = Sub_Aa;
        qp_A.block(order + i * (order+1), (i + 1) * (n + 1), order, n + 1) = Sub_Ab;
        qp_A.block(order + i * (order+1) + order, (i + 1) * (n + 1), 1, n + 1) = -Sub_Ab.block(0, 0, 1, n + 1);
    }

    // 构建约束的上下界，本算法中，约束为等式约束，故上下界相等
    qp_L.block(0, 0, order, 1) = start_end_State.block(dimension_index, 0, 1, order).transpose();
    qp_L.block(order + (order+1) * (k - 1), 0, order, 1) = start_end_State.block(dimension_index, order, 1, order).transpose();

    for (unsigned int i = 0; i < k - 1; i++)
    {
        DVec<double> tempVec3 = DVec<double>::Zero(order, 1);
        qp_L.block(order + i * (order+1), 0, order, 1) = tempVec3;
        qp_L.block(order + i * (order+1) + order, 0, 1, 1) = path.block(dimension_index, i + 1, 1, 1);
    }
    qp_U = qp_L;
    cout << "------------------" << endl;
    cout << start_end_State << endl
         << endl;
    cout << qp_U << endl;
    cout << "------------------" << endl;


    // 构建Q矩阵 多项式系数按照p0……pn顺序
    DMat<double> Q = DMat<double>::Zero(k * (n + 1), k * (n + 1));
    cout << "k_num " << k << endl;
    for (unsigned int i = 1; i < k + 1; i++)
    {
        DMat<double> Sub_Q(n + 1, n + 1);
        Sub_Q.setZero();
        // r c分别为行和列索引
        for (unsigned int r = 0; r <= n; r++)
        {
            for (unsigned int c = 0; c <= n; c++)
            {
                if (r < order || c < order)
                {
                    continue;
                }
                Sub_Q(r, c) = ( (Factorial(c) * Factorial(r) ) /
                              (Factorial(r - order) * Factorial(c - order)) ) *
                              (pow(segment_time[i], r + c - 2 * order + 1) - pow(segment_time[i - 1], r + c - 2 * order + 1)) /
                              (double)(r - order + c - order + 1);
            }
        }
        unsigned int row = (i - 1) * (n + 1);
        Q.block(row, row, n + 1, n + 1) = Sub_Q;
    }
    qp_H = Q;
}

// 轨迹求解时，一维一维地求解，最后合成两维或者三维的轨迹
void MinimumSnap::SolveQp(const DMat<double> &waypoint, const DMat<double> _start_end_State, const int &_order, const double &total_Time, const double &max_vel, const double &max_acc)
{
    order = _order;
    n = 2 * order - 1;
    cout <<"nnnnnnnnnnnnnnnnnnn       "<<n<<endl;
    k = waypoint.cols() - 1;
    max_vel_ = max_vel;
    max_accel_ = max_acc;

    int rows = waypoint.rows();
    int cols = waypoint.cols();

    path.resize(rows, cols);
    path = waypoint;

    path_dimension = waypoint.rows();
    // 每一行代表一个维度的所有段的多项式系数 xy 或者 xyz
    coeff_All.resize(path_dimension, k * (n + 1));
    for (int i = 0; i < path_dimension; i++)
    {
        ResizeQpMats();
        start_end_State = _start_end_State;

        SetParas(start_end_State, i);

        decision_variables = qpSolver->Solve(qp_H, qp_G, qp_A, qp_L, qp_U);
        cout << "decision " << endl
             << decision_variables << endl;
        coeff_All.block(i, 0, 1, k * (n + 1)) = decision_variables.transpose();
    }

}

void MinimumSnap::PublishTrajectory()
{

    Vec3<double> temp_position;
    temp_position.setZero();
    cout<<"dddddddddd "<<dt<<endl;

    for (unsigned int i = 0; i < segment_time.size() - 1; ++i)
    {
    cout<<"dddddddddd "<<segment_time[i]<<endl;
    cout<<"dddddddddd "<<segment_time[i+1]<<endl;


        for (double t = segment_time[i]; t < segment_time[i + 1];)
        {
            cout<<t<<" ";
            temp_position = GetPosPolynomial(coeff_All, i, t);
            pos_List.push_back(temp_position);

            temp_position = GetVelPolynomial(coeff_All, i, t);
            vel_List.push_back(temp_position);

            temp_position = GetAccPolynomial(coeff_All, i, t);
            acc_List.push_back(temp_position);
            t += dt;
            t = (double)t;
        }
        // //将终点也存入容器中
        // if(i == segment_time.size() - 2)
        // {
        //     temp_position = GetPosPolynomial(coeff_All, i, segment_time[i + 1]);
        //     pos_List.push_back(temp_position);

        //     temp_position = GetVelPolynomial(coeff_All, i, segment_time[i + 1]);
        //     vel_List.push_back(temp_position);

        //     temp_position = GetAccPolynomial(coeff_All, i, segment_time[i + 1]);
        //     acc_List.push_back(temp_position);
        // }
    }
}

// 全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3<double> MinimumSnap::GetPosPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t)
{
    Vec3<double> position;
    const unsigned int num_poly_coeff = 2 * order;
    for (unsigned int dim = 0; dim < poly_coeff_mat.rows(); ++dim)
    {
        DVec<double> coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        // 默认都是列向量
        DVec<double> time = DVec<double>::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i)
        {
            if (i == 0)
            {
                time(i) = 1.0;
            }
            else
            {
                time(i) = pow(t, i);
            }
        }

        double temp_position = 0.0;
        temp_position = coeff.transpose() * time;
        position(dim) = temp_position;
    }

    return position;
}

// 全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3<double> MinimumSnap::GetVelPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t)
{
    Vec3<double> vel;
    const unsigned int num_poly_coeff = 2 * order;
    for (unsigned int dim = 0; dim < 3u; ++dim)
    {
        DVec<double> coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        DVec<double> time = DVec<double>::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i)
        {
            if (i < 1)
            {
                time(i) = 0;
            }
            else
            {
                time(i) = pow(t, i - 1) * Factorial(i) / Factorial(i - 1);
            }
        }

        double temp_vel = 0.0;
        temp_vel = coeff.transpose() * time;
        vel(dim) = temp_vel;
    }

    return vel;
}

// 全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3<double> MinimumSnap::GetAccPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t)
{
    Vec3<double> acc;
    const unsigned int num_poly_coeff = 2 * order;
    // 第一层循环，3是指3维，xyz
    for (unsigned int dim = 0; dim < 3u; ++dim)
    {
        DVec<double> coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        DVec<double> time = DVec<double>::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i)
        {
            if (i < 2)
            {
                time(i) = 0.0;
            }
            else
            {
                time(i) = pow(t, i - 2) * Factorial(i) / Factorial(i - 2);
            }
        }

        double temp_acc = 0.0;
        temp_acc = coeff.transpose() * time;
        acc(dim) = temp_acc;
    }

    return acc;
}
