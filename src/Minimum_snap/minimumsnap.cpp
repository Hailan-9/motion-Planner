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

#include "minimumsnap.hpp"
#include <vector>
using namespace std;
using namespace Eigen;

MinimumSnap::MinimumSnap()
{
    qpSolver = new QpSolver();
}

MinimumSnap::~MinimumSnap()
{
    delete qpSolver;
}

//梯形时间分配方法
VecXd MinimumSnap::AllocateTime(const MatXd &waypoint) const 
{

    path = waypoint;
    //每维度数据用一列
    VecXd times = VecXd::Zero(waypoint.rows() - 1);
    const double t = max_vel_ / max_accel_;
    const double dist_threshold_1 = max_accel_ * t * t;

    double segment_t;
    for (unsigned int i = 1; i < waypoint.rows(); ++i) 
    {
        double delta_dist = (waypoint.row(i) - waypoint.row(i - 1)).norm();
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

    return times;
}

void MinimumSnap::ResizeQpMats()
{
    decision_variables.resize(k*(n+1),1);

    qp_H.resize(k*(n+1),k*(n+1));
    qp_H.setZero();

    qp_G.resize(k*(n+1),1);
    qp_G.setZero();

    qp_A.resize(4*k+2,k*(n+1));
    qp_A.setZero();

    qp_L.resize(4*k+2,1);
    qp_L.setZero();

    qp_U.resize(4*k+2,1);
    qp_U.setZero();


    pos_List.clear();
    vel_List.clear();
    acc_List.clear();
}


void MinimumSnap::SetParas()
{
    //每段时间，使用绝对时间，下面先求出每段的相对时间，然后转化为绝对时间
    VecXd relative_segment_Time = AllocateTime(waypoint);
    segment_time.resieze(relative_segment_Time.size() + 1,1);
    segment_time.setZero();

    for (unsigned int i = 1; i < =relative_segment_Time.size(); i++ )
    {
        for(unsigned int j = 0; j < i; j++)
        {
            segment_time[i] += relative_segment_Time[j]; 
        }
    }


    /* 构建等式约束 */
    /* 构建第一组约束：起点终点PVA */
    MatXd Sub_A_start;
    Sub_A_start.resize(3,n+1);
    Sub_A_start.setZero();

    MatXd Sub_A_end;
    Sub_A_end.resize(3,n+1);
    Sub_A_end.setZero();

    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = i; j < n+1; j++)
        {
            Sub_A_start.block(i,j,1,1) = pow(segment_time[0],j-i)*Factorial(j)/Factorial(j-i);
        }
    }
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = i; j < n+1; j++)
        {
            Sub_A_end.block(i,j,1,1) = pow(segment_time[k],j-i)*Factorial(j)/Factorial(j-i);
        }
    }
    qp_A.block(0,0,3,n+1) = Sub_A_start;
    qp_A.block(4*k+2-3, (k-1)*(n+1), 3, n+1) = Sub_A_end;


    /* 构建第二组约束： 位置-速度-加速度~连续 中间点位置, */
    for (unsigned int i = 0; i<k-1;i++)
    {
        MatXd Sub_Aa(3,n+1);
        Sub_Aa.setZero();
        MatXd Sub_Ab(3,n+1);
        Sub_Ab.setZero();
        for(unsigned int l = 0; l<3; l++)
        {
            for (unsigned int j= l; j<n+1; j++)
            {
                Sub_Aa[l][j] = pow(segment_time[i+1],j-l)*Factorial(j)/Factorial(j-l);
                Sub_Ab[l][j] = -Sub_Aa[l][j];
            }
        }
        qp_A.block(3+i*3, i*(n+1), 3, n+1) = Sub_Aa;
        qp_A.block(3+i*3, (i+1)*(n+1), 3, n+1) = Sub_Ab;
        qp_A.block(3+(i+1)*3, (i+1)*(n+1), 1, n+1) = Sub_Ab.block(0,0,1,n+1);

    }

    //构建约束的上下界，本算法中，约束为等式约束，故上下界相等
    qp_L.block(0,0,3,1) = startPoint_State;
    qp_L.block(3+4*(k-1),0,3,1) = endPoint_State;
    for(unsigned int i = 0; i<k-1; i++)
    {
        VecXd tempVec3 = VecXd::Zero(3,1);
        qp_L.block(3+i*3, 0, 3, 1) = tempVec3;
        qp_L.block(3+(i+1)*3, 0, 1, 1) = path(i+1);
    }
    qp_U = qp_L;


    //构建Q矩阵 多项式系数按照p0……pn顺序
    MatXd Q = MatXd::Zero(k*(n+1),k*(n+1));

    for (unsigned int i = 1; i < k+1; i++)
    {
        MatXd Sub_Q(n+1,n+1);
        Sub_Q.setZero();
        //r c分别为行和列索引
        for (unsigned int r = 0; r<=n; j++)
        {
            for (unsigned int c = 0; c<=n; c++)
            {
                if(r < order || c < order)
                {
                    continue;
                }
                Sub_Q[r][c] = (Factorial(c)*Factorial(r))/
                              (Factorial(r-order)*Factorial(c-order))*
                              pow(segment_time[i],r + c - 2*order + 1) - pow(segment_time[i-1],r + c - 2*order + 1)/
                              (r - order + c - order + 1);
            }
        }
        unsigned int row = i * (n+1);
        Q.block(row,row,n+1,n+1) = Sub_Q;
    }
}


void MinimumSnap::SolveQp(const DMat<double> &waypoint,const int &_order,const double &total_Time,const double &max_vel,const double &max_acc)
{
    order = _order;
    n = 2*order - 1;
    k = waypoint.rows() - 1;
    max_vel_ = max_vel;
    max_accel_ = max_acc;

    int path_dimension = waypoint.cols();
    //每一行代表一个维度的所有段的多项式系数
    coeff_All.resize(path_dimension,k*(n+1));
    for (int i=0; i<path_dimension; i++)
    {
        ResizeQpMats();
        AllocateTime(waypoint);
        SetParas();
        decision_variables = qpSolver->Solve(qp_H, qp_G, qp_A, qp_L, qp_U);
        coeff_All.block(i, 0, 1, k*(n+1)) = decision_variables.transpose();
    }

}










void MinimumSnap::PublishTrajectory()                                       
{

    // VecXd temp_position;
    // temp_position.resize(path.cols(),1);
    // temp_position.setZero();
    Vec3d temp_position;
    // temp_position.resize(path.cols(),1);
    temp_position.setZero();

    for (unsigned int i = 0; i < segments_time.size(); ++i) 
    {
        for (double t =segments_time[i] ; t < segments_time[i+1];) 
        {
            temp_position = GetPosPolynomial(coeff_All, i, t);
            pos_List.push_back(temp_position);

            temp_position = GetVelPolynomial(coeff_All, i, t);
            vel_List.push_back(temp_position);

            temp_position = GetAccPolynomial(coeff_All, i, t);
            acc_List.push_back(temp_position);
            t += 0.005;
        }
    }
}




//全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3d MinimumSnap::GetPosPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3d position;
    const unsigned int num_poly_coeff = 2*order;
    for (unsigned int dim = 0; dim < 3u; ++dim) 
    {
        VecXd coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        //默认都是列向量
        VecXd time = VecXd::Zero(num_poly_coeff);

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
        for (unsigned int i = 0u; i < time.rows(); ++i) {
            temp_position = temp_position + coeff(i) * time(i);
        }

        position(dim) = temp_position;
    }

    return position;
}

//全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3d MinimumSnap::GetVelPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3d vel;
    const unsigned int num_poly_coeff = 2*order;
    for (unsigned int dim = 0; dim < 3u; ++dim) 
    {
        VecXd coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        VecXd time = VecXd::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i) 
        {
            if (i < 1) 
            {
                time(i) = 0;
            } 
            else 
            {
                time(i) = pow(t, i-1)*Factorial(i)/Factorial(i-1);
            }
        }

        double temp_vel = 0.0;
        for (unsigned int i = 0u; i < time.rows(); ++i) {
            temp_vel = temp_vel + coeff(i) * time(time.rows() - i - 1u);
        }

        vel(dim) = temp_vel;
    }

    return vel;
}

//全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3d MinimumSnap::GetAccPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3d acc;
    const unsigned int num_poly_coeff = 2*order;
    for (unsigned int dim = 0; dim < 3u; ++dim) 
    {
        VecXd coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        VecXd time = VecXd::Zero(num_poly_coeff);

        for (unsigned int i = 0; i < num_poly_coeff; ++i) 
        {
            if (i < 2) 
            {
                time(i) = 1.0;
            } 
            else 
            {
                time(i) = pow(t, i-2)*Factorial(i)/Factorial(i-2);
            }
        }

        double temp_acc = 0.0;
        for (unsigned int i = 0u; i < time.rows(); ++i) {
            temp_acc = temp_acc + coeff(i) * time(time.rows() - i - 1u);
        }

        acc(dim) = temp_acc;
    }

    return acc;
}

