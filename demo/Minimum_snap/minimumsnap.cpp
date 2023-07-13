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

//梯形时间分配方法 两个const 第一个const是不可以改变引用传递的参数 第二个代表常量成员函数，被修饰后 该函数不可以改变类中的成员变量 这个地方不能加第二个const 不然path无法被修改
//waypoint 一行代表一个维度的路径位置
//path.resize(rows,cols);这行代码之前一直出错，找不到原因，后来查找了很多资料 问了chatgpt后，发现是这个函数后面加了const 这样一来，就不能改变这个类中的任何成员变量
// DVec<double> MinimumSnap::AllocateTime(const DMat<double> &waypoint) const //错误用法 应该去掉第二个const
DVec<double> MinimumSnap::AllocateTime(const DMat<double> &waypoint)
{
    int cols = waypoint.cols();
    int rows = waypoint.rows();

    path.resize(rows,cols);
    path = waypoint;

    //position每维度数据用一行，一般二维xy或者三维xyz
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
    cout <<"time:    "<<times.size()<<endl;



    //临时测试
    times(0) = 0.5;
    times(1) = 0.5;

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
    //一般情况给定起点-终点的P-V-A值
    start_end_State.resize(path_dimension,6);
    start_end_State.setZero();


    pos_List.clear();
    vel_List.clear();
    acc_List.clear();
    //ede
}


void MinimumSnap::SetParas(const DMat<double> &start_end_State,const int &dimension_index)
{
    //每段时间，使用绝对时间，下面先求出每段的相对时间，然后转化为绝对时间
    DVec<double> relative_segment_Time = AllocateTime(path);
        cout<<"7-000000"<<endl;
    //k段轨迹，有k个时间段，k+1个时间点 t0 = 0.0
    segment_time.resize(relative_segment_Time.size() + 1,1);
    segment_time.setZero();
        cout<<"7-000000"<<endl;

    for (unsigned int i = 1; i <=relative_segment_Time.size(); i++ )
    {
        for(unsigned int j = 0; j < i; j++)
        {
            segment_time[i] += relative_segment_Time[j]; 
        }
    }
    cout<<"segment_time--------------"<<endl;
    for(int i(0); i<segment_time.size(); i++)
    cout<<segment_time[i]<<endl;





    /* 构建等式约束 */
    /* 构建第一组约束：起点终点PVA */
    DMat<double> Sub_A_start;
    Sub_A_start.resize(3,n+1);
    Sub_A_start.setZero();

    DMat<double> Sub_A_end;
    Sub_A_end.resize(3,n+1);
    Sub_A_end.setZero();

    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = i; j < n+1; j++)
        {
            Sub_A_start.block(i,j,1,1) << pow(segment_time[0],j-i)*Factorial(j)/Factorial(j-i);
        }
    }
    cout<<"sub-----------"<<endl;
    cout<<Sub_A_start<<endl;
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = i; j < n+1; j++)
        {
            Sub_A_end.block(i,j,1,1) << pow(segment_time[k],j-i)*Factorial(j)/Factorial(j-i);
        }
    }
        cout<<"sub-----------"<<endl;
    cout<<Sub_A_end<<endl;
    qp_A.block(0,0,3,n+1) = Sub_A_start;
    qp_A.block(4*k+2-3-1, (k-1)*(n+1), 3, n+1) = Sub_A_end;


    /* 构建第二组约束：  一共k-1个中间点~~~中间点位置-速度-加速度~连续 中间点位置, */
    for (unsigned int i = 0; i<k-1;i++)
    {
        DMat<double> Sub_Aa(3,n+1);
        Sub_Aa.setZero();
        DMat<double> Sub_Ab(3,n+1);
        Sub_Ab.setZero();
        for(unsigned int l = 0; l<3; l++)
        {
            for (unsigned int j= l; j<n+1; j++)
            {
                Sub_Aa(l,j) = pow(segment_time[i+1],j-l)*Factorial(j)/Factorial(j-l);
                Sub_Ab(l,j) = -Sub_Aa(l,j);
            }
        }
        qp_A.block(3+i*4, i*(n+1), 3, n+1) = Sub_Aa;
        qp_A.block(3+i*4, (i+1)*(n+1), 3, n+1) = Sub_Ab;
        qp_A.block(3+i*4+3, (i+1)*(n+1), 1, n+1) = -Sub_Ab.block(0,0,1,n+1);

    }
        cout<<"11-000000"<<endl;

    //构建约束的上下界，本算法中，约束为等式约束，故上下界相等
    //todolist 这个地方有错误 待修改！！！！！！！！！！！！！
    qp_L.block(0,0,3,1) = start_end_State.block(dimension_index,0,1,3).transpose();
    qp_L.block(3+4*(k-1),0,3,1) = start_end_State.block(dimension_index,3,1,3).transpose();
        cout<<"14-000000"<<endl;

    for(unsigned int i = 0; i<k-1; i++)
    {
        cout<<"15-000000"<<endl;

        DVec<double> tempVec3 = DVec<double>::Zero(3,1);
        cout<<"16-000000"<<endl;

        qp_L.block(3+i*4, 0, 3, 1) = tempVec3;
        cout<<"16-000000"<<endl;
        qp_L.block(3+i*4+3, 0, 1, 1) = path.block(dimension_index,i+1,1,1);
        cout<<"16-000000"<<endl;

    }
    qp_U = qp_L;
    cout <<"------------------"<<endl;
    cout<<start_end_State<<endl<<endl;
    cout <<qp_U<<endl;
    cout <<"------------------"<<endl;


        cout<<"13-000000"<<endl;

    //构建Q矩阵 多项式系数按照p0……pn顺序
    DMat<double> Q = DMat<double>::Zero(k*(n+1),k*(n+1));
        cout<<"12-000000"<<endl;
    cout<<"k_num "<<k<<endl;
    for (unsigned int i = 1; i < k+1; i++)
    {
        DMat<double> Sub_Q(n+1,n+1);
        Sub_Q.setZero();
        //r c分别为行和列索引
        for (unsigned int r = 0; r<=n; r++)
        {
            for (unsigned int c = 0; c<=n; c++)
            {
                if(r < order || c < order)
                {
                    continue;
                }
                Sub_Q(r,c) = (Factorial(c)*Factorial(r))/
                              (Factorial(r-order)*Factorial(c-order))*
                              ( pow(segment_time[i],r + c - 2*order + 1) - pow(segment_time[i-1],r + c - 2*order + 1) )/
                              (r - order + c - order + 1);
            }
        }
        unsigned int row = (i-1) * (n+1);
        cout<<"111-000000"<<endl;

        Q.block(row,row,n+1,n+1) = Sub_Q;
        cout<<i<<"   111-000000"<<endl;

    }
    cout <<Q<<endl;

        cout<<"111-000000"<<endl;

}

//轨迹求解时，一维一维地求解，最后合成两维或者三维的轨迹
void MinimumSnap::SolveQp(const DMat<double> &waypoint,const DMat<double> _start_end_State,const int &_order,const double &total_Time,const double &max_vel,const double &max_acc)
{
    order = _order;
    n = 2*order - 1;
    k = waypoint.cols() - 1;
    max_vel_ = max_vel;
    max_accel_ = max_acc;

    path_dimension = waypoint.rows();
    //每一行代表一个维度的所有段的多项式系数 xy 或者 xyz
    coeff_All.resize(path_dimension,k*(n+1));
    for (int i=0; i<path_dimension; i++)
    {
        ResizeQpMats();
        start_end_State = _start_end_State;

        AllocateTime(waypoint);
        cout<<"8-000000"<<endl;

        SetParas(start_end_State,i);
        cout<<"9-000000"<<endl;

        cout<<"10-000000"<<endl;
        decision_variables = qpSolver->Solve(qp_H, qp_G, qp_A, qp_L, qp_U);
        cout<<"decision "<<endl<<decision_variables<<endl;
        coeff_All.block(i, 0, 1, k*(n+1)) = decision_variables.transpose();
    }

    cout<<"123-000000"<<endl;
}










void MinimumSnap::PublishTrajectory()                                       
{

    // DVec<double> temp_position;
    // temp_position.resize(path.cols(),1);
    // temp_position.setZero();
    Vec3<double> temp_position;
    // temp_position.resize(path.cols(),1);
    temp_position.setZero();

    for (unsigned int i = 0; i < segment_time.size() - 1; ++i)
    {
        for (double t =segment_time[i] ; t < segment_time[i+1];) 
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
Vec3<double> MinimumSnap::GetPosPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3<double> position;
    const unsigned int num_poly_coeff = 2*order;
    for (unsigned int dim = 0; dim < poly_coeff_mat.rows(); ++dim)
    {
        DVec<double> coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        //默认都是列向量
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
        // for (unsigned int i = 0u; i < time.rows(); ++i) 
        // {
        //     temp_position = temp_position + coeff(i) * time(i);
        // }
        temp_position = coeff.transpose() * time;

        position(dim) = temp_position;
    }

    return position;
}

//全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3<double> MinimumSnap::GetVelPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3<double> vel;
    const unsigned int num_poly_coeff = 2*order;
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
                time(i) = pow(t, i-1)*Factorial(i)/Factorial(i-1);
            }
        }

        double temp_vel = 0.0;
        // for (unsigned int i = 0u; i < time.rows(); ++i) {
        //     temp_vel = temp_vel + coeff(i) * time(time.rows() - i - 1u);
        // }
        temp_vel = coeff.transpose() * time;

        vel(dim) = temp_vel;
    }

    return vel;
}

//全部系数 每一行代表一个维度的所有段的系数 k代表第K时间段 t为插值的时间
Vec3<double> MinimumSnap::GetAccPolynomial(const DMat<double> &poly_coeff_mat, unsigned int k, double t) 
{
    Vec3<double> acc;
    const unsigned int num_poly_coeff = 2*order;
    for (unsigned int dim = 0; dim < 3u; ++dim) 
    {
        DVec<double> coeff = (poly_coeff_mat.row(dim)).segment(num_poly_coeff * k, num_poly_coeff);
        DVec<double> time = DVec<double>::Zero(num_poly_coeff);

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
        // for (unsigned int i = 0u; i < time.rows(); ++i) {
        //     temp_acc = temp_acc + coeff(i) * time(time.rows() - i - 1u);
        // }
        temp_acc = coeff.transpose() * time;

        acc(dim) = temp_acc;
    }

    return acc;
}

