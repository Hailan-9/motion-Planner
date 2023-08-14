/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    hybrid_A*.hpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-07-16
 */
#pragma once

#include <iostream>
#include <vector>
#include <list>
#include "cppTypes.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <unordered_map>
#include <boost/functional/hash.hpp>

using namespace std;
using namespace Eigen;


/**
 * @brief   描述
 * 
 * @author  hailan(2522082924@qq.com)
 * @date    2023-07-16
 */
namespace KinodynamicAstar
{
    #define REACH_HORIZON 1
    #define REACH_END 2
    #define NO_PATH 3


    #define IN_CLOSE_SET 'a'
    #define IN_OPEN_SET 'b'
    #define NOT_EXPAND 'c'
    // 右移，使其变成无穷小
    #define inf 1 >> 30


    class PathNode
    {
        public:
            // 机器人运动在二维空间，也就是二维地图，地图上的索引为2维的应该即可
            Vec2<int> index;
            // 基于三角全向移动底盘，状态为px、py、vx、vy也就是说状态为一个四维列向量
            Vec4<double> state;
            // 使用初始化器列表，将变量初始化为默认值。
            // 对于double类型的变量来说，默认初始化的效果是将其值设置为 0.0。
            double g_value{},f_value{};
            // todolist根据车辆模型的不同而不同!!!
            // input 和 duration都是当前节点（P.top()）到扩展节点这个过程中的量，但是将这个量赋值给了新生成的这个扩展节点的对应属性
            // 输入变量，也就是u，为状态变量阶次最高的基础上，再求次导，也就是ax、ay
            Vec2<double> input;
            double duration{};
            double time{};
            // 时间对应的索引值
            int time_idx;
            PathNode* parent;
            char node_state{};

            explicit PathNode()
            {
                parent = NULL;
                node_state = NOT_EXPAND;
            }

            virtual ~PathNode(){};

            EIGEN_MAKE_ALIGEND_OPERATOR_NEW     
    };

    typedef PathNode* PathNodePtr;


    // 优先级队列---priority_queue中，元素之间的比较（通常使用 < 运算符或提供的自定义比较器）用于确定元素的优先级关系,也就是优先级的判定标准。
    // 在c++中，class和struct的唯一区别就是默认的访问权限不同，
    // 成员的默认权限，class默认为私有，struct默认为公有,所以下面这个地方用struct或者class都可以,看个人习惯
    /* ---第二种方法---自定义优先级队列的比较器,要使用重载函数---调用运算符 operator() 来实现自定义比较器*/
    // 一般定义数据类型，使用struct；实现某项功能使用class
    struct NodeComparator
    {
        bool operator() (PathNodePtr node1, PathNodePtr node2)
        {
            // 第二个对象的元素 小于 第一个对象的元素时，才真，说明较小的值具有较高的优先级
            return node1->f_value > node2->f_value;
        }
    };



    // todolist---哈希矩阵 哈希值
    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t>
    {
        std::size_t operator()(T const& matrix) const
        {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i)
            {
                // 一般列优先
                auto elem = *(matrix.data() + i);
                // 异或，对应位不同返回1
                seed ^= std::hash<typename T::Scalar>()(elem) + 
                0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    class NodeHashTable 
    {
        private:
            // data
            // 实现无序映射的一个容器，其构造函数传值顺序 好像有点问题 这个问题还没有解决！！todolist
            std::unordered_map<Vec2<int>, PathNodePtr, matrix_hash<Vec2<int>>> data_2d;
            std::unordered_map<Vec3<int>, PathNodePtr, matrix_hash<Vec3<int>>> data_3d;

        public:
            NodeHashTable(/* param */) {}
            ~NodeHashTable() {}

            void insert(Vec2<int> idx, PathNodePtr node)
            {
                data_2d.insert(std::make_pair(idx, node));
            }
            void insert(Vec2<int> idx, int time_idx, PathNodePtr node)
            {
                data_3d.insert(std::make_pair(
                    Eigen::Vector3i(idx(0),idx(1),time_idx), node));
            }
            PathNodePtr find(Vec2<int> idx)
            {
                auto iter = data_2d.find(idx);
                return iter == data_2d.end() ? NULL : iter->second;
            }
            PathNodePtr find(Vec2<int> idx, int time_idx)
            {
                auto iter = data_3d.find(
                    Eigen::Vector3i(idx(0), idx(1), time_idx));
                return iter == data_3d.end() ? NULL : iter->second;
            }

            void clear()
            {
                data_2d.clear();
                data_3d.clear();
            }

    };




    class Hybrid_Astar
    {
        private:
            /*------main data structure---------*/
            vector<PathNodePtr> path_node_pool;
            int use_node_num{};
            int iter_num{};
            // todolist---哈希表这个东西。第一次用，目前还不是很了解在这个算法的用处

            // 这个变量是在整个搜索过程中，被扩展过的所有节点，也就是被访问过的所有节点。
            NodeHashTable expanded_nodes;
            std::priority_queue<PathNodePtr, vector<PathNodePtr>, NodeComparator>
                open_set;
            // 存储路径点，不包括目标点，最后一个节点，距离目标点特别近
            vector<PathNodePtr> path_nodes;



            /*---------record data---------*/
            Vec2<double> start_vel{};
            Vec2<double> start_acc{};
            Vec2<double> end_vel{};
            /*---状态方程中的A矩阵，如果是nilpotent，也就是幂邻矩阵，那么状态转移矩阵e的At次方计算就比较方便了*/
            // 移动机器人中，如果是二阶积分器的话，也就是p v是状态，a是系统输入，那么状态矩阵A为
            // 0 0 1 0
            // 0 0 0 1
            // 0 0 0 0
            // 0 0 0 0
            // 该矩阵的二次方就开始为0矩阵了，那么状态转移矩阵e的At次方求解很简单
            Mat4<double> phi;//state transit matrix
            // 带shot的均是指从当前节点直接连接一条到终点或者末点的一条代价最小的，满足安全性约束的轨迹
            bool is_shot_succ = false;

            EDTEnvironment::Ptr edt_environment;
            // shot shot---从当前节点直接连接到终点或者末点的一条代价最小，并且满足动力学和无碰撞要求的轨迹的多项式系数
            DMat<double> coef_shot;
            double t_shot{};
            bool has_path = false;

            /*------------parameter--------------*/
            /* search */
            // 最大时间 初始化时间
            double max_tau, init_max_tau;
            double max_vel, max_acc;
            // horizon是搜索范围，last为启发函数的权值 heuristic 在J中的w_time时间权值
            double w_time, horizon, lambda_heu;
            int allocate_num, check_num;
            double tie_breaker;
            bool optimistic;
            
            /* map 分辨率，带inv的是对应分辨率的倒数---*/
            double resolution, inv_resolution, time_resolution,
            inv_time_resolution;
            Vec3<double> origin, map_size_3d;
            double time_origin;


            /* helper */
            Vec3<double> posToIndex(Vec3<double> pt);
            int timeToIndex(double time);
            void retrievePath(PathNodePtr end_node);

            /* shot trajectory */

            // 在 Hybrid A*（混合 A*）算法中，“shot trajectory” 是指发射轨迹，在路径规划问题中用于表示从起始点到目标点的路径。
            // Hybrid A* 算法是一种融合了 A* 算法和采样轨迹的算法，用于在离散的网格地图或连续的空间中，通过生成和评估连续的轨迹来搜索最优路径。在 Hybrid A* 算法中，使用控制输入来生成候选的连续轨迹，并通过启发式函数（如曼哈顿距离、欧几里得距离等）评估轨迹的优劣。
            // “Shot trajectory” 是在 Hybrid A* 算法中用于表示从起始点到目标点的一个候选轨迹。这个轨迹通常是由一个或多个离散的控制输入（例如转向角度、速度的变化等）决定的。通过生成和评估许多不同的 “shot trajectories”，Hybrid A* 算法可以搜索并找到最优路径。
            // todolist 这块求解的还没有看懂
            // 三次
            vector<double> cubic(double a, double b, double c,double d);
            // 四次
            vector<double> quartic(double a, double b, double c, double d, double e);
            bool computeShotTraj(DVec<double> state1, DVec<double> state2, double& time_to_goal);
            double estimateHeuristic(DVec<double> x1, DVec<double> x2, double& optimal_time);


            /*  state propagation  */
            // todolist---需要根据机器人的运动模型进行修改
            void stateTransit(Vec6<double>& state0,
                              Vec6<double>& state1, Vec3<double> um,
                              double tau);


        public:
            explicit Hybrid_Astar(){};
            virtual ~Hybrid_Astar(){};
            // 类中定义的枚举类型，其中的枚举常量可以作为类的成员使用，这里有点类似于宏定义
            enum
            {
                REACH_HORIZON = 1;
                REACH_END = 2;
                NO_PATH = 3;
                NEAR_END = 4;
            };

            /* main API Function */
            void setParam(const ros::NodeHandle& nh);
            void init();
            void reset();
            int search(Vec3<double> start_pt, Vec3<double> start_v,
                       Vec3<double> start_a, Vec3<double> end_pt,
                       Vec3<double> end_v,bool init, bool dynamic,
                       double time_start);
            void setEnvironment(const EDTEnvironment::Ptr& env);

            void getSamples(double& ts, vector<Vec2<doubke>>& point_set,
            vector<Vec2<double>>& start_end_derivatives);

            vector<PathNodePtr> getVisitedNodes();
            vector<Vec2<double>> getKinoTraj(double delta_t);

            typedef shared_ptr<Hybrid_Astar> Ptr;
            
            //---编程技巧，矩阵对齐用的
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };



}





