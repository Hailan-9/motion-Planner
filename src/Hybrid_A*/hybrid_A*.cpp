/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    hybrid_A*.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-07-20
 */
#include "hybrid_A*.hpp"
// 该头文件提供了 stringstream 类，它是 C++ 标准库中的一个类，用于进行字符串和各种数据类型之间的相互转换。
#include <sstream>


using namespace std;
using namespace Eigen;


namespace Hybrid_Astar
{









/**
 * @brief   检索路径，代码算法比较固定
 * 
 * @param   end_node    参数描述
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-24
 */
void Hybrid_Astar::retrievePath(PathNodePtr end_node)
{
    PathNodePtr cur_node = end_node;
    path_nodes.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
        cur_node = cur_node.parent;
        path_nodes.push_back(cur_node);
    }

    reverse(path_nodes.begin(),path_nodes.end());
}

// 


/*




step3:节点扩张---在当前节点的基础上，根据对输入、时间的离散进行扩展得到临时节点，
首先判断临时节点是否被扩展过，是否与当前节点在同一个节点，检查速度约束、检查碰撞，如果
都通过的话，就计算当前节点的g和f。其中的state transit函数就是通过前向积分得到扩展节点的位置和速度，
也就是基于控制空间进行采样。



*/
int Hybrid_Astar::search(Vec3<double> start_pt, Vec3<double> start_v,
            Vec3<double> start_a, Vec3<double> end_pt,
            Vec3<double> end_v,bool init, bool dynamic,
            double time_start)
{
    // todolist记录一下规划的时间， 参考zhangzhimeng的代码
    /*--------将起始点及目标点的二维位置转化至栅格地图上的index，并计算第一个扩展点的heuristic cost----*/
    start_vel = start_v;
    start_acc = start_a;

    // 设置第一个节点 起始节点
    PathNodePtr cur_node = path_node_pool[0];
    cur_node->parent = NULL;
    cur_node->state.head(2) = start_pt;
    cur_node->state.tail(2) = start_vel;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_value = 0.0;


    // 设置最后一个节点 目标节点
    Vec4<double> end_state;
    Vec2<double> end_index;
    double time_to_goal;

    end_state.head(2) = end_pt;
    end_state.tail(2) = end_v;
    end_index = posToIndex(end_pt);


    // core-part caculate f_value
    cur_node->f_value = 
    lambda_heu * estimateHeuristic(cur_node->state,end_state,time_to_goal);
    cur_node->node_state = IN_OPEN_SET;
    // 把起点放入openlist---和---expanded_nodes
    open_set.push(cur_node);
    use_node_num += 1;

    // 若true，则表示不同时间到达同一个栅格的节点，不算重复节点，也就是说排除重复节点时考虑了时间信息
    if (dynamic)
    {
        time_origin = time_start;
        cur_node->time = time_start;
        cur_node->time_idx = timeToIndex(cur_node->time);
        expanded_nodes.insert(cur_node->index, cur_node->time_idx, cur_node);
    }
    else
        expanded_nodes.insert(cur_node->index, cur_node);

    PathNodePtr neighbor = NULL;//邻节点
    PathNodePtr terminate_node = NULL;//终止节点
    bool init_search = init;

    // 判断是否是初始化搜索，也就是是否是第一次搜索，只有从起点开始的搜索是第一次搜索
    // 第一次搜索的输入，是起点的加速度，而其他的搜索，是离散控制输入
    // 向上取整
    int tolerance = ceil(1.0 / resolution);
    // int tolerance2 = ceil(2.5 * end_v.norm() * 0.4 / resolution);
    // int tolerance = 2;





    /*--------搜索扩张节点迭代循环------------*/
    while(!open_set.empty())
    {
        // 在openlist中寻找代价最小的节点，直到openlist为空
        // todolist---计算规划到现在所消耗的时间        

        /* step1---从openset优先级队列中选出f(n) = g(n) + h(n)代价值最小的节点*/
        cur_node = open_set.top();//选出栈顶节点，该优先级队列的栈顶节点就是代价值f最小的节点
        
        /* step2---判断当前节点是否超出horizon或者是离终点很近了，
        并计算一条直达shot曲线，检查这条曲线上是否存在，若存在，则搜索完成，返回路径所有路径点，
        */

        // 可以通过设置不同的值来调整规划算法的搜索范围
        // reach_horizon定义了规划算法搜索或优化的水平范围的最大值。
        // 如果规划结果超出了这个范围，即超过了reach_horizon，则该结果可能会被视为无效或不可行。这可以用来控制规划算法的搜索范围
        bool reach_horizon = (cur_node->state.head(2) - start_pt).norm() >= horizon;
        
        // 较低的分辨率可以使地图更粗糙，每个栅格覆盖的空间范围较大，而较高的分辨率可以使地图更详细，每个栅格覆盖的空间范围较小。
        bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                        abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                        abs(cur_node->index(2) - end_index(2)) <= tolerance;    


        if (reach_horizon || near_end)
        {
            terminate_node = cur_node;
            retrievePath(terminate_node);
            if(near_end)
            {
                // check whether shot traj exit
                estimateHeuristic(cur_node->state, end_state, time_to_goal);
                computeShotTraj(cur_node->state, end_state, time_to_goal);
                if (init_search)// 是否是首次搜索
                {
                    cout << "shot in first search loop" << endl;
                }
            }
        }
        if (reach_horizon)
        {
            if(is_shot_succ)
            {
                cout << "reach end" << endl;
                return REACH_END;
            }
            else
            {
                cout << "reach horizon" << endl;
                return REACH_HORIZON;
            }
        }
        if (near_end)
        {
            if(is_shot_succ)
            {
                cout << "reach end" << endl;
                return REACH_END;
            }
            else if(cur_node->parent != NULL)
            {
                cout << "near end" << endl;
                return NEAR_END;
            }
            else
            {
                cout << "no path" << endl;
                return NO_PATH;
            }
        }

        /* 计算一条直达曲线one shot trajectory */ 
        //计算最优控制时间time_to_goal
        // estimateHeuristic(cur_node->state, end_state, time_to_goal);
        //计算并验证该轨迹是安全的，即不发生碰撞，速度、加速度不超限。
        //computeShotTraj(cur_node->state, end_state, time_to_goal);  



        /*---step3---若当前节点没有抵达终点，就要进行节点扩张----*/
        // 以下代码在当前节点的基础上，根据对系统输入 时间的离散进行扩展得到临时节点
        // 首先检查判断节点是否被扩展过，expanded_nodes里面存储的是被扩展过的所有节点
        // 检查速度约束 检查碰撞 若都通过，则计算当前节点的g和h
        // 其中的state transit函数即通过前向积分得到扩展节点的位置和速度，接下来
        // 进行节点减枝


        /* ---step3.1---在openset中删除当前节点，并在closeset中添加当前节点 */
        open_set.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num += 1;


        /* ---step3.2---初始化状态传递 */
        double res = 1 / 2.0;
        double time_res = 1 / 1.0;
        double time_res_init = 1 / 20.0;

        Vec4<double> cur_state = cur_node->state;
        Vec4<double> pro_state;
        // 临时扩张节点集合
        // 每一次在当前节点基础上进行expand时，产生的primitive（也就是前向积分得到的轨迹）末端的状态，也就是节点
        vector<PathNodePtr> tmp_expand_nodes;
        Vec2<double> um;
        double pro_t;

        vector<Vec2<double>> inputs;
        vector<double> durations;


        /* step3.3---判断节点没有被扩展过，把这个节点存下来 */
        // 1e-5这个用法应该是考虑到double类型的精度，让ax可以达到max_acc
        if (init_search)
        {
            inputs.push_back(start_acc);
            for (double tau = time_res_init * init_max_tau; tau<=init_max_tau + 1e-5; tau += time_res_init * init_max_tau)
            {
                durations.push_back(tau);
            }
            init_search = false;
        }
        else
        {
            for (double ax = -max_acc; ax<=max_acc + 1e-5; ax+=0.5)
            {
                for(double ay = -max_acc; ay<=max_acc + 1e-5; ay+=0.5)
                {
                    um<<ax,ay;
                    inputs.push_back(um);
                }
            }
            // 对于每个primitive来说，长度也就是时间是固定的，是tau
            for (double tau = time_res * max_tau; tau <= max_tau; tau += time_res * max_tau)
                durations.push_back(tau);
        }


        /* step3.4---状态传递迭代 */
        for (int i=0; i<inputs.size(); ++i)
        {
            for(int j=0; j<durations.size(); ++j)
            {
                um = inputs(i);
                double tau = durations(j);
                // 状态传递，通过前向积分得到扩展节点的位置和速度
                stateTransit(cur_state, pro_state, um, tau);
                pro_t = cur_node->time + tau;

                Vec2<double> pro_pos = pro_state.head(2);

                // 检查是否在closeset中 pro_id应该是扩张的节点对应地图中的索引ID号
                Vec2<int> pro_id = posToIndex(pro_pos);
                int pro_t_id = timeToIndex(pro_t);

                // dynamic就是用来判断检查节点时是否考虑时间信息
                // 判断该节点是否已经被扩展过，并且是否在closeset中，后者是重点，和伪代码中流程一样的
                PathNodePtr pro_node = dynamic ? expanded_nodes.find(pro_id, pro_t_id)
                :expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
                {
                    if(init_search)
                    {
                        cout<<"close"<<endl;
                    }
                    continue;
                }
                //check maximal velocity
                Vec2<double> pro_v = pro_state.tail(2);
                if (fabs(pro_v(0)) > max_vel || fabs(pro_v(1)) > max_vel)
                {
                    if(init_search)
                    {
                        cout<<"vel"<<endl;
                    }
                    continue;
                }
                // check not in the same voxel
                Vec2<int> diff = pro_id - cur_node->index;
                int diff_time = pro_t_id - cur_node->time_idx;
                if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
                {
                    if(init_search)
                    {
                        cout<<"same"<<endl;
                    }
                    continue;
                }

                // check safety
                Vec2<double> pos;
                Vec4<double> xt;
                bool is_occ = false;
                for (int k=1; k<=check_num; ++k)
                {
                    double dt = tau * double(k) / double(check_num);
                    stateTransit(cur_state, xt, um, dt);
                    pos = xt.head(2);
                    if (edt_environment->sdf_map->getInflatOccupancy(pos) == 1)
                    {
                        is_occ = true;
                        break;
                    }
                }
                if (is_occ)
                {
                    if (init_search)
                    {
                        cout<<"safe"<<endl;
                    }
                    continue;
                }



                double time_to_goal, tmp_g_value, tmp_f_value;
                tmp_g_value = (um.squareNorm() + w_time) * tau + cur_node->g_value;
                tmp_f_value = tmp_g_value + lambda_heu * estimateHeuristic(pro_state, end_state, time_to_goal);


                /*---******节点剪枝******---*/
                // voxel也就是一个栅格
                // compare nodes expanded from the same parent
                //使用指针操作，剪枝的时候，该栅格中的存储的primitive是代价最小的，已经被修正为最小的了，进而完成剪枝操作！！
                bool prune = false;
                for (int j = 0; j < tmp_expand_nodes.size(); j++)
                {
                    PathNodePtr expand_node = tmp_expand_nodes(j);
                    if( (pro_id - expand_node->index).norm() == 0 
                    && ((!dynamic) || pro_t_id == expand_node->time_idx) )
                    {
                        prune = true;
                        if (tmp_f_value < expand_node->f_value)
                        {
                            expand_node->f_value = tmp_f_value;
                            expand_node->g_value = tmp_g_value;
                            expand_node->state = pro_state;
                            expand_node->input = um;
                            expand_node->duration = tau;
                            if(dynamic)
                            {
                                expand_node->time = cur_node->time + tau;
                                expand_node->time_idx = timeToIndex(pro_node->time);
                            }
                        }
                        break;
                    }
                }


                // this node end up in a voxel different from others
                // 如果这个被扩展产生的primitive节点（primitive的终点），没有被剪枝，则该节点要进入伪代码中的11~16行的步骤
                if (!prune)
                {
                    // 说明不在closeset中，并且满足可行性约束，也就是满足运动学约束和不碰撞障碍物或者穿过障碍物
                    // 并且也不在openset中
                    if (pro_node == NULL)
                    {
                        pro_node = path_node_pool(use_node_num);
                        pro_node->index = pro_id;
                        pro_node->state = pro_state;
                        pro_node->f_value = tmp_f_value;
                        pro_node->g_value = tmp_g_value;
                        pro_node->input = um;
                        pro_node->duration = tau;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;
                        if (dynamic)
                        {
                            pro_node->time = cur_node->time + tau;
                            pro_node->time_idx = timeToIndex(pro_node->time);
                        }

                        open_set.push(pro_node);

                        if (dynamic)
                            expanded_nodes.insert(pro_node->index, pro_node->time, pro_node);
                        else
                            expanded_nodes.insert(pro_node->index, pro_node);
                        
                        tmp_expand_nodes.push_back(pro_node);

                        use_node_num += 1;
                        if (use_node_num == allocate_num)
                        {
                            cout <<"run out of memory." << endl;
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == IN_OPEN_SET)
                    {
                        if (tmp_g_value < pro_node->g_value)
                        {
                            pro_node->state = pro_state;
                            pro_node->g_value = tmp_g_value;
                            pro_node->f_value = tmp_f_value;
                            pro_node->input = um;
                            pro_node->parent = cur_node;
                            pro_node->duration = tau;
                            if (dynamic)
                            {
                                pro_node->time = cur_node->time + tau;
                                pro_node->time_idx = timeToIndex(pro_node->time);
                            }
                        }
                    }
                    else
                    {
                        cout << "error type in searching: " << pro_node->node_state << endl;
                    }

                }

            }
        }

    }

    cout<<"openset empty, no path!"<<endl;
    cout<<"use node num: "<<use_node_num<<endl;
    cout<<"iter num: "<<iter_num<<endl;
    return NO_PATH;


}



void Hybrid_Astar::reset()
{
    expanded_nodes.clear();
    path_nodes.clear();

    priority_queue<PathNodePtr, vector<PathNodePtr>,NodeComparator> empty_queue;
    open_set.swap(empty_queue);

    for (int i(0); i < use_node_num; i++)
    {
        PathNodePtr node = path_node_pool(i);
        node->parent = NULL;
        node->node_state = NOT_EXPAND;
    }

    use_node_num = 0;
    iter_num = 0;
    is_shot_succ = false;
    has_path = false;
}



/**
 * @brief   这一函数的作用是在完成路径搜索之后，按照预设的时间分辨率delta_t，通过
 * 节点回溯和状态前向积分，得到分辨率更高的路径点。
 * 如果最后的shot trajectory存在，也就是最后一段当前点到终点的shot traj，
 * 则还要加上最后一段的shot traj
 * 
 * @param   delta_t     时间分辨率
 * @return  vector<Vec2<double>> 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-27
 */
vector<Vec2<double>> Hybrid_Astar::getKinoTraj(double delta_t)
{
    vector<Vec2<double>> state_list;

    /*-------------get traj of searching--------------*/
    // 最后一个节点，也就是目标点goal_node
    PathNodePtr node = path_nodes.back();
    Vec4<double> x0, xt;

    while (node->parent != NULL)
    {
        Vec2<double> ut = node->input;
        double duration = node->duration;
        x0 = node->parent->state;

        for (double t = duration, t >= -1e-5; t -= delta_t)
        {
            stateTransit(x0, xt, ut, t);
            state_list.push_back(xt.head(2));
        }
        node = node->parent;
    }

    reverse(state_list.begin(),state_list.end());

    /*-------------get traj of one shot----------------*/
    if (is_shot_succ)
    {
        Vec2<double> coord;// 坐标
        DVec<double> poly1d,time(4);

        for (double t = delta_t; t <= t_shot; t += delta_t)
        {
            for (int j = 0; j < 4; j++)
            {
                time(j) = pow(t,j);
            }
            
            for (int dim = 0; dim < 2; dim++)
            {
                poly1d = coef_shot.row(dim);
                coord(dim) = poly1d.dot(time);
            }

            state_list.push_back(coord);
        }
    }

    return state_list;

}

/**
 * @brief   离散地获得一些轨迹点和起始点的速度与加速度
 * 
 * @param   ts          参数描述
 * @param   point_set   参数描述
 * @param   start_end_derivatives 参数描述
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-27
 */
void Hybrid_Astar::getSample(double& ts, vector<Vec<double>>& point_set, vector<Vec2<double>>& start_end_derivatives)
{
    double T_sum = 0.0;
    if (is_shot_succ)
        T_sum += t_shot;
    
    PathNodePtr node = path_nodes.back();

    while(node->parent != NULL)
    {
        T_sum += node->duration;
        node = node->parent;
    }

    Vec2<double> end_vel_temp, end_acc_temp;
    double t;
    if (is_shot_succ)
    {
        t = t_shot;
        end_vel_temp = end_vel;
        for (int dim(0); dim < 2; dim++)
        {
            Vec4<double> coe = coef_shot.row(dim);
            end_acc_temp(dim) = coe(2) * 2 + 6 * t_shot * coe(3);
        } 
    }
    else
    {
        t = path_nodes.back()->duration;
        //fastplanner开源代码中这个地方有点错误！！！！
        end_vel_temp = path_nodes.back()->state.tail(2);
        end_acc_temp = path_nodes.back()->input;
    }

    // get sample points
    int seg_num = floor(T_sum / ts);
    seg_num = max(8,seg_num);
    ts = T_sum / (double)(seg_num);
    bool sample_shot_traj = is_shot_succ;
    node = path_nodes.back();

    for (double ti = T_sum; ti > -1e-5; ti -= ts)
    {
        if (sample_shot_traj)
        {
            Vec2<double> coord_pos;
            Vec4<double> poly1d, time;

            for (int j = 0; j < 4; j++)
            {
                time(j) = pow(t,j);
            }

            for (int dim(0); dim < 2; dim++)
            {
                poly1d = coef_shot.row(dim);
                coord_pos(dim) = poly1d.dot(time);
            }

            point_set.push_back(coord_pos);
            t -= ts;

            if(t < -1e-5)
            {
                sample_shot_traj = false;
                // fastplanner开源代码中 这个时间t有点问题！！！应该是我写的这样
                if (node->parent != NULL)
                    t = node->duration;
            }
        }
        else
        {
            Vec4<double> x0 = node->parent->state;
            Vec4<double> xt;
            Vec2<double> ut = node->input;

            stateTransit(x0, xt, ut, t);

            point_set.push_back(xt.head(2));
            t -= ts;
            // todolist 这里和fastplanner源代码中处理的不一样
            if (t < -1e-5 && node->parent->parent != NULL)
            {
                node = node->parent->parent;
                t = node->duration;
            }
        }
    }

    reverse(point_set.begin(), point_set.end());

    
    // 这个地方和fastplanner源代码中有所不同
    start_end_derivatives.push_back(start_vel);
    start_end_derivatives.push_back(end_vel_temp);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc_temp);
}



void Hybrid_Astar::init()
{
    /*-----------map params--------------*/
    this->inv_resolution = 1.0 / resolution;
    inv_time_resolution = 1.0 / time_resolution;
    edt_environment->sdf_map->getRegion(prigin, map_size_2d);

    cout << "origin:------" << origin.transpose() <<endl;
    cout << "map size:-------" << map_size_2d.transpose() <<endl;

    /*--------------pre-allocated node-----------------*/
    path_node_pool.resize(allocate_num);
    for (int i(0); i < allocate_num; i++)
    {
        // new pathNode 返回的是一个该类型的指针，也就是地址
        path_node_pool(i) = new PathNode;
    }
    phi = Eigen::MatrixXd::Identity(4,4);
    //
    use_node_num = 0;
    // 
    iter_num = 0;
}




vector<PathNodePtr> Hybrid_Astar::getVisitedNodes()
{
    vector<PathNodePtr> visited;
    visited.assign(path_node_pool.begin(), path_node_pool.begin() + use_node_num - 1);
    return visited;
}


/**
 * @brief   时间转化为索引值
 * ---当前时间减去开始时间，然后乘以时间分辨率
 * 
 * @param   time        参数描述
 * @return  int 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-27
 */
int Hybrid_Astar::timeToIndex(double time)
{
    // 向下取整，floor(x)将返回小于等于x的最大整数
    int idx = floor((time - time_origin) * inv_time_resolution);
    return idx;
}

/**
 * @brief   描述
 * 
 * @param   pt          参数描述
 * @return  Vec2<int> 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-27
 */
Vec2<int> Hybrid_Astar::posToIndex(const Vec2<double>& pt)
{
    // 在Eigen库中，Array和Vector是两种不同的数据结构
    // .array()用于将Vector类型的对象转换为Array类型的对象
    // .cast<int>()：这是Array类型中的成员函数，用于将每个元素的类型转换为整数类型
    Vec2<int> idx = ((pt - origin) * inv_resolution).array().floor().cast<int>();
    return idx;
}

/**
 * @brief   根据论文和课程中的公式，亲手推出来的公式,---从理论到code，太爽了！
 * 幂领矩阵太好了，此问题中A的二次方就为0矩阵了，对于解决问题，简单了很多
 * 
 * @param   state0      参数描述
 * @param   state1      参数描述
 * @param   um          离散的控制输入 ud
 * @param   tau         每次生成一段或者一个primitive的长度也就是时间是固定值，是tau
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-27
 */
void Hybrid_Astar::stateTransit(Vec4<double>& state0, Vec4<double>& state1, 
                                Vec2<double> um, double tau)
{
    for (int i(0); i < 2; i++)
    {
        phi(i, i+2) = tau;
    }
    Vec4<double> integral;
    integral.head(2) = 0.5 * pow(tau, 2) * um;
    integral.tail(2) = tau * um;
    // tau时刻的状态，------从0时刻也就当前时刻到tau时刻的状态转移
    state1 = phi * state0 + integral;
}



/**
 * @brief   为混合Astar算法配置参数，这里使用了ros，下面的代码一般格式都比较固定。
 * 
 * @param   nh          ros句柄
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-25
 */
void Hybrid_Astar::setParam(const ros::NodeHandle& nh)
{
    nh.param("search/max_tau", max_tau, -1.0);
    nh.param("search/init_max_tau", init_max_tau, -1.0);
    nh.param("search/max_vel", max_vel, -1.0);
    nh.param("search/max_acc", max_acc, -1.0);
    nh.param("search/w_time", w_time, -1.0);
    nh.param("search/horizon", horizon, -1.0);
    nh.param("search/resolution_astar", resolution, -1.0);
    nh.param("search/time_resolution", time_resolution, -1.0);
    nh.param("search/lambda_heu", lambda_heu, -1.0);
    nh.param("search/allocate_num", allocate_num, -1.0);
    nh.param("search/check_num", check_num, -1.0);
    nh.param("search/optimistic", optimistic, -1.0);
    tie_breaker = 1.0 + 1.0 / 10000;

    double vel_margin;//todolist速度差额？？？
    nh.param("search/vel_margin", vel_margin, 0.0);
    max_vel += vel_margin;
}

/**
 * @brief   基于三角全向移动底盘模型，原理我看明白了，但是涉及到求导的部分，好复杂，不知道咋整理
 * 表达式 咋求解的 这部分先放放 太数学了
 * 原理和高飞讲的基本一样，obvp问题我自己又亲手推导了一遍，很难受，但是推一遍还是很爽的，但是涉及到最后的求导计算，感觉过于复杂 头大
 * 有时间再仔细研究这部分的数学理论！！！
 * 
 * @param   x1          参数描述
 * @param   x2          参数描述
 * @param   optimal_time参数描述
 * @return  double 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-25
 */
double Hybrid_Astar::estimateHeuristic(DVec<double> x1, DVec<double> x2, double& optimal_time)
{
    const Vec2<double> dp = x2.head(2) - x1.head(2);
    const Vec2<double> v0 = x1.segment(2,2);
    const Vec2<double> v1 = x2.segment(2,2);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1,dot(v1));
    double c4 = 0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
        if (t < t_bar)
        continue;
        double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
        if (c < cost)
        {
        cost = c;
        t_d = t;
        }
    }

    optimal_time = t_d;

    return 1.0 * (1 + tie_breaker_) * cost;

}





/**
 * @brief   背后的数学原理暂时没太理解todolist
 * 
 * @param   a           参数描述
 * @param   b           参数描述
 * @param   c           参数描述
 * @param   d           参数描述
 * @param   e           参数描述
 * @return  vector<double> 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-26
 */
vector<double> Hybrid_Astar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}




/**
 * @brief   背后的数学原理暂时未理解，出现问题或者有需要的话再仔细研究，这部分特别头大！！！todolist
 * 
 * @param   a           参数描述
 * @param   b           参数描述
 * @param   c           参数描述
 * @param   d           参数描述
 * @return  vector<double> 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-25
 */
vector<double> Hybrid_Astar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}














/**
 * @brief   利用庞特里亚金原理解一个两点边值问题,
 * ----当当前节点距离目标点较近时，检查是否可以直接连一条到目标点的轨迹，使得满足OBVP问题，求解后再检查安全性！
 * 这个函数的目的是为了验证该轨迹是安全的，也就是不发生与障碍物的碰撞，速度 加速度在运动学约束范围内
 * todolist数学原理未深究，先把混合Astar的大框架搭好，在解决这些求解的数学问题！
 * 
 * @param   state1      当前点状态 p v等
 * @param   state2      目标点状态 p v等
 * @param   time_to_goal 这个参数是当前点到目标点，使得J最小的时间T，也就是在estimateHeuristic里面计算的时间
 * @return  bool 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-26
 */
bool Hybrid_Astar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
    /*--------利用庞特里亚金和求解OBVP问题，根据亲手推导的p(t)公式，针对二阶积分器，
    则p(t) = 1/6*alpha*t^3 + 0.5*beta*t^2 + v0*t + p0 -----------------*/
    /*-----------get polynomial coefficient -------------*/
    const Vec2<double> p0 =state1.head(2);
    const Vec2<double> dp = state2.head(2) - p0;

    const Vec2<double> v0 = state1.tail(2);
    const Vec2<double> v1 = state2.tail(2);
    const Vec2<double> dv = v1 - v0;

    // 右值是使得从当前点到目标终点，使得J最小的时间T，也就是optimal t
    // 根据J对T求导得出的，这部分较为复杂，在estimateHeuristic里面有计算和求解过程。
    double t_d = time_to_goal;
    // 移动机器人，仅考虑二维平面上的运动，无Z维度
    DMat<double> coef(2,4);
    end_vel = v1;

    Vec2<double> a = 1.0 / 6.0 * ( (-12.0 / pow(t_d,3)) * (dp - v0 * t_d) + 6.0 / pow(t_d,2) * dv );
    Vec2<double> b = 0.5 * ( 6.0 / pow(t_d,2) * (dp - v0 * t_d) + (-2.0 / t_d) * dv );
    Vec2<double> c = v0;
    Vec2<double> d = p0;

    // 阶次依次变高
    coef.col(0) = d;
    coef.col(1) = c;
    coef.col(2) = b;
    coef.col(3) = a;
    // 中间离散点的坐标位置 速度 加速度
    Vec2<double> coord_pos, coord_vel, coord_acc;
    // 多项式系数
    Vec4<double> poly1d, t;
    Vec2<double> index;

    DMat<double> Tm(4,4);
    Tm << 0, 1, 0, 0,
          0, 0, 2, 0,
          0, 0, 0, 3,
          0, 0, 0, 0;


    /*----forward checking of trajectory-----------*/
    // 将当前点到期望终点的最优时间，离散为十份，进行安全检查，包括有无和障碍物的碰撞，有无速度 加速度不满足约束
    // 这里使用的时间是相对时间，也就是以当前节点开始为0时刻
    double t_delta = t_d / 10.0;
    for (double time = t_delta,; time <= t_d + 1e-10; time += t_delta)
    {
        for (int i = 0; i < 4; i++)
        {
            t(i) = pow(time,i);
        }
        for (int dimension = 0; dimension < 2; dimension++)
        {
            poly1d = coef.row(dimension);
            coord_pos(dimension) = t.dot(poly1d);
            coord_vel(dimension) = t.dot(Tm * poly1d);
            coord_acc(dimension) = t.dot(Tm * Tm * poly1d);

            // check vel && acc
            if ( fabs(coord_vel(dimension)) > max_vel || fabs(coord_acc(dimension)) > max_acc )
            {
                cout << "vel: " <<coord_vel(dimension) << " , " << "acc: " << coord_acc(dimension) << endl;
                // return false;
            }
        }
        // Check if it exceeds the map boundary 
        if (coord_pos(0) < origin(0) || coord_pos(0) >= map_size_2d(0) ||
            coord_pos(1) < origin(1) || coord_pos(1) >= map_size_2d(1))
        {
            return false;
        }
        // check collision
        if (edt_environment->sdf_map->getInflatOccupancy(coord_pos) == 1)
        {
            return false;
        }

    }

    coef_shot = coef;
    t_shot = t_d;
    is_shot_succ = true;
    return true;

}






















}