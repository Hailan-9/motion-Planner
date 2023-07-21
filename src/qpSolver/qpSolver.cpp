/**
 * @file qpSolver.cpp
 * @brief Actiger二次规划求解器QpSolver
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "qpSolver.hpp"
#include <iostream>
#include <thread>

#include <iostream>
#include <fstream>


using namespace std;

/**
 * @brief 判断输入的参数是否可以认为约等于0
 * @param  val  待判断参数            
 * @return uint8_t 
 */
uint8_t NearZero(double val)
{
    return (val < 1e-10 && val > -1e-10);
}
 
/**
 * @brief QpSolver类的构造函数
 */
QpSolver::QpSolver(void)
{
    cout << "QpSolver Init" << endl;
}

/**
 * @brief QpSolver类的析构函数
 */
QpSolver::~QpSolver(void) {}

/**
 * @brief OSQP接口函数
 */
void QpSolver::Free(void)
{
    c_free(h_data);
    c_free(h_indices);
    c_free(h_indptr);
    c_free(a_data);
    c_free(a_indices);
    c_free(a_indptr);
    c_free(q_data);
    c_free(l_data);
    c_free(u_data);
}

/**
 * @brief OSQP接口函数
 * @param  H_matrix         
 */
void QpSolver::CalculateDataH(DMat<double> H_matrix)
{
    std::vector<c_float> H_data;
    std::vector<c_int> H_indices;
    std::vector<c_int> H_indptr;
    //cout << "qp_H" << endl;
    //cout << qp_H << endl;

    int ind_h = 0;
    for (int i = 0; i < H_matrix.cols(); i++)
    {
        (&H_indptr)->emplace_back(ind_h);
        for (int j = 0; j < H_matrix.rows(); j++)
        {
            if (!NearZero(H_matrix(j, i)))
            {
                (&H_indices)->emplace_back(j);
                (&H_data)->emplace_back(H_matrix( j, i));
                ind_h++;
            }
        }
    }
    (&H_indptr)->emplace_back(ind_h);
    
    h_data = (c_float*)c_malloc(sizeof(c_float) * H_data.size());
    h_indices = (c_int*)c_malloc(sizeof(c_int) * H_indices.size());
    h_indptr = (c_int*)c_malloc(sizeof(c_int) * H_indptr.size());

    size_H = H_data.size();
    //cout << "h_data" << endl;
    for (int i = 0; i < H_data.size(); i++)
    {
        h_data[i] = H_data[i];
        //cout << h_data[i] << endl;
    }
    //cout << "h_indices" << endl;
    for (int i = 0; i < H_indices.size(); i++)
    {
        h_indices[i] = H_indices[i];
        //cout << h_indices[i] << endl;
    }
    //cout << "h_indptr" << endl;
    for (int i = 0; i < H_indptr.size(); i++)
    {
        h_indptr[i] = H_indptr[i];
        //cout << h_indptr[i] << endl;
    }
       
}

/**
 * @brief OSQP接口函数
 * @param  G_vector         
 */
void QpSolver::CalculateDataG(DVec<double> G_vector)
{
    q_data = (c_float*)c_malloc(sizeof(c_float) * G_vector.size());

    //cout << "q_data" << endl;
    for (int i = 0; i < G_vector.size(); i++)
    {
        q_data[i] = G_vector(i);
       //cout << q_data[i] << endl;
    }
       
}

/**
 * @brief OSQP接口函数
 * @param  A_matrix         
 */
void QpSolver::CalculateDataA(DMat<double> A_matrix)
{
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;

    //cout << "qp_A" << endl;
    //cout << qp_A << endl;

    int ind_a = 0;
    for (int i = 0; i < A_matrix.cols(); i++)
    {
        (&A_indptr)->emplace_back(ind_a);
        for (int j = 0; j < A_matrix.rows(); j++)
        {
            if (!NearZero(A_matrix(j, i)))
            {
                (&A_indices)->emplace_back(j);
                (&A_data)->emplace_back(A_matrix(j, i));
                ind_a++;
            }
        }
    }
    (&A_indptr)->emplace_back(ind_a);

    a_data = (c_float*)c_malloc(sizeof(c_float) * A_data.size());
    a_indices = (c_int*)c_malloc(sizeof(c_int) * A_indices.size());
    a_indptr = (c_int*)c_malloc(sizeof(c_int) * A_indptr.size());

    size_A = A_data.size();
    //cout << "a_data" << endl;
    for (int i = 0; i < A_data.size(); i++)
    {
        a_data[i] = A_data[i];
        //cout << a_data[i] << endl;
    }
    //cout << "a_indices" << endl;
    for (int i = 0; i < A_indices.size(); i++)
    {
        a_indices[i] = A_indices[i];
       //cout << a_indices[i] << endl;

    }
    //cout << "a_indptr" << endl;
    for (int i = 0; i < A_indptr.size(); i++)
    {
        a_indptr[i] = A_indptr[i];
        //cout << a_indptr[i] << endl;
    }
}

/**
 * @brief OSQP接口函数
 * @param  L_vector         
 * @param  U_vector         
 */
void QpSolver::CalculateDataLandU(DVec<double> L_vector, DVec<double> U_vector)
{
    l_data = (c_float*)c_malloc(sizeof(c_float) * L_vector.size());
    u_data = (c_float*)c_malloc(sizeof(c_float) * U_vector.size());

    //cout << "l_data" << endl;
    for (int i = 0; i < L_vector.size(); i++)
    {
        l_data[i] = L_vector(i);
        //cout << l_data[i] << endl;
    }
    //cout << "u_data" << endl;
    for (int i = 0; i < U_vector.size(); i++)
    {
        
        u_data[i] = U_vector(i);
        //cout << u_data[i] << endl;
    }   
}

/**
 * @brief OSQP接口函数
 * @param  H_matrix         
 * @param  G_vector         
 * @param  A_matrix         
 * @param  L_vector         
 * @param  U_vector         
 * @return DVec<double> 
 */
DVec<double> QpSolver::Solve(DMat<double> H_matrix, DVec<double> G_vector,
                           DMat<double> A_matrix, DVec<double> L_vector, DVec<double> U_vector)
{
    DVec<double> return_data;

    OSQPWorkspace* work = nullptr;
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));

    size_A = 0;
    size_H = 0;

    CalculateDataH(H_matrix);

    CalculateDataG(G_vector);

    CalculateDataA(A_matrix);

    CalculateDataLandU(L_vector, U_vector);

    return_data.resize(A_matrix.cols(),1);
    
    // std::cout<<A_matrix <<std::endl;
    // std::cout<<"--------------------H_matrix-----------------------" <<std::endl;
    // for(int i=0;i<H_matrix.rows();i++)
    // {
    //     for(int j=0;j<H_matrix.cols();j++)
    //     {
    //         std::cout<<H_matrix(i,j) <<" , ";
    //     }
    //     std::cout<<endl;
    // }
    
        
    // std::cout<<"-------------------------------------------" <<std::endl;
    // std::cout<<L_vector <<std::endl;
    // std::cout<<"-------------------------------------------" <<std::endl;
    // std::cout<<U_vector <<std::endl;
    // std::cout<<"-------------------------------------------" <<std::endl;

    // std::cout<<G_vector <<std::endl;
    // std::cout<<"-------------------------------------------" <<std::endl;

    c_int p_row = A_matrix.cols();
    c_int a_row = A_matrix.rows();

    if (data) {
        data->n = p_row;
        data->m = a_row;
        data->P = csc_matrix(data->n, data->n, size_H, h_data, h_indices, h_indptr);
        data->q = q_data;
        data->A = csc_matrix(data->m, data->n, size_A, a_data, a_indices, a_indptr);
        data->l = l_data;
        data->u = u_data;
    }
    if (settings != nullptr)
    {
        osqp_set_default_settings(settings);
        settings->polish = true;
        settings->scaled_termination = true;
        settings->verbose = false;
        settings->max_iter = 20000;
        settings->eps_abs = 0.000001;
        settings->eps_rel = 0.000001;
    }
    work = osqp_setup(data, settings);

    osqp_solve(work);
    auto status = work->info->status_val;

    if (status < 0 || (status != 1 && status != 2))
    {
        cout << "failed optimization status:\t" << work->info->status << endl;
    }
    else if (work->solution == nullptr)
    {
        cout << "The solution from OSQP is nullptr" << endl;
    }

    for (size_t i = 0; i < A_matrix.cols(); ++i) {
        return_data(i) = work->solution->x[i];
        
    }

    cout <<"iternum "<<work->info->iter<<" time: "<<work->info->solve_time*1000.0<<std::endl;


    Free();
    // Cleanup
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
    c_free(work);
    
    // std::cout<<return_data <<std::endl;
    // std::cout<<"-------------------------------------------" <<std::endl;
    return return_data;
}