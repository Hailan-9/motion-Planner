/**
 * @file qpSolver.hpp
 * @brief Actiger二次规划求解器QpSolver头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP

#include <osqp.h>
#include <vector> 
#include <math.h>

#include "cppTypes.h"

// namespace quadruped

    /**
     * @brief Actiger二次规划求解器
     */
    class QpSolver
    {
    private:
        void Free(void);
        void CalculateDataH(DMat<double> H_matrix);
        void CalculateDataG(DVec<double> G_vector);
        void CalculateDataA(DMat<double> A_matrix);
        void CalculateDataLandU(DVec<double> L_vector, DVec<double> U_vector);

        c_float* h_data;
        c_int* h_indices;
        c_int* h_indptr;
        c_float* a_data;
        c_int* a_indices;
        c_int* a_indptr;
        c_float* q_data;
        c_float* l_data;
        c_float* u_data;

        c_int size_H = 0;
        c_int size_A = 0;

    protected:

    public:
        explicit QpSolver(void);
        virtual ~QpSolver();
        
        DVec<double> Solve(DMat<double> H_matrix, DVec<double> G_vector,
                           DMat<double> A_matrix, DVec<double> L_vector, DVec<double> U_vector);
    };


uint8_t NearZero(double val);

#endif // !QPSOLVER_HPP