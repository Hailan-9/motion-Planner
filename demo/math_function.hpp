#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H
#include "cppTypes.h"
/**
 * @brief   描述
 * 
 * @param   x           参数描述
 * @return  double 
 * @author  hailan(https://github.com/Hailan-9)
 * @date    2023-07-18
 */
inline double Factorial(unsigned int x) 
{
    unsigned int fac = 1;

    for (unsigned int i = x; i > 0; --i) {
        fac = fac * i;
    }

    return static_cast<double>(fac);
}

inline DMat<double> bezier2polynomial_Matrix(const unsigned int &order)
{
    int num;
    num = order+1;
    DMat<double> matrix(num,num);
    /** 贝塞尔曲线转成普通多项式曲线，系数转换，有通项公式，可以推导，但是考虑到5次多项式和7次多项式较为通用，故暂且不推导公式*/
    if(num == 6)
    {
        // a*(1-t)^5+5*b*t*(1-t)^4+10*c*t^2*(1-t)^3+10*d*t^3*(1-t)^2+5*e*t^4*(1-t)+f*t^5
        matrix<<
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -5.0, 5.0, 0.0, 0.0, 0.0, 0.0,
        10.0, -20.0, 10.0, 0.0, 0.0, 0.0, 
        -10.0, 30.0, -30.0, 10.0, 0.0, 0.0,
        5.0, -20.0, 30.0, -20.0, 5.0, 0.0,
        -1.0, 5.0, -10.0, 10.0, -5.0, 1.0;
    }
    if(num == 8)
    {
        // a*(1-t)^7+7*b*t*(1-t)^6+21*c*t^2*(1-t)^5+35*d*t^3*(1-t)^4+35*e*t^4*(1-t)^3+21*f*t^5*(1-t)^2+7*g*t^6*(1-t)+h*t^7
        matrix<<
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -7.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        21.0, -42.0, 21.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -35.0, 105.0, -105.0, 35.0, 0.0, 0.0,0.0, 0.0,
        35.0, -140.0, 210.0, -140.0, 35.0, 0.0, 0.0, 0.0,
        -21.0, 105.0, -210.0, 210.0, -105.0, 21.0, 0.0, 0.0,
        7.0, -42.0, 105.0, -140.0, 105.0, -42.0, 7.0, 0.0,
        -1.0, 7.0, -21.0, 35.0, -35.0, 21.0, -7.0, 1.0;
    }
    return matrix;
}


#endif //TRAJECTORY_GENERATOR_MATH_FUNCTION_H