

// #include "math_function.hpp"
// /*
// 贝塞尔曲线
// 1.参考文献
// https://blog.csdn.net/sinat_35676815/article/details/120884682
// https://blog.csdn.net/younggung/article/details/128755855
// */

// /**
//  * @brief   描述
//  * 
//  * @author  hailan(2522082924@qq.com)
//  * @date    2023-07-17
//  */
// /*


// */

// /** n是贝塞尔曲线的阶次，n次贝塞尔曲线，需要n+1个控制点*/
// for(unsigned int i(0);i<=n;i++)
// {
//     B_t += p(i)*pow(t,i)*pow(1-t,n-i)*(double)(Factorial(n)/Factorial(i)/Factorial(n-i))
// }

// /** 贝塞尔曲线转成普通多项式曲线，系数转换，有通项公式，可以推导，但是考虑到5次多项式和7次多项式较为通用，故暂且不推导公式*/
// // a*(1-t)^5+5*b*t*(1-t)^4+10*c*t^2*(1-t)^3+10*d*t^3*(1-t)^2+5*e*t^4*(1-t)+f*t^5
// bezier2polynomial_Matrix_five_order
// bezier2polynomial_Matrix_five_order<<
// 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// -5.0, 5.0, 0.0, 0.0, 0.0, 0.0,
// 10.0, -20.0, 10.0, 0.0, 0.0, 0.0, 
// -10.0, 30.0, -30.0, 10.0, 0.0, 0.0,
// 5.0, -20.0, 30.0, -20.0, 5.0, 0.0,
// -1.0, 5.0, -10.0, 10.0, -5.0, 1.0;
// // a*(1-t)^7+7*b*t*(1-t)^6+21*c*t^2*(1-t)^5+35*d*t^3*(1-t)^4+35*e*t^4*(1-t)^3+21*f*t^5*(1-t)^2+7*g*t^6*(1-t)+h*t^7
// bezier2polynomial_Matrix_seven_order<<
// 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// -7.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 21.0, -42.0, 21.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// -35.0, 105.0, -105.0, 35.0, 0.0, 0.0,0.0, 0.0,
// 35.0, -140.0, 210.0, -140.0, 35.0, 0.0, 0.0, 0.0,
// -21.0, 105.0, -210.0, 210.0, -105.0, 21.0, 0.0, 0.0,
// 7.0, -42.0, 105.0, -140.0, 105.0, -42.0, 7.0, 0.0,
// -1.0, 7.0, -21.0, 35.0, -35.0, 21.0, -7.0, 1.0;

