#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H

inline double Factorial(unsigned int x) 
{
    unsigned int fac = 1;

    for (unsigned int i = x; i > 0; --i) {
        fac = fac * i;
    }

    return static_cast<double>(fac);
}

#endif //TRAJECTORY_GENERATOR_MATH_FUNCTION_H