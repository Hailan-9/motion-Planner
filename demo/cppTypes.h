/**
 * @file cppTypes.h
 * @brief Actiger工程中的多维变量类型模板的定义
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef CPPTYPES_H
#define CPPTYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <Eigen/core>

/*********************************************************向量*********************************************************/
/** @brief 2 x 1 向量 */
template <typename T>
using Vec2 = Eigen::Matrix<T, 2, 1>;

/** @brief 3 x 1 向量 */
template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

/** @brief 4 x 1 向量 */
template <typename T>
using Vec4 = Eigen::Matrix<T, 4, 1>;

/** @brief 5 x 1 向量 */
template <typename T>
using Vec5 = Eigen::Matrix<T, 5, 1>;

/** @brief 6 x 1 向量 */
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

/** @brief 8 x 1 向量 */
template <typename T>
using Vec8 = Eigen::Matrix<T, 8, 1>;

/** @brief 9 x 1 向量 */
template <typename T>
using Vec9 = Eigen::Matrix<T, 9, 1>;

/** @brief 10 x 1 向量 */
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

/** @brief 11 x 1 向量 */
template <typename T>
using Vec11 = Eigen::Matrix<T, 11, 1>;

/** @brief 12 x 1 向量 */
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

/** @brief 13 x 1 向量 */
template <typename T>
using Vec13 = Eigen::Matrix<T, 13, 1>;

/** @brief 18 x 1 向量 */
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

/** @brief 19 x 1 向量 */
template <typename T>
using Vec19 = Eigen::Matrix<T, 19, 1>;

/** @brief 20 x 1 向量 */
template <typename T>
using Vec20 = Eigen::Matrix<T, 20, 1>;

/** @brief 23 x 1 向量 */
template <typename T>
using Vec23 = Eigen::Matrix<T, 23, 1>;

/** @brief 28 x 1 向量 */
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

/** @brief 四元数向量 */
template <typename T>
using Quat = typename Eigen::Quaternion<T>;

/** @brief 动态向量 */
template <typename T>
using DVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

/*********************************************************矩阵*********************************************************/
/** @brief 2 x 2 矩阵 */
template <typename T>
using Mat2 = Eigen::Matrix<T, 2, 2>;

/** @brief 2 x 13 矩阵 */
template <typename T>
using Mat2x13 = Eigen::Matrix<T, 2, 13>;

/** @brief 3 x 3 矩阵 */
template <typename T>
using Mat3 = Eigen::Matrix<T, 3, 3>;

/** @brief 3 x 6 矩阵 */
template <typename T>
using Mat3x6 = Eigen::Matrix<T, 3, 6>;

/** @brief 3 x 18 矩阵 */
template <typename T>
using Mat3x18 = Eigen::Matrix<T, 3, 18>;

/** @brief 4 x 4 矩阵 */
template <typename T>
using Mat4 = Eigen::Matrix<T, 4, 4>;

/** @brief 4 x 8 矩阵 */
template <typename T>
using Mat4x8 = Eigen::Matrix<T, 4, 8>;

/** @brief 5 x 3 矩阵 */
template <typename T>
using Mat5x3 = Eigen::Matrix<T, 5, 3>;

/** @brief 6 x 6 矩阵 */
template <typename T>
using Mat6 = Eigen::Matrix<T, 6, 6>;

/** @brief 6 x 3 矩阵 */
template <typename T>
using Mat6x3 = Eigen::Matrix<T, 6, 3>;

/** @brief 8 x 4 矩阵 */
template <typename T>
using Mat8x4 = Eigen::Matrix<T, 8, 4>;

/** @brief 8 x 8 矩阵 */
template <typename T>
using Mat8 = Eigen::Matrix<T, 8, 8>;

/** @brief 11 x 13 矩阵 */
template <typename T>
using Mat11x13 = Eigen::Matrix<T, 11, 13>;

/** @brief 12 x 12 矩阵 */
template <typename T>
using Mat12 = Eigen::Matrix<T, 12, 12>;

/** @brief 13 x 13 矩阵 */
template <typename T>
using Mat13 = Eigen::Matrix<T, 13, 13>;

/** @brief 13 x 12 矩阵 */
template <typename T>
using Mat13x12 = Eigen::Matrix<T, 13, 12>;

/** @brief 18 x 18 矩阵 */
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

/** @brief 18 x 3 矩阵 */
template <typename T>
using Mat18x3 = Eigen::Matrix<T, 18, 3>;

/** @brief 18 x 13 矩阵 */
template <typename T>
using Mat18x13 = Eigen::Matrix<T, 18, 13>;

/** @brief 18 x 28 矩阵 */
template <typename T>
using Mat18x28 = Eigen::Matrix<T, 18, 28>;

/** @brief 20 x 12 矩阵 */
template <typename T>
using Mat20x12 = Eigen::Matrix<T, 20, 12>;

/** @brief 23 x 12 矩阵 */
template <typename T>
using Mat23x12 = Eigen::Matrix<T, 23, 12>;

/** @brief 28 x 28 矩阵 */
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

/** @brief 28 x 18 矩阵 */
template <typename T>
using Mat28x18 = Eigen::Matrix<T, 28, 18>;

/** @brief 动态矩阵 */
template <typename T>
using DMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Eigen::MatrixXd是一个动态大小的矩阵类型，它继承自Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>。
// 因此，可以使用Eigen::Matrix中定义的所有函数和操作符来操作MatrixXd对象。
typedef typename Eigen::MatrixXd MatXd;

// Eigen::VectorXd 是 Eigen 库中表示动态大小列向量的类模板。
// 它是 Eigen::Matrix<double, Eigen::Dynamic, 1> 的别名，
// 其中 Eigen::Dynamic 表示动态大小。因此，Eigen::VectorXd 可以看作是一个动态大小的列向量，元素类型为 double。
typedef typename Eigen::VectorXd VecXd;


#endif // ！CPPTYPES_H