#ifndef _matrix_def_H_
#define _matrix_def_H_

#include <iostream>
#include <Eigen/Dense>

// New system matrices

const double A_data[4] = {
    0.997492557719805, -0.287963170704879,
    0.0723445978980524, 0.719866078472963
};
const Eigen::Matrix<double, 2, 2> A(A_data);

const double B_data[2] = {
    -0.00241850623056217,
    -0.00268609419023186
};
const Eigen::Matrix<double, 2, 1> B(B_data);

const Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 1e-3;

const Eigen::Matrix<double, 1, 1> R(1);

const double C_data[2] = {
    495.872774382649, -406.557367598786
};
const Eigen::Matrix<double, 1, 2> C(C_data);



#endif