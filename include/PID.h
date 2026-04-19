#ifndef _PID_H_
#define _PID_H_

#include <stdio.h>
#include <Eigen/Dense>
#include "pico/stdlib.h"
#include <cmath>

using Eigen::Matrix;
using Eigen::MatrixXd;

class PID
{
private:
  Matrix<double, 2, 3> K = Matrix<double, 2, 3>::Zero();
  Matrix<double, 3, 2> error = Matrix<double, 3, 2>::Zero();
  double h = 0; // time-step between control actions
  Matrix<double, 2, 1> limits = Matrix<double, 2, 1>::Zero();

public:
  /*
   * @brief Costructor for class PID: atributes the parameters when the class is
   * called to the private variables of the class
   */
  explicit PID(Matrix<double, 2, 3> , double ,
               Matrix<double, 2, 1> );
  ~PID() {}

  /*
   * @brief Compute the control signal: sums all the previous terms and clips
   * the control signal (maximum is 4095 and minimum is 0)
   *
   * @param REF reference value (in LUX)
   * @param y value measured (in LUX)
   *
   *
   * @returns The computed control signal
   */
  Matrix<double, 2, 1> control(Matrix<double, 2, 1> , Matrix<double, 2, 1>);
};

#endif