
#include "PID.h"

PID::PID(Matrix<double, 2, 3>  K_, double h_, Matrix<double, 2, 1> limits_)
{
  K = K_;
  h = h_;
  limits = limits_;

}


Matrix<double, 2, 1> PID::control(Matrix<double, 2, 1> ref, Matrix<double, 2, 1> meas)
{
  Matrix<double, 2, 1> u = Matrix<double, 2, 1>::Zero();
  Matrix<double, 2, 2> v = Matrix<double, 2, 2>::Zero();
  Matrix<double, 2, 1> last_error = Matrix<double, 2, 1>::Zero();


  last_error(0,0) = error(0,0);
  last_error(1,0) = error(0,1);

  error(0,0) = ref(0,0) - meas(0,0);
  error(1,0) += error(0,0)*h;
  error(2,0) = (error(0,0)-last_error(0,0))/h;

  error(0,1) = ref(1,0) - meas(1,0);
  error(1,1) += error(0,1)*h;
  error(2,1) = (error(0,1)-last_error(1,0))/h;

  v = K*error;

  // while(true){
  // printf(" %.3f\t", h);
  // printf(" %.3f\t", meas(0,0));
  // printf(" %.3f\t", error(0,0));
  // printf(" %.3f\t", error(1,0));
  // printf(" %.3f\t", error(2,0));
  // printf(" %.3f\t", meas(1,0));
  // printf(" %.3f\t", error(0,1));
  // printf(" %.3f\t", error(1,1));
  // printf(" %.3f\n", error(2,1));
  //   sleep_ms(1);
  // }

  
  v(0,0) = std::max(v(0,0),limits(0,0));
  v(0,0) = std::min(v(0,0),limits(1,0));

  v(1,1) = std::max(v(1,1),limits(0,0));
  v(1,1) = std::min(v(1,1),limits(1,0));

  u(0,0) = int(std::round(v(0,0)));
  u(1,0) = int(std::round(v(1,1)));

  return u;
}

