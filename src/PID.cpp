
#include "PID.h"


PID::PID(float K_, float Ti_, float Td_,float N_, float b_, float h_ , float Tt_, float limit_inf_ , float limit_sup_){
  K = K_;
  Ti = Ti_;
  Td = Td_; 
  N = N_;
  b = b_;
  h = h_;
  Tt = Tt_;
  ad = Td/(Td+N*h);
  bd = Td*K*N/(Td+N*h);
  limit_inf = limit_inf_;
  limit_sup = limit_sup_;
}

float PID::proporcional(float error){
  return K*error;
}


float PID::integrator(float error,float saturation_error){
  if(anti_windup) return h* (error * K/Ti + saturation_error/Tt);
  return h* error * K/Ti;
}

float PID::derivative(float y){
  D = ad*D-bd*(y-y_old);
  return  D;
}

float PID::control(float REF, float sensor_value){
  float error = 0,saturation_error = 0;
  float u = 0,v = 0;
  

  error = REF - sensor_value;
  //printf("%f",error);printf("\t");
  P = proporcional(error);
  D = derivative(sensor_value);
  v = P + I;
  // + D;

  if( v < -limit_inf ) u = limit_inf;
  else if( v > limit_sup ) u = limit_sup;
  else u = v;
  saturation_error = u-v;
  I += integrator(error,saturation_error); //back calculation slide 19
  y_old = sensor_value;
  // P = u;
  // D = v;

  //u = map(u, -1500, 1500, 0, 1000);


  return u;
}

void PID::set_anti_windup(bool to_set){
  anti_windup = to_set;
}

int PID::get_anti_windup_state(){
  return anti_windup;
}

void PID::print_pid_values(){
  printf("%f",P);printf(" ");
  printf("%f",I);printf(" ");
  printf("%f \n",D);
}

void PID::print_gains(){
  printf("%f",K);printf(" ");
  printf("%f",K/Ti);printf(" ");
  printf("%f \n",ad);
}


void PID::set_gains(double K_, double Ti_, double ad_){
  K = K_;
  Ti = Ti_;
  ad = ad_;
}
