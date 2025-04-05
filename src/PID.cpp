
#include "PID.h"


PID::PID(float K_, float Ti_, float Td_,float N_, float b_, float h_ , float Tt_){
  K = K_;
  Ti = Ti_;
  Td = Td_; 
  N = N_;
  b = b_;
  h = h_;
  Tt = Tt_;
  ad = Td/(Td+N*h);
  bd = Td*K*N/(Td+N*h);
}

float PID::proporcional(float REF,float y){
  return K*(REF-y);
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
  
  //D = derivative(sensor_value);
  P = proporcional(REF,sensor_value);
  v = P + I;
  // + D;

  if( v < -2000 ) u = -2000;
  else if( v > 2000 ) u = 2000;
  else u = v;
  saturation_error = u-v;
  I += integrator(error,saturation_error); //back calculation slide 19
  // y_old = sensor_value;
  // P = u;
  // D = v;

  u = map(u, -2000, 2000, 0, 1000);
  return u;
}

void PID::set_anti_windup(bool to_set){
  anti_windup = to_set;
}

int PID::get_anti_windup_state(){
  return anti_windup;
}

void PID::print_pid_values(){
  Serial.print(P);Serial.print(" ");
  Serial.print(I);Serial.print(" ");
  Serial.print(D);Serial.print(" ");
}

void PID::print_gains(){
  Serial.print(K);Serial.print(" ");
  Serial.print(K/Ti);Serial.print(" ");
  Serial.print(ad);Serial.print(" ");
}


void PID::set_gains(double K_, double Ti_, double ad_){
  K = K_;
  Ti = Ti_;
  ad = ad_;
}
