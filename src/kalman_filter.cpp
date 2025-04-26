

#include "kalman_filter.h"

kalman::kalman(float A_,float B_,float Q_,float R_){

    //h = h_;
    A = A_;
    B = B_;
    Q = Q_;
    R = R_;

}


void kalman::predict(float u){

    x_pred = A*x_pred+ B*u;

    P_pred = A*P_pred*A + Q;

}



void kalman::update(float y){

    float error = y-x_pred;

    P_meas = P_pred + R;

    K = P_pred/P_meas;

    x_pred = x_pred + K*error;

    P_pred = (1-K)*P_pred;

    x = x_pred;

    //Serial.print("Printing Kalman filter Error:");Serial.println(y-x_pred);



}


void kalman::verbose(){


    Serial.println("------- Printing Kalman filter info -----");
    Serial.print("State update A: ");Serial.print(A);Serial.print(" State update B: ");Serial.println(B);
    Serial.print("Predicted Covariance: ");Serial.print(P_pred);Serial.print("Kalman Gain: ");Serial.println(K);



}