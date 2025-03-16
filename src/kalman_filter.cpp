

#include "kalman_filter.h"

kalman::kalman(float h_){

    h = h_;

    B = h;

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



}


void kalman::verbose(){


    Serial.println("------- Printing Kalman filter info -----");
    Serial.print("State update A: ");Serial.print(A);Serial.print(" State update B: ");Serial.println(B);
    Serial.print("Predicted Covariance: ");Serial.print(P_pred);Serial.print("Kalman Gain: ");Serial.println(K);



}