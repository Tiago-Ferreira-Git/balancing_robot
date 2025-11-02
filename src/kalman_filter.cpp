

#include "kalman_filter.h"



kalman::kalman(MatrixXd  A_, MatrixXd B_, MatrixXd C_, MatrixXd Q_, MatrixXd R_){

    A = A_;
    A_t = A_.transpose();
    B = B_;
    Q = Q_;
    R = R_;
    C = C_;
    C_t = C_.transpose();
    I = MatrixXd::Identity(A_.rows(), A_.rows());
    x_pred = MatrixXd::Zero(A_.rows(),1);
    P = MatrixXd::Zero(A_.rows(), A_.rows());
    L = MatrixXd::Zero(C.cols(), C.rows());

}


void kalman::predict(MatrixXd u){

    x_pred = A*x_pred+ B*u;

    P = A*P*A_t + Q;

}



void kalman::update(MatrixXd y){

    MatrixXd error = y-C*x_pred;

    P_meas = C*P*C_t + R;

    L = P*C_t*P_meas.inverse();

    x_pred = x_pred + L*error;

    P = (I-L*C)*P;
}


void kalman::verbose(){


    // Serial.println("------- Printing Kalman filter info -----");
    // Serial.println("Printing P: ");
    // for(int i = 0 ; i < P_meas.rows(); i++){
    //     for(int j = 0 ; j < P_meas.cols(); j++){

    //         Serial.print(P(i,j));
    //         Serial.print(" ");

    //     }
    //     Serial.println("");
    //}
    

}