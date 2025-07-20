#ifndef _kalman_filter_H_
#define _kalman_filter_H_

#include "Arduino.h"
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;



class kalman
{
private:
    // State transition matrix
    MatrixXd A;
    MatrixXd A_t;
    // Control input matrix
    MatrixXd B;
    // Output matrix
    MatrixXd C;
    MatrixXd C_t;
    // Process noise covariance matrix
    MatrixXd Q;
    // Measurement noise covariance matrix
    MatrixXd R;
    // State covariance matrix
    MatrixXd P;
    // Kalman gain matrix
    MatrixXd L;
    // Measurement covariance matrix
    MatrixXd P_meas;
    MatrixXd P_meas_inv;
    
    MatrixXd I;
    
public:
    // Predicted state vector
    MatrixXd x_pred;


    /*
        * @brief Constructor for class kalman: assigns the parameters to the private variables of the class.
        *
        * @param A State transition matrix
        * @param B Control input matrix
        * @param Q Process noise covariance matrix
        * @param R Measurement noise covariance matrix
    */
    explicit kalman(MatrixXd,MatrixXd,MatrixXd,MatrixXd,MatrixXd );
    ~kalman(){}

    /*
        * @brief Predicts the next state and covariance based on the control input.
        *
        * @param u Control input vector
    */
    void predict(MatrixXd u);

    /*
        * @brief Updates the state estimate and covariance using the measurement.
        *
        * @param y Measurement vector
    */
    void update(MatrixXd y);

    /*
        * @brief Prints the current Kalman filter matrices and state for debugging.
    */
    void verbose();

};



#endif