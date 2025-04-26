#ifndef _kalman_filter_H_
#define _kalman_filter_H_

#include "Arduino.h"


class kalman
{
private:
    //model
    float A = 0;
    float B = 0;

    float Q = 0;
    float R = 0;
public:
    float x_pred = 0.0;
    float h = 0.0; //sampling time
    float K = 0.0; 
    float P_pred = 10000.0;
    float P_meas = 10000.0;
    float x = 0.0;

    /*
        * @brief Costructor for class mpu6050: atributes the parameters when the class is called to the private variables of the class.
    */
    explicit kalman(float,float,float,float );
    ~kalman(){}
    /*
        * @brief Resets the MPU6050 chip.
        *
        * @param The Pico pin corresponding to the IN1 H motor bridge pin
        * @param The Pico pin corresponding to the IN2 H motor bridge pin
        
    */
    void predict(float u);
    /*

        * @brief Writes the value "val" in the register "register".
        *
        * @param Register to be written
        * @param Value to be written
        
    */
    void update(float y);


    void verbose();

};



#endif