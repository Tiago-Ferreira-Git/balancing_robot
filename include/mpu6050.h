#ifndef _mpu6050_H_
#define _mpu6050_H_

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "Arduino.h"


class mpu6050
{
private:
    float gyro_drift[3] = {0.0,0.0,0.0}; 
    float accel_drift[3] = {0.0,0.0,0.16};   
public:
    uint8_t i2c_addr = 0x68; //i2c adress default: 0x68
    float gyro[3] = {0.0,0.0,0.0};
    float accel[3] = {0.0,0.0,0.0};
    float angle[3] = {0.0,0.0,0.0};

    /*
        * @brief Costructor for class mpu6050: atributes the parameters when the class is called to the private variables of the class.
    */
    explicit mpu6050(uint8_t addr_);
    ~mpu6050(){}
    /*
        * @brief Resets the MPU6050 chip.
        *
        * @param The Pico pin corresponding to the IN1 H motor bridge pin
        * @param The Pico pin corresponding to the IN2 H motor bridge pin
        
    */
    void reset();
    /*

        * @brief Writes the value "val" in the register "register".
        *
        * @param Register to be written
        * @param Value to be written
        
    */
    void write(uint8_t , uint8_t );

    /*

        * @brief Read accelerometer and gyroscope data.
        
    */
    void read_values();

    /*

        * @brief Read the values from the register "register" and keep the values in the "value" argument.
        *
        * @param Register to be read
        * @param Variable to be written
        
    */
    void read(uint8_t , uint8_t,uint8_t * );
    /*

        * @brief Obtain the offset of each sensor. If the flag is true, the calibration requires user to set the sensor oriented in n directions (calibration is done externally).
        *
        * @param Register to be read
        * @param Variable to be written
        
    */
    void calibration(int n,bool flag_accel);

    void std(int n,bool flag_accel);
    uint8_t debug(uint8_t );
    /*
        * @brief Compute the integrator term of the controller
        *
        * @param error the difference between reference and the value measured
        * @param saturation_error difference between clipped control signal and the "real" control signal (if anti_windup is true)
        * 
        * @returns The value of integrator term with set point weighting (uses parameters anti_windup, K , Ti, Tt )
    */

};



#endif