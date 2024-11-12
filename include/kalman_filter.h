#ifndef _motor_H_
#define _motor_H_


#include "Arduino.h"

class motor
{
private:
    
    
public:
    uint8_t IN = 0; //pwm pin to control motor speed that 
    uint8_t encoder = 0; //encoder pin to measure speed
    volatile float prev = 0;
    volatile float vel = 0;
    /*
        * @brief Costructor for class motor: atributes the parameters when the class is called to the private variables of the class
    */
    explicit motor(uint8_t IN1_,uint8_t encoder_);
    ~motor(){}
    /*
        * @brief Sets direction of motor
        *
        * @param The Pico pin corresponding to the IN1 H motor bridge pin
        * @param The Pico pin corresponding to the IN2 H motor bridge pin
        
    */
    void set_direction(float ,float);
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