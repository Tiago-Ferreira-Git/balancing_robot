#ifndef _motor_H_
#define _motor_H_

#include "Arduino.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/time.h"
class motor
{
private:
    
    
public:
    uint8_t IN_1 = 0; //pwm pin to control motor speed that 
    uint8_t IN_2 = 0; //pwm pin to control motor speed that 
    uint8_t encoder_a = 0; //encoder pin to measure speed
    uint8_t encoder_b = 0; //encoder pin to measure speed
    uint8_t pwm = 0; //pwm pin to set velocity

    double vel = 0;
    volatile long  int next = 0;
    volatile long int prev = 0;
    volatile long int diff = 0;
    /*
        * @brief Costructor for class motor: atributes the parameters when the class is called to the private variables of the class
    */
    explicit motor(uint8_t ,uint8_t ,uint8_t ,uint8_t, uint8_t );
    ~motor(){}
    /*
        * @brief Sets direction of motor
        *
        * @param The Pico pin corresponding to the IN1 H motor bridge pin
        * @param The Pico pin corresponding to the IN2 H motor bridge pin
        
    */
    void set_direction(bool);

    /*
        * @brief Call back to measure speed
        *
        * @param The gpio that trigers the interruption
        * @param The event that triggers the interruption
        
    */
   void encoder_callback(uint , uint32_t );
    /*
    */
   void set_speed(float );


};



#endif