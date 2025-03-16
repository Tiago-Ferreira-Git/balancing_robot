#ifndef _constants_H_
#define _constants_H_



#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "Arduino.h"
#include "motor.h"
#include "mpu6050.h"
#include "kalman_filter.h"


#define PS_PIN 23 // Power Save Pin, H to disable, L default

const float motor_steps = 0.065;


volatile bool timer_fired {false};


//the interrupt service routine
struct repeating_timer timer;

bool bit = 0;

volatile int check_rotation = 0;

volatile long int next = 0;


static int h = 10; //sampling time in ms

// IN1 gpio 20
// IN2 gpio 21
#define motor_a 20
#define motor_b 21

int encoder_a  = 16;
int encoder_b  = 17;


motor motor_right(2,3,encoder_a,encoder_b);
motor motor_left(0,1,6,7);

mpu6050 mpu(0x68);

kalman roll((float) h/1000),pitch((float) h/1000);


unsigned long prev = 0;


spin_lock_t *slk {0};


#endif


