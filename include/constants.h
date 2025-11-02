#ifndef _constants_H_
#define _constants_H_



#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "motor.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "average_moving_filter.h"
#include "PID.h"
#include <iostream>
#include <Eigen/Dense>
#include "matrix_def.h"


using namespace std;
using namespace Eigen;

#define PS_PIN 23 // Power Save Pin, H to disable, L default
#define WHEEL_RADIUS 0.025


const float motor_steps = 0.065;

volatile bool time_to_sample_core0 {false};
volatile bool time_to_sample_core1 {false};


//the interrupt service routine
struct repeating_timer timer_core0;
struct repeating_timer timer_core1;

bool bit = 0;

volatile int check_rotation = 0;

absolute_time_t next_time = 0;
absolute_time_t prev_time = 0;


static int h = 1; //sampling time in ms
static int h_outer = h;
double time_stamp = 0;


motor motor_left(21,22,17,16,9);
motor motor_right(11,13,6,7,10);
kalman estimator(A, B, C , Q , R);
average_moving_filter lp_filter(10); // Go to branch average moving filter 
PID motor_right_pid(80, 0.1, 0, 0, 0, h/1000.0, 0, 0 , 1000); // K Ti Td N b h Tt
double left_vel = 0.0;

mpu6050 mpu(0x68);



double REF = 0;

bool automatic_ref = 1;
int duty_left = 300;
int duty_right = 1000;


bool newData = false;


int counter = 0;

#endif


