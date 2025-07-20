#ifndef _constants_H_
#define _constants_H_



#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "Arduino.h"
#include "motor.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "PID.h"
#include <iostream>
#include <Eigen/Dense>
#include "matrix_def.h"


using namespace Eigen;

#define PS_PIN 23 // Power Save Pin, H to disable, L default
#define WHEEL_RADIUS 0.025


const float motor_steps = 0.065;

volatile bool timer_fired {false};


//the interrupt service routine
struct repeating_timer timer;

bool bit = 0;

volatile int check_rotation = 0;

volatile long int next = 0;
volatile long int prev = 0;


static int h = 1; //sampling time in ms
static int h_outer = 10*h;
double time_stamp = 0;

// IN1 gpio 20
// IN2 gpio 21
#define motor_a 20
#define motor_b 21

int encoder_a  = 16;
int encoder_b  = 17;


motor motor_right(1,0,encoder_a,encoder_b,20);
motor motor_left(2,3,7,6,21);

mpu6050 mpu(0x68);



double REF = 0.0;

int automatic_ref = 0;
int duty_left = 300;
int duty_right = 0;


PID pid_left(0.7, 0.08, 0, 0, 0, (float) h/1000, 0,1000); // K Ti Td N b h Tt
PID pid_right(2, 0.15, 0, 0, 0, (float) h/1000, 0,1000); // K Ti Td N b h Tt

PID tilt(25, 100000, 0, 0, 0, (float) h_outer/1000, 0,1000); // K Ti Td N b h Tt


boolean newData = false;

const byte numChars = 20;
char receivedChars[numChars];   // an array to store the received data


spin_lock_t *slk {0};



// K_left Ti_left ad_left K_right Ti_right ad_right
double parameters[6] = {1,0.1,0 , 2,0.15,0};

int count = 0;

double ref_angle = 0;

kalman estimator(A, B, C , Q_e , R_e);

Eigen::MatrixXd measurement = Eigen::MatrixXd::Zero(6, 1);
Eigen::MatrixXd control_action = Eigen::MatrixXd::Zero(2, 1);


#endif


