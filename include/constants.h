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

#define PS_PIN 23 // Power Save Pin, H to disable, L default

const float motor_steps = 0.065;


volatile bool timer_fired {false};


//the interrupt service routine
struct repeating_timer timer;

bool bit = 0;

volatile int check_rotation = 0;

volatile long int next = 0;
volatile long int prev = 0;


static int h = 10; //sampling time in ms
double time_stamp = 0;

// IN1 gpio 20
// IN2 gpio 21
#define motor_a 20
#define motor_b 21

int encoder_a  = 16;
int encoder_b  = 17;


motor motor_right(1,0,encoder_a,encoder_b,20);
motor motor_left(3,2,6,7,21);

mpu6050 mpu(0x68);

kalman roll(1,(float) h/1000,2,1);

kalman left_velocity( 0.927699109413340,7.491278366739943,4,1);

double REF = 0.0;
int automatic_ref = 1;
int duty_left = 0;
int duty_right = 0;


PID pid_left(1, 0.1, 0, 0, 0, (float) h/1000, 0); // K Ti Td N b h Tt
PID pid_right(2, 0.15, 0, 0, 0, (float) h/1000, 0); // K Ti Td N b h Tt


boolean newData = false;

const byte numChars = 20;
char receivedChars[numChars];   // an array to store the received data


spin_lock_t *slk {0};



// K_left Ti_left ad_left K_right Ti_right ad_right
double parameters[6] = {1,0.1,0 , 2,0.15,0};

#endif


