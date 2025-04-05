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
motor motor_left(2,3,6,7,21);

mpu6050 mpu(0x68);

kalman roll((float) h/1000),pitch((float) h/1000);

double REF = 0.0;
int automatic_ref = 1;
double K = 10.0;
double Ti = 1.0;
double ad = 0.0;
int duty = 0;
PID pid(K, Ti, 0, 0, 0, (float) h/1000, 0); // K Ti Td N b h Tt


boolean newData = false;

const byte numChars = 20;
char receivedChars[numChars];   // an array to store the received data


spin_lock_t *slk {0};
#endif


