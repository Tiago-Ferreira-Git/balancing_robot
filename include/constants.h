#ifndef _constants_H_
#define _constants_H_



#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "motor.h"


#define PS_PIN 23 // Power Save Pin, H to disable, L default


// IN1 gpio 20
// IN2 gpio 21
#define motor_a 20
#define motor_b 21

#define encoder_a 16
#define encoder_b 18

const float motor_steps = 0.065;


volatile bool timer_fired {false};



//the interrupt service routine
struct repeating_timer timer;

bool bit = 0;

volatile int check_rotation = 0;

volatile long int next = 0;

Adafruit_MPU6050 mpu;
Adafruit_Sensor  *mpu_accel, *mpu_gyro;

sensors_event_t accel;
sensors_event_t gyro;

motor motor_right(motor_a,encoder_a);
motor motor_left(motor_b,encoder_b);

#endif