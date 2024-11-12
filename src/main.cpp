#include "constants.h"




bool sampling( struct repeating_timer *t  ){
  if(!timer_fired) timer_fired = true;

  return true;
}


void encoder_callback(uint gpio, uint32_t events){
  next = micros();
  //rad per sec
  //Serial.println(1000000*motor_steps/(next-prev)); 

  //rpm
  //Serial.println(1000000*60/((next-prev)*96)); 

  if (gpio == motor_right.encoder){
    motor_right.vel = 1000000*60/((next-motor_right.prev)*96/2);
    motor_right.prev = next;
  }else{
    motor_left.vel = 1000000*60/((next-motor_left.prev)*96/2);
    motor_left.prev = next;
  }

  if (events == GPIO_IRQ_EDGE_RISE || gpio == motor_right.encoder){
    motor_right.vel = -motor_right.vel;
    return;
  }

  if (events == GPIO_IRQ_EDGE_RISE){
    motor_left.vel  = -motor_left.vel;
  }

  
  //gpio_irq_callback_
}



void setup() {
  //stdio_init_all();  
  Serial.begin(19200);

  pinMode(motor_right.IN, OUTPUT);
  pinMode(motor_left.IN, OUTPUT);


  pinMode(motor_right.encoder, INPUT);
  pinMode(motor_left.encoder, INPUT);


  analogReadResolution(12); // Default resolution is 10, change it to match ADC characteristics
  analogWriteResolution(12);
  //analogWriteFreq(100000);


  // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
  // If the delay is < 0 then the next call to the callback will be exactly   500ms after the
  add_repeating_timer_ms( -1, sampling, NULL,&timer); //100 Hz
  gpio_set_irq_enabled_with_callback(motor_right.encoder, GPIO_IRQ_EDGE_RISE , true, &encoder_callback);

  bool initialized = false;

  mpu.calibration(6,false);

  delay(100);

}



void loop() {

  

  if( timer_fired ){
    timer_fired = false;
    digitalWrite(motor_a, (int) bit);
    digitalWrite(motor_b, (int) !bit);
    bit = !bit;

    mpu.read_values();


    Serial.print("Acceleration_X:");
    Serial.print(mpu.accel[0]);
    Serial.print(" Acceleration_Y:");
    Serial.print(mpu.accel[1]);
    Serial.print(" Acceleration_Z:");
    Serial.println(mpu.accel[2]);
    
    Serial.print("Gyro_X:");
    Serial.print(mpu.gyro[0]);
    Serial.print(" Gyro_Y:");
    Serial.print(mpu.gyro[1]);
    Serial.print(" Gyro_Z:");
    Serial.println(mpu.gyro[2]);

  }
}


