#include "constants.h"




bool sampling( struct repeating_timer *t  ){
  if(!timer_fired) timer_fired = true;
  return true;
}


void encoder_callback_core0(uint gpio, uint32_t events){

  next = micros();
  bool direction =  digitalRead(motor_right.encoder_b);
  //rad per sec
  //Serial.println(1000000*motor_steps/(next-prev)); 

  //rpm
  //Serial.println(1000000*60/((next-prev)*96)); 
  motor_right.vel = 1000000*60/((next-motor_right.prev)*96/2);
  //motor_right.diff = next-motor_right.prev;
  motor_right.prev = next;
  if (!direction){
    motor_right.vel =  -motor_right.vel;
  }
  
  return;
}

void encoder_callback_core1(uint gpio, uint32_t events){
  next = micros();
  bool direction  =  digitalRead(motor_left.encoder_b);
  
  if(next-motor_left.prev < 800) return;
  uint32_t isr_state = spin_lock_blocking( slk );
  motor_left.vel = 1000000*60/((next-motor_left.prev)*96/2);
  motor_left.prev = next;
  if (!direction){
    motor_left.vel =  -motor_left.vel;
  }
  spin_unlock(slk, isr_state);
  return;
}



void setup() {
  //stdio_init_all();  
  Serial.begin(19200);
  
  
  
  analogReadResolution(12); // Default resolution is 10, change it to match ADC characteristics
  analogWriteResolution(12);
  //analogWriteFreq(100000);
  
  
  // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
  // If the delay is < 0 then the next call to the callback will be exactly   500ms after the
  add_repeating_timer_ms( -h, sampling, NULL,&timer); //100 Hz
  gpio_set_irq_enabled_with_callback(motor_right.encoder_a, GPIO_IRQ_EDGE_RISE , true, &encoder_callback_core0);
  
  
  bool initialized = false;
  
  mpu.calibration(6,false);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  int slk_number = spin_lock_claim_unused(true);
  slk = spin_lock_init(slk_number);
  
  delay(100);
  
  
}

void setup1(){
  gpio_set_irq_enabled_with_callback(motor_left.encoder_a, GPIO_IRQ_EDGE_RISE , true, &encoder_callback_core1);
}

void loop() { 
  
  


  if((millis() - prev) > 1000){
    digitalWrite(motor_right.IN_1, (int) bit);
    digitalWrite(motor_right.IN_2, (int) !bit);
    digitalWrite(motor_left.IN_1, (int) bit);
    digitalWrite(motor_left.IN_2, (int) !bit);
    digitalWrite(LED_BUILTIN, bit);
    bit = !bit;
    prev = millis();
  }

  if( timer_fired ){
    timer_fired = false;

    uint32_t irq_state = spin_lock_blocking(slk);  // Lock
    float left_velocity = motor_left.vel;  // Copy to local variable
    spin_unlock(slk, irq_state);  // Unlock

    mpu.read_values();
  
    roll.predict(mpu.gyro[0]);


    roll.update(mpu.angle[0]);


    Serial.print("Raw Roll:");
    Serial.print(mpu.gyro[0]);
    Serial.print(" ");
    Serial.print(mpu.angle[0]);
    Serial.print(" ");
    Serial.println(roll.x);

    Serial.print("Limit_1:");Serial.print(800);Serial.print("\t");
    Serial.print("Limit_2:");Serial.print(-800);Serial.print("\t");
    Serial.print("Right_motor:");
    Serial.print(motor_right.vel);
    Serial.print("\t");
    Serial.print("Left_motor:");
    Serial.println(left_velocity);
    Serial.println();


  }
}


