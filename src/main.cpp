#include "constants.h"




bool sampling( struct repeating_timer *t  ){
  if(!timer_fired) timer_fired = true;
  return true;
}


void encoder_callback_core0(uint gpio, uint32_t events){

  next = micros();
  bool direction =  digitalRead(motor_right.encoder_b);
  double multiplier = 1.0;
  //rad per sec
  //Serial.println(1000000*motor_steps/(next-prev)); 

  //rpm
  //Serial.println(1000000*60/((next-prev)*96)); 
  if (!direction){
    multiplier =  -1.0;
  }
  //0.8*motor_right.vel + (1-0.8)*  
  motor_right.vel = multiplier*1000000*60/((next-motor_right.prev)*96/2);
  motor_right.prev = next;
  
  return;
}

void encoder_callback_core1(uint gpio, uint32_t events){

  next = micros();
  bool direction =  digitalRead(motor_left.encoder_b);
  double multiplier = 1.0;
  //rad per sec
  //Serial.println(1000000*motor_steps/(next-prev)); 

  //rpm
  //Serial.println(1000000*60/((next-prev)*96)); 
  if (!direction){
    multiplier =  -1.0;
  }
  uint32_t isr_state = spin_lock_blocking( slk );
  //0.5095*motor_left.vel + 0.2452*(raw+

  motor_left.vel = multiplier*1000000*60/((next-motor_left.prev)*96/2);

  spin_unlock(slk, isr_state);
  motor_left.prev = next;
  
  return;
}



void setup() {
  //stdio_init_all();  
  Serial.begin(74880);
  
  
  
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
  
  digitalWrite(motor_right.IN_1, 1);
  digitalWrite(motor_right.IN_2, 0);
  digitalWrite(motor_left.IN_1, 0);
  digitalWrite(motor_left.IN_2, 1);
  // pwm_set_chan_level(pwm_gpio_to_slice_num(motor_right.pwm), pwm_gpio_to_channel(motor_right.pwm), duty);
  // pwm_set_chan_level(pwm_gpio_to_slice_num(motor_left.pwm), pwm_gpio_to_channel(motor_left.pwm), 0);
  delay(100);
  
}

void setup1(){
  gpio_set_irq_enabled_with_callback(motor_left.encoder_a, GPIO_IRQ_EDGE_RISE , true, &encoder_callback_core1);
}

void loop() { 
  


  if((millis() - prev) > 10000){
    //(int) bit (int) !bit
    
    digitalWrite(LED_BUILTIN, bit);
    prev = millis();
    bit = !bit;
    if (bit && automatic_ref){
      //bit = !bit;
      REF = 400.0;
    }else if(!bit && automatic_ref){
      REF = -400.0;
    }
  }

  if( timer_fired ){
    timer_fired = false;

    mpu.read_values();
  
    
    roll.predict(mpu.gyro[0]);
    roll.update(mpu.angle[0]);


    uint32_t irq_state = spin_lock_blocking( slk );
    pid_left.set_gains(parameters[0],parameters[1],parameters[2]);
    pid_right.set_gains(parameters[3],parameters[4],parameters[5]);
    spin_unlock(slk, irq_state);  // Unlock




    
    // left_velocity.predict(12.0*duty_left/1000);
    // left_velocity.update(motor_left.vel);
    
    
    duty_left = pid_left.control(REF,motor_left.vel);
    duty_right = pid_right.control(REF,motor_right.vel);

    motor_left.set_speed(duty_left);
    motor_right.set_speed(duty_right);
    
    
    
    Serial.print("Time:");Serial.print(time_stamp);Serial.print("\t");
    Serial.print("Duty:");Serial.print(12.0*duty_left/1000);Serial.print("\t");
    Serial.print("Right_motor:");
    Serial.print(motor_right.vel);
    Serial.print("\t");
    Serial.print("Left_motor:");
    Serial.print(motor_left.vel);
    Serial.print("\t");
    Serial.print("REF:");
    Serial.print(REF);
    Serial.print("\t");
    Serial.print("Angle:");
    Serial.print(roll.x);
    Serial.print("\t");
    Serial.print("Angle_raw:");
    Serial.print(mpu.angle[0]);
    Serial.print("\t");
    Serial.print("Angle_raw:");
    Serial.print(mpu.angle[1]);
    Serial.print("\t");
    time_stamp = time_stamp + h/1000.0  ;

  }
}


void loop1(){

  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  int n = 0;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
    else {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
    }
  }

  if (newData == true) {
    uint32_t irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"Auto:%d",&automatic_ref);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"K_left:%lf",&parameters[0]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"Ti_left:%lf",&parameters[1]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"ad_left:%lf",&parameters[2]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"K_right:%lf",&parameters[3]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"Ti_right:%lf",&parameters[4]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"ad_right:%lf",&parameters[5]);
    spin_unlock(slk, irq_state);  // Unlock
    irq_state = spin_lock_blocking( slk );
    n += sscanf(receivedChars,"REF:%lf",&REF);
    spin_unlock(slk, irq_state);  // Unlock
    if(n==0){
      Serial.println("Error getting parameters!!!!");
    }
    newData = false;
  }
}

