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
  motor_right.vel = multiplier*1000000*motor_steps/((next-motor_right.prev));
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

  motor_left.vel = multiplier*1000000*motor_steps/((next-motor_left.prev));

  spin_unlock(slk, isr_state);
  motor_left.prev = next;

  return;
}



void setup() {
  //stdio_init_all();  
  Serial.begin(57600);
  
  
  
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



  delay(100);
  
}

void setup1(){
  gpio_set_irq_enabled_with_callback(motor_left.encoder_a, GPIO_IRQ_EDGE_RISE , true, &encoder_callback_core1);
}



void print_serial(){

  Serial.print("Time:");Serial.print(time_stamp);Serial.print(" ");
  //Serial.print("REF:");Serial.print(REF);Serial.print("\t");
  Serial.print("Duty_left:");Serial.print(12.0*duty_left/1000);Serial.print(" ");
  Serial.print("Duty_right:");Serial.print(12.0*duty_right/1000);Serial.print(" ");
  Serial.print("Right_motor_velocity:");Serial.print(motor_right.vel);Serial.print(" ");
  Serial.print("Left_motor_velocity:");Serial.print(motor_left.vel);Serial.print(" ");
  //Serial.print("Roll_raw:");Serial.print(mpu.angle[0]);Serial.print(" ");
  //Serial.print("Estimated_i_r:");Serial.print(estimator.x_pred(0,0));Serial.print(" ");
  //Serial.print("Estimated_w_r:");Serial.print(estimator.x_pred(1,0));Serial.print(" ");
  Serial.print("Estimated_i_l:");Serial.print(estimator.x_pred(2,0));Serial.print(" ");
  Serial.print("Estimated_w_l:");Serial.print(estimator.x_pred(3,0));Serial.print(" ");
  //Serial.print("Estimated_x:");Serial.print(estimator.x_pred(4,0));Serial.print(" ");
  //Serial.print("Estimated_x_dot:");Serial.print(estimator.x_pred(5,0));Serial.print(" ");
  //Serial.print("Estimated_theta:");Serial.print(estimator.x_pred(6,0));Serial.print(" ");
  //Serial.print("Estimated_theta_dot:");Serial.print(estimator.x_pred(7,0));
  Serial.println("\t");

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


    uint32_t irq_state = spin_lock_blocking( slk );
    pid_left.set_gains(parameters[0],parameters[1],parameters[2]);
    pid_right.set_gains(parameters[3],parameters[4],parameters[5]);
    spin_unlock(slk, irq_state);  // Unlock

    measurement(0,0) = motor_right.vel;
    measurement(1,0) = motor_left.vel;
    measurement(2,0) += h_outer*measurement(3,0);
    measurement(3,0) = WHEEL_RADIUS*motor_right.vel;
    measurement(4,0) = mpu.angle[0];
    measurement(5,0) = mpu.gyro[0];



    control_action(0,0) = 12.0*duty_right/1000;
    control_action(1,0) = 12.0*duty_left/1000;

    estimator.predict(control_action);
    estimator.update(measurement);
    
    
    //duty_left = pid_left.control(REF,motor_left.vel);
    //duty_right = pid_right.control(REF,motor_right.vel);
    
    motor_left.set_speed(duty_left);
    motor_right.set_speed(duty_right);

    if (std::abs(mpu.angle[0]) > 25){
      duty_left = 0;
      duty_right = 0;
    }
    
    count++;
    if (count == h_outer) {
      
      if (!automatic_ref ){
        //duty_right = duty_left;
      }  
      //else if (!automatic_ref)   REF = 0;
      
      if (!automatic_ref)  REF = -REF;
      
      
      print_serial();
      //estimator.verbose();

      time_stamp = time_stamp + h_outer/1000.0  ;
      count = 0;
    }
    
    

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

