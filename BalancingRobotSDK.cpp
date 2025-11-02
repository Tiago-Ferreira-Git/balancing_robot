#include "constants.h"




bool sampling_core0( struct repeating_timer *t  ){
  if(!time_to_sample_core0) time_to_sample_core0 = true;
  return true;
}



void encoder_callback_core0(uint gpio, uint32_t events){

  absolute_time_t next_time = get_absolute_time();
  bool direction =  gpio_get(motor_right.encoder_b);
  double multiplier = 1.0;

  if (!direction){
    multiplier =  -1.0;
  }

  
  motor_right.vel = multiplier*1000000*motor_steps/(absolute_time_diff_us(next_time,motor_right.prev));
  motor_right.prev = next_time;

  
  return;
}






int main(){ 
  
  stdio_init_all();

  // If the delay is > 0 then this is the delay between the prev_timeious callback ending and the next_time starting.
  // If the delay is < 0 then the next_time call to the callback will be exactly   500ms after the
  add_repeating_timer_ms( -h, sampling_core0, NULL,&timer_core0); //100 Hz
  gpio_set_irq_enabled_with_callback(motor_right.encoder_a, GPIO_IRQ_EDGE_RISE , true, &encoder_callback_core0);
  
  
  bool initialized = false;
  
  mpu.calibration(6,false);
  
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN , GPIO_OUT);
  
  
  gpio_put(motor_right.IN_1, 1);
  gpio_put(motor_right.IN_2, 0);
  motor_right.set_speed(REF); 
  

  sleep_ms(1000);

  while(true){

      if( time_to_sample_core0 ){
        
        gpio_put(PICO_DEFAULT_LED_PIN, bit);
        prev_time = get_absolute_time();
        bit = !bit;
        
        if (counter == 5000){
          REF = 0;
        }else if(counter == 10000){
          REF += 80;
        }else if(counter == 15000){
          REF -= 30;
        }else if(counter == 20000){
          REF -= 40;
        }else if(counter == 25000){
          REF += 50;
        }else if(counter == 30000){
          REF += 10;
        }else if(counter == 35000){
          REF -= 70;
        }else if(counter == 40000){
          REF += 45;
        }else if(counter == 50000){
          REF += 35;
        }else if(counter == 60000){
          REF -= 80;
        }else if(counter == 70000){
          REF += 20;
        }
        Eigen::Matrix<double, 1, 1> vel(motor_right.vel);

        duty_right = 10*REF;
        //pid_right.control(REF,motor_right.vel);
        Eigen::Matrix<double, 1, 1> control(12.0*duty_right/1000);

        filter_3.predict(motor_right.vel);
        filter_5.predict(motor_right.vel);
        filter_10.predict(motor_right.vel);
        filter_15.predict(motor_right.vel);
        filter_20.predict(motor_right.vel);

        estimator.predict(control);
        
        motor_right.set_speed(duty_right); 
        time_to_sample_core0 = false;
        estimator.update(vel);
       


        printf(" %.3f\t",time_stamp);
        printf("%.3f\t",REF);
        printf("%d\t",duty_right);
        printf("%.3f\t",motor_right.vel);
        printf("%.3f\t",filter_3.current_avg);
        printf("%.3f\t",filter_5.current_avg);
        printf("%.3f\t",filter_10.current_avg);
        printf("%.3f\t",filter_15.current_avg);
        printf("%.3f\t",filter_20.current_avg);
        printf("%.3f\t",(estimator.C*estimator.x_pred).value());
        printf("\n");
        time_stamp = time_stamp + h_outer/1000.0 ;
        counter += 1; 
      }
  }
  return 0;
}

