
#include "motor.h"

motor::motor(uint8_t IN_1_,uint8_t IN_2_,uint8_t encoder_a_,uint8_t encoder_b_,uint8_t pwm_){
  IN_1 = IN_1_;
  IN_2 = IN_2_;
  encoder_a = encoder_a_;
  encoder_b = encoder_b_;
  pwm = pwm_;
  
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  pinMode(encoder_a, INPUT);
  pinMode(encoder_b, INPUT);

  pinMode(pwm, OUTPUT);


  gpio_set_function(pwm, GPIO_FUNC_PWM);

  uint slice_num = pwm_gpio_to_slice_num(pwm);
  pwm_set_wrap(slice_num, 1000);  // Max 16-bit value (higher value = smoother dimming)
  pwm_set_enabled(slice_num, true);
  
}

void motor::set_direction(bool Forward){
  if(Forward){
    digitalWrite(IN_1, 1);
    digitalWrite(IN_2, 0);
  }else{
    digitalWrite(IN_1, 0);
    digitalWrite(IN_2, 1);
  }

};

void motor::set_speed(float duty){
  if(duty > 0){
    set_direction(1);
  }else{
    set_direction(0);
  }
  pwm_set_chan_level(pwm_gpio_to_slice_num(pwm), pwm_gpio_to_channel(pwm), std::abs(duty));

};

