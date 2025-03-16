
#include "motor.h"

motor::motor(uint8_t IN_1_,uint8_t IN_2_,uint8_t encoder_a_,uint8_t encoder_b_){
  IN_1 = IN_1_;
  IN_2 = IN_2_;
  encoder_a = encoder_a_;
  encoder_b = encoder_b_;
  
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  pinMode(encoder_a, INPUT);
  pinMode(encoder_b, INPUT);
  
}

