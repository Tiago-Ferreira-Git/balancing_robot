#include "constants.h"

bool sampling_core0(struct repeating_timer *t)
{
  if (!time_to_sample_core0)
    time_to_sample_core0 = true;
  return true;
}


bool sampling_core1(struct repeating_timer *t)
{
  if (!time_to_sample_core1)
    time_to_sample_core1 = true;
  return true;
}

void encoder_callback_core0(uint gpio, uint32_t events)
{

  absolute_time_t next_time = get_absolute_time();
  bool direction = gpio_get(motor_right.encoder_b);
  double multiplier = 1.0;

  if (!direction)
  {
    multiplier = -1.0;
  }

  motor_right.vel = multiplier * 1000000 * motor_steps / (absolute_time_diff_us(next_time, motor_right.prev));
  motor_right.prev = next_time;

  return;
}


void encoder_callback_core1(uint gpio, uint32_t events)
{

  absolute_time_t next_time = get_absolute_time();
  bool direction = gpio_get(motor_left.encoder_b);
  double multiplier = 1.0;

  if (!direction)
  {
    multiplier = -1.0;
  }

  motor_left.vel = multiplier * 1000000 * motor_steps / (absolute_time_diff_us(next_time, motor_left.prev));
  motor_left.prev = next_time;

  return;
}

void core1_main(){
  add_repeating_timer_ms(-h, sampling_core1, NULL, &timer_core1); // 100 Hz
  gpio_set_irq_enabled_with_callback(motor_left.encoder_a, GPIO_IRQ_EDGE_RISE, true, &encoder_callback_core1);

   while (true)
  {

    if (time_to_sample_core1)
    {
      multicore_fifo_push_blocking(motor_left.vel);

    }

  }


}


int main()
{

  stdio_init_all();

  // If the delay is > 0 then this is the delay between the prev_timeious callback ending and the next_time starting.
  // If the delay is < 0 then the next_time call to the callback will be exactly   500ms after the
  add_repeating_timer_ms(-h, sampling_core0, NULL, &timer_core0); // 100 Hz
  gpio_set_irq_enabled_with_callback(motor_right.encoder_a, GPIO_IRQ_EDGE_RISE, true, &encoder_callback_core0);

  multicore_launch_core1(core1_main);
  

  bool initialized = false;

  mpu.calibration(6, false);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);


  sleep_ms(1000);
  

  while (true)
  {

    if (time_to_sample_core0)
    {

      gpio_put(PICO_DEFAULT_LED_PIN, bit);
      bit = !bit;

      Eigen::Matrix<double, 1, 1> vel(motor_right.vel);

      double left_vel = multicore_fifo_pop_blocking();


      meas(0,0) = motor_right.vel;
      meas(1,0) = left_vel;

      REF(0,0) = 500.0;
      REF(1,0) = 500.0;

      duty = controller.control(REF, meas);

      
      Eigen::Matrix<double, 1, 1> control(12.0 * duty(0,0) / 1000);

      //estimator.predict(control);

      motor_left.set_speed(duty(0,0));
      motor_right.set_speed(duty(1,0));
      time_to_sample_core0 = false;
      //estimator.update(vel);

      printf(" %.3f\t", time_stamp);
      printf("%.3f\t", meas(0,0));
      printf("%.3f\t", meas(1,0));
      printf("%.3f\t", duty(0,0));
      printf("%.3f\t", duty(1,0));
      
      printf("\n");
      time_stamp = time_stamp + h_outer / 1000.0;
      counter += 1;
    }
  }
  return 0;
}
