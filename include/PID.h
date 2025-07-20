#ifndef _PID_H_
#define _PID_H_

#include "Arduino.h"

class PID {
 private:
  float K = 0, Ti = 0, Td = 0;
  float I = 0, D = 0, P = 0;
  float b = 0;      // Set Point Weighting
  float h = 0;      // time-step between control actions
  float y_old = 0;  // keep last value read
  float N = 0;
  float ad = 0;
  float bd = 0;
  float Tt = 0;  // anti-windup gain
  float REF_old = 0;
  float limit = 0;
  bool anti_windup = false;

 public:
  /*
   * @brief Costructor for class PID: atributes the parameters when the class is
   * called to the private variables of the class
   */
  explicit PID(float K_, float Ti_, float Td_, float N_, float b_, float h_,
               float Tt_,float limit_);
  ~PID() {}
  /*
   * @brief Compute the proporcional term of the controller
   *
   * @param REF value inputed by the user (in LUX)
   * @param y the value measured (in LUX)
   *
   * @returns The value of proportional term with set point weighting (uses
   * parameters b and K)
   */
  float proporcional(float);
  /*
   * @brief Compute the integrator term of the controller
   *
   * @param error the difference between reference and the value measured
   * @param saturation_error difference between clipped control signal and the
   * "real" control signal (if anti_windup is true)
   *
   * @returns The value of integrator term with set point weighting (uses
   * parameters anti_windup, K , Ti, Tt )
   */
  float integrator(float, float);
  /*
   * @brief Compute the derivative term of the controller
   *
   * @param y value measured (in LUX)
   *
   *
   * @returns The value of derivative term with set point weighting (uses
   * parameters ad, bd , D)
   */
  float derivative(float);
  /*
   * @brief Compute the control signal: sums all the previous terms and clips
   * the control signal (maximum is 4095 and minimum is 0)
   *
   * @param REF reference value (in LUX)
   * @param y value measured (in LUX)
   *
   *
   * @returns The computed control signal
   */
  float control(float, float);
  /*
   * @brief Sets the anti_windup flag as the input of the function
   * @param to_set boolean that sets (if 1) or resets (if 0) the flag
   * anti_windup
   */
  void set_anti_windup(bool);
  /*
   * @brief As requested in the guidelines: function to print the anti_windup
   * flag as it is a private variable
   */
  int get_anti_windup_state();
  /*
   * @brief For debug purposes: prints the values of the PID (P,I,D in this
   * order), useful for plots presented in report
   */
  void print_pid_values();
    /*
   * @brief For debug purposes: prints the values of the PID (P,I,D in this
   * order), useful for plots presented in report
   */
  void print_gains();
      /*
   * @brief For debug purposes: prints the values of the PID (P,I,D in this
   * order), useful for plots presented in report
   */
  void set_gains(double , double , double );
};

#endif