#include "pid_control.h"

PID_control::PID_control(float p_term, float d_term, float i_term, float goal) 
{
  k_p = p_term;
  k_d = d_term;
  k_i = i_term;
  set_config = goal;
  reset();
}

PID_control::PID_control(float p_term, float d_term, float i_term)
{
  k_p = p_term;
  k_d = d_term;
  k_i = i_term;
  set_config = 0;
  reset();
}

void PID_control::reset()
{
  P_err = 0;
  I_err = 0;
  D_err = 0;
  prev_error = 0;
}

void PID_control::change_goal(float goal)
{
  set_config = goal;
  reset();
}

void PID_control::change_p(float p)
{
  k_p = p;
  reset();
}

void PID_control::change_i(float i)
{
  k_i = i;
  reset();
}

void PID_control::change_d(float d)
{
  k_d = d;
  reset();
}

float PID_control::pid(float input, double time)
{
  float error = set_config - input;
  double dt = time - prev_time;

  P_err = error;
  I_err += prev_error*dt;
  D_err = (error - prev_error)/dt;

  prev_time = time;
  prev_error = error;
  return k_p*P_err + k_i*I_err + k_d*D_err;
}
