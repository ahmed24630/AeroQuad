#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

class PID_control
{
  public:
    PID_control(float p_term, float d_term, float i_term, float goal); 
    PID_control(float p_term, float d_term, float i_term); 

    void change_goal(float goal);
    void change_p(float p);
    void change_i(float i);
    void change_d(float d);

    float pid(float input, double time);

  private:
    void reset();
    
    float k_p;
    float k_i;
    float k_d;
    
    float set_config;
    float prev_error;
    double prev_time;

    float P_err;
    float I_err;
    float D_err;
};

#endif
