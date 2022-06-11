#include "pid_control.h"
#include <stdio.h>

int main()
{
  PID_control* meow = new PID_control(.07, .01, .7, 5);
  float result = 1;
  for(int counter = 0; counter < 100; counter++)
  {
    result = meow->pid(result, counter+1);
    printf("%f\t", result);
  }
  return 0;
}
