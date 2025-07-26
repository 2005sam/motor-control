#include "slide_mode_compute.h"
#include "math.h"
float K_t = 0.03f;
float b = 0.01f;
float k = 10;
float d = 0.1f;
float SlideModeCompute(float sp, float cur)
{
  float error = sp - cur;
  float u_eq = b / k_t * error;
  float u_sw = k * error / (fabs(error) + d);
  return u_eq + u_sw;
}