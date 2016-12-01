#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"
#include "firmwares/rotorcraft/stabilization/wls/wls_alloc.h"

#define INDI_OUTPUTS 4
#define INDI_NUM_ACT 4

int main(int argc, char **argv)
{

  float u_min[4] = {0, 0, 0, 0};
  float u_max[4] = {9600, 9600, 9600, 9600};

  float g1g2[4][4] =
  {{ 0.0200,   -0.0200,   -0.0200,    0.0200},
    {0.0140,    0.0140,   -0.0140,   -0.0140},
    {0.0610,   -0.0610,    0.0610,   -0.0610},
    {-0.0004,   -0.0004,   -0.0004,   -0.0004}};

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};

  // The control objective in array format
  float indi_v[4] = {-0.001818, -2.349125, -0.110446, -0.000000};
  float indi_du[4];

  // Initialize the array of pointers to the rows of g1g2
  float* Bwls[4];
  uint8_t i;
  for(i=0; i<INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du,indi_v,u_min,u_max,Bwls,INDI_NUM_ACT,INDI_OUTPUTS,0,0,Wv,0,0,10000,10);
  printf("du = %f, %f, %f, %f\n", indi_du[0],indi_du[1],indi_du[2],indi_du[3]);
}
