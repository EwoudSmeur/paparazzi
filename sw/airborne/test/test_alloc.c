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

  float u_min[4] = {-107, -19093, 0, -95, };
  float u_max[4] = {19093, 107, 4600, 4505, };

  float g1g2[4][4] =
  {{      0,         0,  -0.0105,  0.0107016},
  {-0.0030044, 0.0030044, 0.035, 0.035},
  {-0.004856, -0.004856, 0, 0},
  {       0,         0,   -0.0011,   -0.0011}};

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 1000, 0.1, 10};
  /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

  // The control objective in array format
  float indi_v[4] = {10.8487,  -10.5658,    6.8383,    1.8532};
  float indi_du[4];

  // Initialize the array of pointers to the rows of g1g2
  float* Bwls[4];
  uint8_t i;
  for(i=0; i<INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du,indi_v,u_min,u_max,Bwls,INDI_NUM_ACT,INDI_OUTPUTS,0,0,Wv,0,0,10000,20);

  printf("du = %f, %f, %f, %f\n", indi_du[0],indi_du[1],indi_du[2],indi_du[3]);

  int8_t j;
  float out[4] = {0.0};
  for(i=0; i<INDI_OUTPUTS; i++) {
    for(j=0; j<INDI_NUM_ACT; j++) {
      out[i] = out[i] + g1g2[i][j] * indi_du[j];
    }
  }

  printf("dv_req = %f, %f, %f, %f\n", indi_v[0],indi_v[1],indi_v[2],indi_v[3]);
  printf("dv = %f, %f, %f, %f\n", out[0],out[1],out[2],out[3]);

  printf("number of iterations = %d\n", num_iter);
}