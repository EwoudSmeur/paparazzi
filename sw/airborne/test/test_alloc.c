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

  float u_min[4] = {-2500, 0, -4500, -4500};
  float u_max[4] = {2*9600-2500, 2*9600, 4500, 4500};

  float g1g2[4][4] =
  {{      0,         0,   -0.0089,    0.0083},
  { -0.0045,    0.0035,         0,         0},
  { -0.0012,   -0.0013,    0.0007,   -0.0005},
  {       0,         0,   -0.0008,   -0.0008}};

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {1000, 1000, 1, 100};
  /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

  // The control objective in array format
  float indi_v[4] = {10, 100, -125, 0.0};
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
