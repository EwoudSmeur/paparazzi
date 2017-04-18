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
#define INDI_NUM_ACT 5

int main(int argc, char **argv)
{

  float u_min[INDI_NUM_ACT] = {-6759.22534179688,         -11841.2458496094,          -1807.7490234375,          -1955.8349609375,          -6232.2822265625};
  float u_max[INDI_NUM_ACT] = {12440.7746582031,          7358.75415039062,           2792.2509765625,           2644.1650390625,          12967.7177734375};

  float g1g2[4][INDI_NUM_ACT] =
     {{      0,            0,      -0.0133,       0.0133,            0},
      {-0.0021,       0.0021,            0,            0,           0.0},
      { -0.002,       -0.002,            0,            0,            0},
      {      0,            0,      -0.0011,      -0.0011,            0}};

  //State prioritization {W Roll, W pitch, W yaw, TOTAL THRUST}
  static float Wv[INDI_OUTPUTS] = {100, 1000, 0.1, 10};
  /*static float Wv[INDI_OUTPUTS] = {10, 10, 0.1, 1};*/

  // The control objective in array format
  float indi_v[4] = {6.32495880126953,          38.7138252258301,         -2.64358568191528,        -0.218257620930672};
  float indi_du[INDI_NUM_ACT];

  // Initialize the array of pointers to the rows of g1g2
  float* Bwls[4];
  uint8_t i;
  for(i=0; i<INDI_OUTPUTS; i++) {
    Bwls[i] = g1g2[i];
  }

  // WLS Control Allocator
  int num_iter =
    wls_alloc(indi_du,indi_v,u_min,u_max,Bwls,INDI_NUM_ACT,INDI_OUTPUTS,0,0,Wv,0,0,10000,20);

  printf("du = %f, %f, %f, %f, %f\n", indi_du[0],indi_du[1],indi_du[2],indi_du[3],indi_du[4]);

  int8_t j;
  float out[INDI_NUM_ACT] = {0.0};
  for(i=0; i<INDI_OUTPUTS; i++) {
    for(j=0; j<INDI_NUM_ACT; j++) {
      out[i] = out[i] + g1g2[i][j] * indi_du[j];
    }
  }

  printf("dv_req = %f, %f, %f, %f\n", indi_v[0],indi_v[1],indi_v[2],indi_v[3]);
  printf("dv = %f, %f, %f, %f\n", out[0],out[1],out[2],out[3]);
  printf("number of iterations = %d\n", num_iter);
}
