#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"

#include "filters/high_pass_filter.h"

int main(int argc, char **argv)
{
  struct FourthOrderHighPass flap_accel_hp;
  double coef_b1[4] = {0.995201365263607,         -3.98080546105443,          5.97120819158164,         -3.98080546105443}; //0.3 Hz
  double coef_a1[4] = {-3.99037963870238,          5.97118516477772,         -3.97123128331507,         0.990425757422548};
  double coef_b2[4] = {0.992015065636079,         -3.96806026254432,          5.95209039381647,         -3.96806026254432}; //0.5 Hz
  double coef_a2[4] = {-3.9839660723158,          5.95202663534277,         -3.95215445206974,         0.984093890448954};
  float data[200] = {1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
  1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1};

  /*double data[200] = {1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,*/
  /*0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0};*/

  // init high pass filter
  init_fourth_order_high_pass(&flap_accel_hp, coef_a1, coef_b1, 0);
  /*flap_accel_hp.o[0] = 4;*/
  /*flap_accel_hp.o[0] = 3;*/
  /*flap_accel_hp.o[0] = 2;*/
  /*flap_accel_hp.o[0] = 1;*/
    printf("i0 = %f, i1 = %f, i2 = %f, i3 = %f, o0 = %f, o1 = %f o2 = %f, o3 = %f\n", flap_accel_hp.i[0], flap_accel_hp.i[1], flap_accel_hp.i[2], flap_accel_hp.i[3], flap_accel_hp.o[0], flap_accel_hp.o[1], flap_accel_hp.o[2], flap_accel_hp.o[3]);

  int32_t i;
  for(i=0; i<200; i++) {
    update_fourth_order_high_pass(&flap_accel_hp, data[i]);
    /*printf("out = %f\n", flap_accel_hp.o[0]);*/
    printf("i0 = %f, i1 = %f, i2 = %f, i3 = %f, o0 = %f, o1 = %f o2 = %f, o3 = %f\n", flap_accel_hp.i[0], flap_accel_hp.i[1], flap_accel_hp.i[2], flap_accel_hp.i[3], flap_accel_hp.o[0], flap_accel_hp.o[1], flap_accel_hp.o[2], flap_accel_hp.o[3]);
  }
    printf("coefb2 = %f, %f, %f, %f\n", flap_accel_hp.b[0], flap_accel_hp.b[1], flap_accel_hp.b[2], flap_accel_hp.b[3]);
    printf("coefa2 = %f, %f, %f, %f\n", flap_accel_hp.a[0], flap_accel_hp.a[1], flap_accel_hp.a[2], flap_accel_hp.a[3]);
}
