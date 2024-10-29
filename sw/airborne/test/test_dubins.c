


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "std.h"

// #include "modules/guidance/dubins.h"

void circle_origin2(float x1, float y1, float x2, float y2, float x3, float y3, float *x0, float *y0, float *xtp1, float *ytp1, float *xtp3, float *ytp3, float turn_radius);
float signed_angle_between_two_lines(float x1, float y1, float x2, float y2, float x3, float y3);

int main(int argc, char **argv)
{

  float x1 = 1.f;
  float y1 = 1.f;
  float x2 = 2.f;
  float y2 = 2.f;
  float x3 = 3.f;
  float y3 = 1.f;

  printf("angle_between_two_lines: %f\n", 180.f/M_PI*signed_angle_between_two_lines(x1, y1, x2, y2, x3, y3));

  float x0, y0, xtp1, ytp1, xtp2, ytp2;
  float turn_radius = 0.2f;

  circle_origin2(x1, y1, x2, y2, x3, y3, &x0, &y0, &xtp1, &ytp1, &xtp2, &ytp2, turn_radius);

  printf("circle_origin: %f %f %f %f %f %f\n", x0, y0, xtp1, ytp1, xtp2, ytp2);

}


float signed_angle_between_two_lines(float x1, float y1, float x2, float y2, float x3, float y3) {
  float angle1 = atan2(y2-y1, x2-x1);
  float angle2 = atan2(y3-y2, x3-x2);
  return angle2 - angle1;
}


/**
 * Function that calculates the circle origin for a circle that is tangent to two lines
 */
void circle_origin2(float x1, float y1, float x2, float y2, float x3, float y3, float *x0, float *y0, float *xtp1, float *ytp1, float *xtp3, float *ytp3, float turn_radius) {
  float angle1 = atan2(y2-y1, x2-x1);
  float angle2 = atan2(y3-y2, x3-x2);

  printf("angle1: %f\n", angle1);
  printf("angle2: %f\n", angle2);

  float angle = (angle2 - angle1)/2.f;

  printf("angle: %f\n", angle);

  float dist = sinf(angle) * turn_radius;
  // calculate the circle origin
  *x0 = x2 + dist * cosf(angle + angle1);
  *y0 = y2 + dist * sinf(angle + angle1);

  // calculate distance d along line from x2, y2 to tangent point
  float d = sqrtf(powf(turn_radius, 2) + powf(dist, 2));

  //calculate tangent points
  float v1x = x1 - x2;
  float v1y = y1 - y2;
  float l1 = sqrtf(powf(v1x, 2) + powf(v1y, 2));
  *xtp1 = v1x/l1*d + x2;
  *ytp1 = v1y/l1*d + y2;

  float v3x = x3 - x2;
  float v3y = y3 - y2;
  float l3 = l1;
  *xtp3 = v3x/l1*d + x2;
  *ytp3 = v3y/l1*d + y2;
}