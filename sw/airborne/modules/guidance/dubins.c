/*
 * Copyright (C) 2024 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "modules/guidance/dubins.h"
#include <stdio.h>
#include "math/pprz_geodetic_float.h"


float turn_radius;

float signed_angle_between_two_lines(float x1, float y1, float x2, float y2, float x3, float y3) {
  float angle1 = atan2(y2-y1, x2-x1);
  float angle2 = atan2(y3-y2, x3-x2);
  return angle2 - angle1;
}

void dubins_segment(void) {

  //

  return false;
}

/**
 * Function that calculates the circle origin for a circle that is tangent to two lines
 */
void circle_origin(uint8_t wp1, uint8_t wp2, uint8_t wp3, float *x0, float *y0, float *xtp1, float *ytp1, float *xtp3, float *ytp3, float turn_radius) {

  struct EnuCoor_f *p1 = waypoint_get_enu_f(wp1);
  struct EnuCoor_f *p2 = waypoint_get_enu_f(wp2);
  struct EnuCoor_f *p3 = waypoint_get_enu_f(wp3);

  // calculate angles of the two lines
  float angle1 = atan2(p2->y - p1->y, p2->x - p1->x);
  float angle2 = atan2(p3->y - p2->y, p3->x - p2->x);

  // Signed angle between two lines
  float angle = (angle2 - angle1)/2.f;

  float dist = sinf(angle) * turn_radius;
  // calculate the circle origin
  *x0 = p2->x + dist * cosf(angle + angle1);
  *y0 = p2->y + dist * sinf(angle + angle1);

  // calculate distance d along line from x2, y2 to tangent point
  float d = sqrtf(powf(turn_radius, 2) + powf(dist, 2));

  //calculate tangent points
  float v1x = p1->x - p2->x;
  float v1y = p1->y - p2->y;
  float l1 = sqrtf(powf(v1x, 2) + powf(v1y, 2));
  *xtp1 = v1x/l1*d + p2->x;
  *ytp1 = v1y/l1*d + p2->y;

  float v3x = p3->x - p2->x;
  float v3y = p3->y - p2->y;
  float l3 = l1;
  *xtp3 = v3x/l1*d + p2->x;
  *ytp3 = v3y/l1*d + p2->y;
}

nav_circle(struct EnuCoor_f *wp_center, float radius)