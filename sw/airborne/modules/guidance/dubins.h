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

#ifndef DUBINS_H
#define DUBINS_H

#include "std.h"


extern float signed_angle_between_two_lines(float x1, float y1, float x2, float y2, float x3, float y3);

extern void circle_origin(uint8_t wp1, uint8_t wp2, uint8_t wp3, float *x0, float *y0, float *xtp1, float *ytp1, float *xtp2, float *ytp2, float turn_radius);

#endif // DUBINS_H

