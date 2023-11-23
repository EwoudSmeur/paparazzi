/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file filters/second_order_filter.h
 *  @brief Simple first second order filter
 *
 */

#ifndef SECOND_ORDER_FILTER_H
#define SECOND_ORDER_FILTER_H

#include "std.h"
#include "math/pprz_algebra_int.h"

#define INT32_FILT_FRAC  8

/** Second order filter structure.
 *
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *

 *
 * Note that b[0]=b[2], so we don't need to save b[2]
 */
struct SecondOrderFilter {
  float a[2]; ///< denominator gains
  float b[3]; ///< numerator gains
  float i[2]; ///< input history
  float o[2]; ///< output history
};

/** Init second order  filter.
 *
 */
static inline void init_second_order_filter(struct SecondOrderFilter *filter, float* a, float* b, float value)
{
  filter->a[0] = a[0];
  filter->a[1] = a[1];
  filter->b[0] = b[0];
  filter->b[1] = b[1];
  filter->b[2] = b[2];
  filter->i[0] = filter->i[1] = filter->o[0] = filter->o[1] = value;
}

/** Update second order filter state with a new value.
 *
 * @param filter second order filter structure
 * @param value new input value of the filter
 * @return new filtered value
 */
static inline float update_second_order_filter(struct SecondOrderFilter *filter, float value)
{
  float out = filter->b[0] * value
              + filter->b[1] * filter->i[0]
              + filter->b[2] * filter->i[1]
              - filter->a[0] * filter->o[0]
              - filter->a[1] * filter->o[1];
  filter->i[1] = filter->i[0];
  filter->i[0] = value;
  filter->o[1] = filter->o[0];
  filter->o[0] = out;
  return out;
}

/** Get current value of the second order filter.
 *
 * @param filter second order filter structure
 * @return current value of the filter
 */
static inline float get_second_order_filter(struct SecondOrderFilter *filter)
{
  return filter->o[0];
}



#endif

