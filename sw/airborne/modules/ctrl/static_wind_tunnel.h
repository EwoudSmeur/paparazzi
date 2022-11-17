/*
 * Copyright (C) 2020 Freek van Tienen
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/static_wind_tunnel.h
 * @brief Wind tunnel actuator settings
 *
 * Sets the actuators to a predefined set of values 
 */

#ifndef STATIC_WIND_TUNNEL_H_
#define STATIC_WIND_TUNNEL_H_

#include <std.h>

struct WT_data {
  float measurement_time;
  int32_t commands[4];
  bool run;
  int32_t counter;
};

extern struct WT_data wt_data;

extern void wt_run(void);


#endif /* STATIC_WIND_TUNNEL_H_ */
