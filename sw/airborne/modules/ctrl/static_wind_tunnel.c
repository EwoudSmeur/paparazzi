/*
 * Copyright (C) 2020 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/ctrl/static_wind_tunnel.c
 * @brief Windtunnel automatic step controller
 *
 */

#include "modules/ctrl/static_wind_tunnel.h"


#define STATIC_WIND_TUNNEL_NUM_CONFIGS 2

void wt_init(void);

struct WT_data wt_data = {
  .measurement_time = 10.0,
  .commands = {0},
  .run = false,
  .counter = 0,
};

// Define actuator inputs for each condition in PPRZ units
int32_t configurations[STATIC_WIND_TUNNEL_NUM_CONFIGS][4] = {
  {0, 0, 0, 0},
  {100, 100, 0, 0},
};

// Nothing to do here
void wt_init(void)
{
}

/**
 * Periodic function
 * 
 * wt_data.commands are used in the airframe file to set the actuators
 */
void wt_run(void)
{
  if (wt_data.run) {

    wt_data.counter = wt_data.counter + 1;

    int32_t stage = floor((wt_data.counter / STATIC_WIND_TUNNEL_FREQUENCY) / wt_data.measurement_time);

    if (stage > STATIC_WIND_TUNNEL_NUM_CONFIGS) {
      wt_data.counter = 0;
      wt_data.run = false;
      return;
    }

    for (int i=0; i<4; i++) {
      wt_data.commands[i] = configurations[stage][i];
    }

  } else {
    // Set everything to 0 by default
    for (int i=0; i<4; i++) {
      wt_data.commands[i] = 0;
    }

    wt_data.counter = 0;
  }

}