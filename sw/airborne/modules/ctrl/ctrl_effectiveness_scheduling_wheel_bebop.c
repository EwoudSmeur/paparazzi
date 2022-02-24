/*
 * Copyright (C) 2017 Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file modules/ctrl/ctrl_effectiveness_scheduling.c
 * Module that interpolates gainsets in flight based on the transition percentage
 */

#include "modules/ctrl/ctrl_effectiveness_scheduling_wheel_bebop.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/radio_control/radio_control.h"
#include "modules/core/abi.h"

#if STABILIZATION_INDI_ALLOCATION_PSEUDO_INVERSE
#error "You need to use WLS control allocation for this module"
#endif

static float factor[INDI_OUTPUTS][INDI_NUM_ACT] = {{0.19/2, -0.19/2, -0.19/2, 0.19/2},
                                                  {0.155/2, 0.155/2, -0.155/2, -0.155/2},
                                                  {-1.,  1., -1.,  1.},
                                                  {-1., -1., -1., -1.}};

static float inertia[INDI_OUTPUTS] = {0.00125, 0.0015, 1.0, 1.0}; //scaled by INDI_G_SCALING

static float Tact[INDI_NUM_ACT] = {0.1};
static float dTdrps[INDI_NUM_ACT] = {0.1};
static float rpm_obs[INDI_NUM_ACT];

abi_event rpm_ev;
static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t num_act);

//Get the specified gains in the gainlibrary
void ctrl_eff_scheduling_init(void)
{
  AbiBindMsgRPM(RPM_SENSOR_ID, &rpm_ev, rpm_cb);
}

void ctrl_eff_scheduling_periodic(void)
{

  // estimate on ground:
  int8_t i;
  float thrust = 0;
  for (i = 0; i < INDI_NUM_ACT; i++) {
    thrust += Tact[i];
  }

  //if (-accelz > (thrust/9.81 )
  

  // Only modify roll and pitch
  int8_t j;
  for (i = 0; i < 2; i++) // loop over axes
  {
    for (j = 0; j < INDI_NUM_ACT; j++)
    {
      Bwls[i][j] = factor[i][j] * inertia[i] * dTdrps[j] / 60.0 / (9600/(9800-3000));
    }
  }
}

static void rpm_cb(uint8_t __attribute__((unused)) sender_id, uint16_t UNUSED *rpm, uint8_t UNUSED num_act)
{
  int8_t i;
  for (i = 0; i < num_act; i++)
  {
    rpm_obs[i] = rpm[i];
    Bound(rpm_obs[i], 3000, 9800);
    float rps = rpm_obs[i]/60.0;
    Tact[i] = 7.088e-5*rps*rps - 0.002283*rps + 0.08001;
    dTdrps[i] = 2*7.088e-5*rps - 0.002283;
  }
}

