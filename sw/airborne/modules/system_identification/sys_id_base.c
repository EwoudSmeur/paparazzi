/*
 * Copyright (C) 2015
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
 * @file modules/system_identification/sys_id_base.c
 * @brief module that executes sys_id maneuvers
 *
 */

#include "modules/system_identification/sys_id_base.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/actuators.h"

struct sys_id_base_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;

} sys_id_base;

float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
float ctrl_module_demo_pr_d_gain;
float ctrl_module_demo_y_ff_gain;   // Yaw
float ctrl_module_demo_y_d_gain;
int16_t signal_type;

struct doublet_config_t {
  float amplitude;
  float timestep;
} doublet_config;

float trim_state[4];
float final_trim[4];

void sys_id_base_init(void);
void sys_id_base_run(bool in_flight);
void sys_id_base_enter(void);

void sys_id_base_init(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averaged_command

  sys_id_base.rc_x = 0;
  sys_id_base.rc_y = 0;
  sys_id_base.rc_z = 0;
  sys_id_base.rc_t = 0;

  doublet_config.amplitude = 0;
  doublet_config.timestep = 1.0;
}

void sys_id_base_enter(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averaged_command

  sys_id_base.rc_x = 0;
  sys_id_base.rc_y = 0;
  sys_id_base.rc_z = 0;
  sys_id_base.rc_t = 0;

  final_trim[0] = trim_state[0];
  final_trim[1] = trim_state[1];
}

// simple rate control without reference model nor attitude
void sys_id_base_run(UNUSED bool in_flight)
{
  // TODO Run signal generation
  // if(type == doublet) {
  //   doublet_signal();
  // } else if(...)
  //

   // Bound the inputs to the actuators
  // for each actuator
  /*if it is a servo:*/
      /*BoundAbs(command, MAX_PPRZ);*/
    //else
      /*Bound(command, 0, MAX_PPRZ);*/

  // TODO Execute commands
  /*actuators_pprz[0] = final_trim[0] + sys_id_signal*/
  /*actuators_pprz[1] = ..*/
}

// Only use for things that need to run when the module is not active!
// For instance keeping track of the trim condition
void sys_id_base_periodic(void)
{
  //TODO average out commands (actuators_pprz)
  //
  /*trim_state[0] =  trim_state[0] + factor*(actuators_pprz[0] - trim_state[0]);*/
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  sys_id_base_init();
}

void guidance_h_module_enter(void)
{
  sys_id_base_enter();
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  sys_id_base.rc_t = radio_control.values[RADIO_THROTTLE];
  sys_id_base.rc_x = radio_control.values[RADIO_ROLL];
  sys_id_base.rc_y = radio_control.values[RADIO_PITCH];
  sys_id_base.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  sys_id_base_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
  // not needed
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
  // not needed
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
  // not needed
}
