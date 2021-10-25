/*
 * Copyright (C) 2014 OpenUAS
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
 *
 */

#include "attitude_tracker.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "subsystems/datalink/datalink.h"
#include "generated/modules.h"

struct AttitudeTracker attitude_tracker;

bool at_att_sp_valid(void);
void at_read_msg(uint8_t *buf);

void attitude_tracker_init(void)
{
  attitude_tracker.att.phi = 0.0;
  attitude_tracker.att.theta = 0.0;
  attitude_tracker.att.psi = 0.0;
  attitude_tracker.thrust = 0.0;
  attitude_tracker.last_update_time = get_sys_time_float();
}

void at_read_msg(uint8_t *buf)
{
  uint8_t ac_id = DL_TARGET_ATT_ac_id(buf);

  if (ac_id == AC_ID) {
    attitude_tracker.att.phi = DL_TARGET_ATT_phi(buf);
    attitude_tracker.att.theta = DL_TARGET_ATT_theta(buf);
    attitude_tracker.att.psi = DL_TARGET_ATT_psi(buf);
    attitude_tracker.thrust = DL_TARGET_ATT_thrust(buf);

    attitude_tracker.last_update_time = get_sys_time_float();
  }
}

bool at_att_sp_valid(void) {
  float cur_time = get_sys_time_float();

  if ((cur_time - attitude_tracker.last_update_time) < 0.5) {
    return true;
  } else {
    return false;
  }
}

void guidance_h_module_init(void) {
  // Nothing to be done
}

void guidance_h_module_enter(void) {
  // Nothing to be done
}

void guidance_h_module_read_rc(void) { 
  // Nothing to be done
}

void guidance_h_module_run(bool in_flight) {

  struct Int32Eulers sp_cmd_i;
  sp_cmd_i.phi = ANGLE_BFP_OF_REAL(attitude_tracker.att.phi);
  sp_cmd_i.theta = ANGLE_BFP_OF_REAL(attitude_tracker.att.theta);
  sp_cmd_i.psi = ANGLE_BFP_OF_REAL(attitude_tracker.att.psi);
  stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);

  stabilization_attitude_run(in_flight);
  stabilization_cmd[COMMAND_THRUST] = attitude_tracker.thrust;

  // Backup to NAV mode if not receiving attitude inputs
  if (!at_att_sp_valid()) {
    autopilot_static_SetModeHandler(AP_MODE_NAV);
  }
}

void guidance_v_module_init(void) {
  // Nothing to be done
}

void guidance_v_module_enter(void) {
  // Nothing to be done
}

void guidance_v_module_run(bool in_flight) {
  // Nothing to be done
}