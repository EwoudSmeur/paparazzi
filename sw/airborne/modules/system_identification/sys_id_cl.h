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
 * @file modules/system_identification/sys_id_cl.h
 * @brief module that executes sys_id maneuvers in closed-loop
 *
 */

#ifndef SYS_ID_BASE_H
#define SYS_ID_BASE_H

#include <std.h>
#include "paparazzi.h"

// Settings
extern float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
extern float ctrl_module_demo_pr_d_gain;
extern float ctrl_module_demo_y_ff_gain;   // Yaw
extern float ctrl_module_demo_y_d_gain;

extern float time_roll_0;
extern uint8_t actuators_combination;

extern uint8_t signal_type_pitch;
extern uint8_t increase_first_pitch;
extern pprz_t amplitude_pitch;
extern float timestep_pitch;
extern float fstart_chirp_pitch;
extern float fend_chirp_pitch;
extern float length_chirp_pitch;
extern float chirp_noise_ratio_pitch;

extern uint8_t signal_type_thrust;
extern uint8_t increase_first_thrust;
extern pprz_t amplitude_thrust;
extern float timestep_thrust;
extern float fstart_chirp_thrust;
extern float fend_chirp_thrust;
extern float length_chirp_thrust;
extern float chirp_noise_ratio_thrust;

extern float time_shift;

// Bypass all controllers
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

extern void sys_id_base_periodic_init(void);
extern void sys_id_base_periodic(void);

#endif /* SYS_ID_BASE_H */
