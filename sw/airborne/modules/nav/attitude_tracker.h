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

/** \file attitude_tracker.h
 *
 * 
 */

#ifndef ATTITUDE_TRACKER_H
#define ATTITUDE_TRACKER_H

#include "std.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

struct AttitudeTracker {
  struct FloatEulers att;
  float thrust;
  float last_update_time;
};


extern void attitude_tracker_init(void);
extern void at_read_msg(uint8_t *buf);
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif // ATTITUDE_TRACKER_H
