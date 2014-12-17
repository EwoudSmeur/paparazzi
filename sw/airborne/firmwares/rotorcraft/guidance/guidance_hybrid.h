/*
 * Copyright (C) 2014 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
 * This is code for guidance of hybrid UAVs. It needs a simple velocity
 * model to control the ground velocity of the UAV while estimating the
 * wind velocity.
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

#ifndef GUIDANCE_HYBRID_H
#define GUIDANCE_HYBRID_H

#include "math/pprz_algebra_int.h"

extern int32_t guidance_hybrid_norm_ref_airspeed;

extern void guidance_hybrid_run(void);
extern void guidance_hybrid_init(void);
extern void guidance_hybrid_set_cmd_i(struct Int32Eulers *sp_cmd);
extern void guidance_h_airspeed_to_attitude(struct Int32Eulers *ypr_sp);
extern void guidance_hybrid_airspeed_to_attitude(struct Int32Eulers *ypr_sp);
extern void guidance_hybrid_position_to_airspeed(void);
extern void guidance_hybrid_determine_wind_estimate(void);
extern void guidance_hybrid_reset_heading(struct Int32Eulers *sp_cmd);
extern void guidance_hybrid_vertical(void);


#endif /* GUIDANCE_HYBRID_H */