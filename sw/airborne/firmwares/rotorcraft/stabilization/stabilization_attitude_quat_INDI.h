/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#ifndef STABILIZATION_ATTITUDE_QUAT_INT_INDI_H
#define STABILIZATION_ATTITUDE_QUAT_INT_INDI_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"

extern float kp_p;
extern float kd_p;
extern float m_c_p;
extern float kp_q;
extern float kd_q;
extern float m_c_q;
extern float kp_r;
extern float kd_r;
extern float m_c_r;
extern float m_c_r2;

extern struct FloatRates filtered_rate;
extern struct FloatRates filtered_rate_deriv;
extern struct FloatRates filtered_rate_2deriv;
extern struct FloatRates angular_accel_ref;
extern struct FloatRates indi_u;
extern struct FloatRates indi_du;
extern struct FloatRates u_act_dyn;
extern struct FloatRates u_in;
extern struct FloatRates udot;
extern struct FloatRates udotdot;

void stabilization_indi_filter_gyro(void);
void stabilization_indi_filter_inputs(void);
void stabilization_indi_adaptive_gains(void);

extern int32_t elevator_gain;
extern int32_t aileron_gain;

extern int32_t elevator_gain_goal;
extern int32_t aileron_gain_goal;

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */
