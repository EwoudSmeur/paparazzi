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

#ifndef STABILIZATION_ATTITUDE_QUAT_INDI_H
#define STABILIZATION_ATTITUDE_QUAT_INDI_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"
struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

extern struct FloatRates inv_control_effectiveness;
extern struct ReferenceSystem reference_acceleration;
extern float act_obs_rpm[ACTUATORS_NB];

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
extern int32_t indi_u_in_estimation_i[4];
extern float G1[3][4];
extern float G2[4];
extern float indi_u_in_actuators[4];

extern float vv_gain;

extern float pgain[3];
extern float dgain[3];
extern float igain[3];

void stabilization_indi_filter_gyro(void);
void stabilization_indi_filter_inputs(void);
void lms_estimation(void);
void filter_inputs_actuators(void);
void calc_g1_element(float du_norm, float dx_error, int8_t i, int8_t j, float mu_extra);
void calc_g2_element(float dx_error, int8_t j, float mu_extra);
void filter_estimation_indi(void);
void calc_g1g2_pseudo_inv(void);
void stabilization_indi_filter_accel(void);

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_H */

