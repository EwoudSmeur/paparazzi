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

/** @file stabilization_attitude_quat_indi.c
 * Rotorcraft quaternion attitude stabilization INDI control
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

//for step input
#include "subsystems/radio_control.h"


int32_t stabilization_att_indi_cmd[COMMANDS_NB];
struct FloatRates inv_control_effectiveness = {STABILIZATION_INDI_CONTROL_EFFECTIVENESS_P, STABILIZATION_INDI_CONTROL_EFFECTIVENESS_Q, STABILIZATION_INDI_CONTROL_EFFECTIVENESS_R};
struct ReferenceSystem reference_acceleration = {
  STABILIZATION_INDI_REF_ERR_P,
  STABILIZATION_INDI_REF_ERR_Q,
  STABILIZATION_INDI_REF_ERR_R,
  STABILIZATION_INDI_REF_RATE_P,
  STABILIZATION_INDI_REF_RATE_Q,
  STABILIZATION_INDI_REF_RATE_R,
};

float filtered_accelz = 0;
float filtered_accelz_deriv = 0;
float filtered_accelz_2deriv = 0;
float accel_ref = 0;

float vertical_velocity_rc = 0;
float vertical_velocity_err = 0;
float vv_gain = 6.0;
float z_filt = 0;
float altitude_sp = 0;

struct FloatRates filtered_rate = {0., 0., 0.};
struct FloatRates filtered_rate_deriv = {0., 0., 0.};
struct FloatRates filtered_rate_2deriv = {0., 0., 0.};
struct FloatRates angular_accel_ref = {0., 0., 0.};
struct FloatRates indi_u = {0., 0., 0.};
struct FloatRates indi_du = {0., 0., 0.};

float att_err_x = 0;
struct FloatRates u_act_dyn = {0., 0., 0.};
struct FloatRates u_in = {0., 0., 0.};
struct FloatRates udot = {0., 0., 0.};
struct FloatRates udotdot = {0., 0., 0.};
struct FloatRates filt_rate = {0., 0., 0.};
float act_obs_rpm[ACTUATORS_NB];

float dx_error_disp[3];
int32_t step_timer = 0;

float pgain[3] = {11250, 16200, 30000};
float dgain[3] = {498, 720, 1200};
float igain[3] = {30, 30, 30};
float sum_error[3] = {0, 0, 0};

struct FloatQuat quat_saved;

struct FloatMat33 G1G2_trans_mult;
struct FloatMat33 G1G2inv;
float G2_times_du = 0;
int32_t indi_u_in_estimation_i[4] = {0, 0, 0, 0};
float indi_du_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float u_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float udot_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float udotdot_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float u_actuators[4] = {0.0, 0.0, 0.0, 0.0};
float udot_actuators[4] = {0.0, 0.0, 0.0, 0.0};
float udotdot_actuators[4] = {0.0, 0.0, 0.0, 0.0};
float u_act_dyn_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float indi_u_in_actuators[4] = {0.0, 0.0, 0.0, 0.0};
float indi_du_in_actuators[4] = {0.0, 0.0, 0.0, 0.0};
float u_act_dyn_actuators[4] = {0.0, 0.0, 0.0, 0.0};
struct FloatRates rate_estimation = {0., 0., 0.};
struct FloatRates ratedot_estimation = {0., 0., 0.};
struct FloatRates ratedotdot_estimation = {0., 0., 0.};
float u_in_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float indi_u_in_estimation[4] = {0.0, 0.0, 0.0, 0.0};
// float G1G2_pseudo_inv[4][3] = {{ -14.0 , 18.0, 4.0},
// { 14.0, 18.0, -4.0},
// { 14.0, -18.0, 4.0},
// {-14.0 , -18.0, -4.0}};
float G1G2_pseudo_inv[4][3] = {{-12.5000,   17.8571,    4.0984},
{  12.5000,   17.8571,   -4.0984},
{  12.5000,  -17.8571,    4.0984},
{ -12.5000,  -17.8571,   -4.0984}};
float G1[3][4] = {{-20 , 20, 20 , -20 }, //scaled by 1000
{14 , 14, -14 , -14 },
{1, -1, 1, -1}};
float G2[4] = {60.0, -60.0, 60.0, -60.0}; //scaled by 1000
float G1G2[3][4] = {{-0.01 , 0.01 , 0.01 , -0.01 },
{0.01 , 0.01, -0.01 , -0.01 },
{-0.0025, 0.0025, -0.0025, 0.0025}};
float G1_new[3][4] = {{0.015 , -0.015, -0.015 , 0.015 },
{0.015 , 0.015, -0.015 , -0.015 },
{-0.0025, 0.0025, -0.0025, 0.0025}};
float G2_new[4];
float mu1 = 0.00001;
float mu2 = 0.00001*600.0;
float dx_estimation[3] = {0.0, 0.0, 0.0};
float du_estimation[4] = {0.0, 0.0, 0.0, 0.0};
float ddu_estimation[4] = {0.0, 0.0, 0.0, 0.0};

static const int32_t roll_coef[MOTOR_MIXING_NB_MOTOR]   = MOTOR_MIXING_ROLL_COEF;
static const int32_t pitch_coef[MOTOR_MIXING_NB_MOTOR]  = MOTOR_MIXING_PITCH_COEF;
static const int32_t yaw_coef[MOTOR_MIXING_NB_MOTOR]    = MOTOR_MIXING_YAW_COEF;

abi_event rpm_ev;
static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t count);

#define STABILIZATION_INDI_FILT_OMEGA2 (STABILIZATION_INDI_FILT_OMEGA*STABILIZATION_INDI_FILT_OMEGA)

#ifndef STABILIZATION_INDI_FILT_OMEGA_R
#define STABILIZATION_INDI_FILT_OMEGA_R STABILIZATION_INDI_FILT_OMEGA
#define STABILIZATION_INDI_FILT_ZETA_R STABILIZATION_INDI_FILT_ZETA
#endif

#define STABILIZATION_INDI_FILT_OMEGA2_R (STABILIZATION_INDI_FILT_OMEGA_R*STABILIZATION_INDI_FILT_OMEGA_R)

#define IDENTIFICATION_INDI_FILT_OMEGA 25
#define IDENTIFICATION_INDI_FILT_OMEGA2 625
#define IDENTIFICATION_FILT_ZETA 0.55


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_ref_quat.qi,
                              &stab_att_ref_quat.qx,
                              &stab_att_ref_quat.qy,
                              &stab_att_ref_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}

static void send_att_indi(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_INDI(trans, dev, AC_ID,
                                   &G1[0][0],
                                   &G1[0][1],
                                   &G1[0][2],
                                   &G1[0][3],
                                   &G1[1][0],
                                   &G1[1][1],
                                   &G1[1][2],
                                   &G1[1][3],
                                   &G1[2][0],
                                   &G1[2][1],
                                   &G1[2][2],
                                   &G1[2][3]);
}

static void send_att_indi2(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_INDI2(trans, dev, AC_ID,
                                   &vertical_velocity_rc,
                                   &vertical_velocity_err,
                                   &stateGetPositionNed_f()->z,
                                   &z_filt,
                                   &stateGetAccelNed_f()->x,
                                   &stateGetAccelNed_f()->y,
                                   &G1G2_pseudo_inv[2][1],
                                   &G1G2_pseudo_inv[3][1],
                                   &G1G2_pseudo_inv[0][2],
                                   &G1G2_pseudo_inv[1][2],
                                   &G1G2_pseudo_inv[2][2],
                                   &G1G2_pseudo_inv[3][2],
                                   &rc_accel_roll,
                                   &roll_in,
                                   &speedpitch,
                                   &speedroll);
}
#endif

void stabilization_attitude_init(void)
{
  stabilization_attitude_ref_init();

  // Register to RPM messages
  AbiBindMsgRPM(ABI_BROADCAST, &rpm_ev, rpm_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_INDI", send_att_indi);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_INDI2", send_att_indi2);
#endif
}

#define BOUND_CONTROLS(_v, _min, _max) { \
_v = _v < _min ? _min : _v > _max ? _max : _v; \
}

#define VECT4_ADD(_a, _b, _c) { \
_a[0] = _b[0] + _c[0]; \
_a[1] = _b[1] + _c[1]; \
_a[2] = _b[2] + _c[2]; \
_a[3] = _b[3] + _c[3]; \
}
#define VECT4_SIMPLE_MULT(_a, _b, _c) { \
_a[0] = _b[0] * _c; \
_a[1] = _b[1] * _c; \
_a[2] = _b[2] * _c; \
_a[3] = _b[3] * _c; \
}
#define VECT4_MULT(_a, _b, _c) { \
_a[0] = _b[0] * _c[0]; \
_a[1] = _b[1] * _c[1]; \
_a[2] = _b[2] * _c[2]; \
_a[3] = _b[3] * _c[3]; \
}
#define VECT4_ZERO(_a) { \
_a[0] = 0; \
_a[1] = 0; \
_a[2] = 0; \
_a[3] = 0; \
}
#define VECT4_INTEGRATE(_a, _b, _c) { \
_a[0] = _a[0] + _b[0]/_c; \
_a[1] = _a[1] + _b[1]/_c; \
_a[2] = _a[2] + _b[2]/_c; \
_a[3] = _a[3] + _b[3]/_c; \
}
#define RATES_INTEGRATE(_a, _b, _c) { \
_a.p = _a.p + _b.p/_c; \
_a.q = _a.q + _b.q/_c; \
_a.r = _a.r + _b.r/_c; \
}

void stabilization_attitude_enter(void)
{
  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  FLOAT_RATES_ZERO(filtered_rate);
  FLOAT_RATES_ZERO(filtered_rate_deriv);
  FLOAT_RATES_ZERO(filtered_rate_2deriv);
  FLOAT_RATES_ZERO(angular_accel_ref);
  FLOAT_RATES_ZERO(indi_u);
  FLOAT_RATES_ZERO(indi_du);
  FLOAT_RATES_ZERO(u_act_dyn);
  FLOAT_RATES_ZERO(u_in);
  FLOAT_RATES_ZERO(udot);
  FLOAT_RATES_ZERO(udotdot);
  FLOAT_RATES_ZERO(filt_rate);
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

static void attitude_run_pid(int32_t pid_commands[], struct Int32Quat *att_err) {

  sum_error[0] = sum_error[0] + QUAT1_FLOAT_OF_BFP(att_err->qx);
  sum_error[1] = sum_error[1] + QUAT1_FLOAT_OF_BFP(att_err->qy);
  sum_error[2] = sum_error[2] + QUAT1_FLOAT_OF_BFP(att_err->qz);

  if(stabilization_cmd[COMMAND_THRUST] < 800) {
    sum_error[0] = 0;
    sum_error[1] = 0;
    sum_error[2] = 0;
  }

  /*  PID feedback */
  pid_commands[COMMAND_ROLL] = pgain[0] * QUAT1_FLOAT_OF_BFP(att_err->qx) - dgain[0] * stateGetBodyRates_f()->p + igain[0] * sum_error[0];
  pid_commands[COMMAND_PITCH] = pgain[1] * QUAT1_FLOAT_OF_BFP(att_err->qy) - dgain[1] * stateGetBodyRates_f()->q + igain[1] * sum_error[1];
  pid_commands[COMMAND_YAW] = pgain[2] * QUAT1_FLOAT_OF_BFP(att_err->qz) - dgain[2] * stateGetBodyRates_f()->r + igain[2] * sum_error[2];
}

static void stabilization_indi_filter_z(void) {
  z_filt = z_filt + 0.01*(stateGetPositionNed_f()->z - z_filt);
}

static void attitude_run_indi(int32_t indi_commands[], struct Int32Quat *att_err)
{
  angular_accel_ref.p = reference_acceleration.err_p * QUAT1_FLOAT_OF_BFP(att_err->qx)
                        - reference_acceleration.rate_p * stateGetBodyRates_f()->p;
  angular_accel_ref.q = reference_acceleration.err_q * QUAT1_FLOAT_OF_BFP(att_err->qy)
                        - reference_acceleration.rate_q * stateGetBodyRates_f()->q;
  angular_accel_ref.r = reference_acceleration.err_r * QUAT1_FLOAT_OF_BFP(att_err->qz)
                        - reference_acceleration.rate_r * stateGetBodyRates_f()->r;

  indi_du.p = inv_control_effectiveness.p * (angular_accel_ref.p - filtered_rate_deriv.p);
  indi_du.q = inv_control_effectiveness.q * (angular_accel_ref.q - filtered_rate_deriv.q);
  indi_du.r = inv_control_effectiveness.r * (angular_accel_ref.r - filtered_rate_deriv.r);

  indi_u_in_estimation[0] = (float) motor_mixing.commands[0];
  indi_u_in_estimation[1] = (float) motor_mixing.commands[1];
  indi_u_in_estimation[2] = (float) motor_mixing.commands[2];
  indi_u_in_estimation[3] = (float) motor_mixing.commands[3];

  G2_times_du = (G2[0]/1000.0*indi_du_in_actuators[0] + G2[1]/1000.0*indi_du_in_actuators[1] + G2[2]/1000.0*indi_du_in_actuators[2] + G2[3]/1000.0*indi_du_in_actuators[3]);

  indi_du_in_actuators[0] = (G1G2_pseudo_inv[0][0] * (angular_accel_ref.p - filtered_rate_deriv.p)) + (G1G2_pseudo_inv[0][1] * (angular_accel_ref.q - filtered_rate_deriv.q)) + (G1G2_pseudo_inv[0][2] * (angular_accel_ref.r - filtered_rate_deriv.r + G2_times_du));
  indi_du_in_actuators[1] = (G1G2_pseudo_inv[1][0] * (angular_accel_ref.p - filtered_rate_deriv.p)) + (G1G2_pseudo_inv[1][1] * (angular_accel_ref.q - filtered_rate_deriv.q)) + (G1G2_pseudo_inv[1][2] * (angular_accel_ref.r - filtered_rate_deriv.r + G2_times_du));
  indi_du_in_actuators[2] = (G1G2_pseudo_inv[2][0] * (angular_accel_ref.p - filtered_rate_deriv.p)) + (G1G2_pseudo_inv[2][1] * (angular_accel_ref.q - filtered_rate_deriv.q)) + (G1G2_pseudo_inv[2][2] * (angular_accel_ref.r - filtered_rate_deriv.r + G2_times_du));
  indi_du_in_actuators[3] = (G1G2_pseudo_inv[3][0] * (angular_accel_ref.p - filtered_rate_deriv.p)) + (G1G2_pseudo_inv[3][1] * (angular_accel_ref.q - filtered_rate_deriv.q)) + (G1G2_pseudo_inv[3][2] * (angular_accel_ref.r - filtered_rate_deriv.r + G2_times_du));

  indi_u_in_actuators[0] = u_actuators[0] + indi_du_in_actuators[0];
  indi_u_in_actuators[1] = u_actuators[1] + indi_du_in_actuators[1];
  indi_u_in_actuators[2] = u_actuators[2] + indi_du_in_actuators[2];
  indi_u_in_actuators[3] = u_actuators[3] + indi_du_in_actuators[3];

  //filter the accelerometer ned z axis
  stabilization_indi_filter_accel();
  stabilization_indi_filter_z();

  vertical_velocity_rc = -(stabilization_cmd[COMMAND_THRUST]-4500.0)/4500.0*2.0;
  vertical_velocity_err = vertical_velocity_rc - stateGetSpeedNed_f()->z;

  altitude_sp = -(stabilization_cmd[COMMAND_THRUST]-2500.0)/4500.0*2.0;

  float altitude_gain = 0.5;
  float vertical_velocity_sp = altitude_gain*(altitude_sp - stateGetPositionNed_f()->z);

  vertical_velocity_err = vertical_velocity_sp - stateGetSpeedNed_f()->z;


  float inv_control_eff_accel = -500.0;
//   accel_ref = -(stabilization_cmd[COMMAND_THRUST]-4500.0)/4500.0*1.0;
  accel_ref = vertical_velocity_err*vv_gain;
  indi_u_in_actuators[0] = indi_u_in_actuators[0] + inv_control_eff_accel*(accel_ref - filtered_accelz);
  indi_u_in_actuators[1] = indi_u_in_actuators[1] + inv_control_eff_accel*(accel_ref - filtered_accelz);
  indi_u_in_actuators[2] = indi_u_in_actuators[2] + inv_control_eff_accel*(accel_ref - filtered_accelz);
  indi_u_in_actuators[3] = indi_u_in_actuators[3] + inv_control_eff_accel*(accel_ref - filtered_accelz);

  float avg_u_in = (indi_u_in_actuators[0] + indi_u_in_actuators[1] + indi_u_in_actuators[2] + indi_u_in_actuators[3])/4.0;

#warning "saturation at 8100 instead of max_pprz because of bebop motors"
  Bound(stabilization_cmd[COMMAND_THRUST],0, 8100);

//   if(avg_u_in > 1.0) { //avoid dividing by zero
//     indi_u_in_actuators[0] = indi_u_in_actuators[0] /avg_u_in * stabilization_cmd[COMMAND_THRUST];
//     indi_u_in_actuators[1] = indi_u_in_actuators[1] /avg_u_in * stabilization_cmd[COMMAND_THRUST];
//     indi_u_in_actuators[2] = indi_u_in_actuators[2] /avg_u_in * stabilization_cmd[COMMAND_THRUST];
//     indi_u_in_actuators[3] = indi_u_in_actuators[3] /avg_u_in * stabilization_cmd[COMMAND_THRUST];
//   }

  Bound(indi_u_in_actuators[0],0,8100);
  Bound(indi_u_in_actuators[1],0,8100);
  Bound(indi_u_in_actuators[2],0,8100);
  Bound(indi_u_in_actuators[3],0,8100);

  indi_u_in_estimation_i[0] = (int32_t) indi_u_in_actuators[0];
  indi_u_in_estimation_i[1] = (int32_t) indi_u_in_actuators[1];
  indi_u_in_estimation_i[2] = (int32_t) indi_u_in_actuators[2];
  indi_u_in_estimation_i[3] = (int32_t) indi_u_in_actuators[3];

//   stabilization_indi_filter_inputs();
  filter_inputs_actuators();
  filter_estimation_indi();

  u_in.p = indi_u.p + indi_du.p;
  u_in.q = indi_u.q + indi_du.q;
  u_in.r = indi_u.r + indi_du.r;

  Bound(u_in.p, -4500, 4500);
  Bound(u_in.q, -4500, 4500);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST] / 2);
  Bound(u_in.r, -half_thrust, half_thrust);

  //Don't increment if thrust is off
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    indi_u_in_actuators[0] = 0;
    indi_u_in_actuators[1] = 0;
    indi_u_in_actuators[2] = 0;
    indi_u_in_actuators[3] = 0;
    indi_u_in_estimation_i[0] = 0;
    indi_u_in_estimation_i[1] = 0;
    indi_u_in_estimation_i[2] = 0;
    indi_u_in_estimation_i[3] = 0;
    FLOAT_RATES_ZERO(indi_u);
    FLOAT_RATES_ZERO(indi_du);
    FLOAT_RATES_ZERO(u_act_dyn);
    FLOAT_RATES_ZERO(u_in);
    FLOAT_RATES_ZERO(udot);
    FLOAT_RATES_ZERO(udotdot);
  }
  else {
#if ADAPTIVE_INDI
    lms_estimation();
#endif
  }

  /*  INDI feedback */
  indi_commands[COMMAND_ROLL] = u_in.p;
  indi_commands[COMMAND_PITCH] = u_in.q;
  indi_commands[COMMAND_YAW] = u_in.r;
}

void stabilization_attitude_run(bool_t enable_integrator)
{

  /* Propagate the second order filter on the gyroscopes */
  stabilization_indi_filter_gyro();

  /*
   * Compute error for feedback
   */

#if STEP_INPUT_ON_MODE_AUTO1
#warning "Using step input on auto1!!! Only for testing/experiment!!"
  struct FloatQuat q_sp;
//   if((radio_control.values[RADIO_MODE] > -4000) && (step_timer < 256)) {
  if((radio_control.values[RADIO_MODE] > -4000) && (step_timer < 512)) {
    //doublet input
//     if(step_timer < 128) {
    if(step_timer < 256) {
//       struct FloatEulers rotation_eulers = {0.5236, 0, 0};
      struct FloatEulers rotation_eulers = {0, 0, 0.08727};
      struct FloatQuat rotation_quat;
      float_quat_of_eulers(&rotation_quat, &rotation_eulers);
      float_quat_comp(&q_sp, &quat_saved, &rotation_quat);
      float_quat_normalize(&q_sp);
    }
    else {
//       struct FloatEulers rotation_eulers = {-0.5236, 0, 0};
      struct FloatEulers rotation_eulers = {0, 0, -0.08727};
      struct FloatQuat rotation_quat;
      float_quat_of_eulers(&rotation_quat, &rotation_eulers);
      float_quat_comp(&q_sp, &quat_saved, &rotation_quat);
      float_quat_normalize(&q_sp);
    }
    step_timer = step_timer + 1;
    QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
  }
  else if(radio_control.values[RADIO_MODE] < -4000) {
    //normal flying
    step_timer = 0;
    QUAT_FLOAT_OF_BFP(q_sp, stab_att_sp_quat);
    QUAT_COPY(quat_saved,q_sp);
  }
#endif

  struct FloatQuat q_sp;
  stabilization_attitude_calc_setpoint(&q_sp, enable_integrator, FALSE);
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);
  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /* compute the INDI command */
#if ATTITUDE_PID
  attitude_run_pid(stabilization_att_indi_cmd, &att_err);
#else
  attitude_run_indi(stabilization_att_indi_cmd, &att_err);
#endif

  stabilization_cmd[COMMAND_ROLL] = stabilization_att_indi_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_indi_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_indi_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

void stabilization_indi_filter_gyro(void)
{
  filtered_rate.p = filtered_rate.p + filtered_rate_deriv.p / 512.0;
  filtered_rate.q = filtered_rate.q + filtered_rate_deriv.q / 512.0;
  filtered_rate.r = filtered_rate.r + filtered_rate_deriv.r / 512.0;

  filtered_rate_deriv.p = filtered_rate_deriv.p + filtered_rate_2deriv.p / 512.0;
  filtered_rate_deriv.q = filtered_rate_deriv.q + filtered_rate_2deriv.q / 512.0;
  filtered_rate_deriv.r = filtered_rate_deriv.r + filtered_rate_2deriv.r / 512.0;

  filtered_rate_2deriv.p = -filtered_rate_deriv.p * 2 * STABILIZATION_INDI_FILT_ZETA * STABILIZATION_INDI_FILT_OMEGA + (stateGetBodyRates_f()->p - filtered_rate.p) * STABILIZATION_INDI_FILT_OMEGA2;
  filtered_rate_2deriv.q = -filtered_rate_deriv.q * 2 * STABILIZATION_INDI_FILT_ZETA * STABILIZATION_INDI_FILT_OMEGA + (stateGetBodyRates_f()->q - filtered_rate.q) * STABILIZATION_INDI_FILT_OMEGA2;
  filtered_rate_2deriv.r = -filtered_rate_deriv.r * 2 * STABILIZATION_INDI_FILT_ZETA_R * STABILIZATION_INDI_FILT_OMEGA_R + (stateGetBodyRates_f()->r - filtered_rate.r) * STABILIZATION_INDI_FILT_OMEGA2_R;
}

void stabilization_indi_filter_accel(void)
{
  filtered_accelz = filtered_accelz + filtered_accelz_deriv / 512.0;

  filtered_accelz_deriv = filtered_accelz_deriv + filtered_accelz_2deriv / 512.0;

  filtered_accelz_2deriv = -filtered_accelz_deriv * 2 * STABILIZATION_INDI_FILT_ZETA * STABILIZATION_INDI_FILT_OMEGA + (stateGetAccelNed_f()->z - filtered_accelz) * STABILIZATION_INDI_FILT_OMEGA2;
}


void filter_inputs_actuators(void) {
#ifdef INDI_RPM_FEEDBACK
  u_act_dyn_actuators[0] = act_obs_rpm[0];
  u_act_dyn_actuators[1] = act_obs_rpm[1];
  u_act_dyn_actuators[2] = act_obs_rpm[2];
  u_act_dyn_actuators[3] = act_obs_rpm[3];
#else
#warning "not using rpm feedback"
  //actuator dynamics
  u_act_dyn_actuators[0] = u_act_dyn_actuators[0] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_actuators[0] - u_act_dyn_actuators[0]);
  u_act_dyn_actuators[1] = u_act_dyn_actuators[1] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_actuators[1] - u_act_dyn_actuators[1]);
  u_act_dyn_actuators[2] = u_act_dyn_actuators[2] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_actuators[2] - u_act_dyn_actuators[2]);
  u_act_dyn_actuators[3] = u_act_dyn_actuators[3] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_actuators[3] - u_act_dyn_actuators[3]);
#endif

  //Sensor dynamics (same filter as on gyro measurements)
  VECT4_INTEGRATE(u_actuators,udot_actuators,512.0);

  VECT4_INTEGRATE(udot_actuators,udotdot_actuators,512.0);

  udotdot_actuators[0] = -udot_actuators[0] * 2*STABILIZATION_INDI_FILT_ZETA*STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn_actuators[0] - u_actuators[0])*STABILIZATION_INDI_FILT_OMEGA2;
  udotdot_actuators[1] = -udot_actuators[1] * 2*STABILIZATION_INDI_FILT_ZETA*STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn_actuators[1] - u_actuators[1])*STABILIZATION_INDI_FILT_OMEGA2;
  udotdot_actuators[2] = -udot_actuators[2] * 2*STABILIZATION_INDI_FILT_ZETA*STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn_actuators[2] - u_actuators[2])*STABILIZATION_INDI_FILT_OMEGA2;
  udotdot_actuators[3] = -udot_actuators[3] * 2*STABILIZATION_INDI_FILT_ZETA*STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn_actuators[3] - u_actuators[3])*STABILIZATION_INDI_FILT_OMEGA2;
}

void stabilization_indi_filter_inputs(void)
{
#ifdef INDI_RPM_FEEDBACK
  u_act_dyn.p = (-act_obs_rpm[0] + act_obs_rpm[1] + act_obs_rpm[2] - act_obs_rpm[3]) / 4.0;
  u_act_dyn.q = (act_obs_rpm[0] + act_obs_rpm[1] - act_obs_rpm[2] - act_obs_rpm[3]) / 4.0;
  u_act_dyn.r = (act_obs_rpm[0] - act_obs_rpm[1] + act_obs_rpm[2] - act_obs_rpm[3]) / 4.0;
#else
  //actuator dynamics
  u_act_dyn.p = u_act_dyn.p + STABILIZATION_INDI_ACT_DYN_P * (u_in.p - u_act_dyn.p);
  u_act_dyn.q = u_act_dyn.q + STABILIZATION_INDI_ACT_DYN_Q * (u_in.q - u_act_dyn.q);
  u_act_dyn.r = u_act_dyn.r + STABILIZATION_INDI_ACT_DYN_R * (u_in.r - u_act_dyn.r);
#endif

  //Sensor dynamics (same filter as on gyro measurements)
  indi_u.p = indi_u.p + udot.p / 512.0;
  indi_u.q = indi_u.q + udot.q / 512.0;
  indi_u.r = indi_u.r + udot.r / 512.0;

  udot.p = udot.p + udotdot.p / 512.0;
  udot.q = udot.q + udotdot.q / 512.0;
  udot.r = udot.r + udotdot.r / 512.0;

  udotdot.p = -udot.p * 2 * STABILIZATION_INDI_FILT_ZETA * STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn.p - indi_u.p) * STABILIZATION_INDI_FILT_OMEGA2;
  udotdot.q = -udot.q * 2 * STABILIZATION_INDI_FILT_ZETA * STABILIZATION_INDI_FILT_OMEGA + (u_act_dyn.q - indi_u.q) * STABILIZATION_INDI_FILT_OMEGA2;
  udotdot.r = -udot.r * 2 * STABILIZATION_INDI_FILT_ZETA_R * STABILIZATION_INDI_FILT_OMEGA_R + (u_act_dyn.r - indi_u.r) * STABILIZATION_INDI_FILT_OMEGA2_R;
}

void filter_estimation_indi(void) {
#ifdef INDI_RPM_FEEDBACK
  u_act_dyn_estimation[0] = act_obs_rpm[0];
  u_act_dyn_estimation[1] = act_obs_rpm[1];
  u_act_dyn_estimation[2] = act_obs_rpm[2];
  u_act_dyn_estimation[3] = act_obs_rpm[3];
#else
  //actuator dynamics
  u_act_dyn_estimation[0] = u_act_dyn_estimation[0] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_estimation[0] - u_act_dyn_estimation[0]);
  u_act_dyn_estimation[1] = u_act_dyn_estimation[1] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_estimation[1] - u_act_dyn_estimation[1]);
  u_act_dyn_estimation[2] = u_act_dyn_estimation[2] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_estimation[2] - u_act_dyn_estimation[2]);
  u_act_dyn_estimation[3] = u_act_dyn_estimation[3] + STABILIZATION_INDI_ACT_DYN_P*( indi_u_in_estimation[3] - u_act_dyn_estimation[3]);
#endif

  //Sensor dynamics (same filter as on gyro measurements)
  VECT4_INTEGRATE(u_estimation,udot_estimation,512.0);

  VECT4_INTEGRATE(udot_estimation,udotdot_estimation,512.0);

  udotdot_estimation[0] = -udot_estimation[0] * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (u_act_dyn_estimation[0] - u_estimation[0])*IDENTIFICATION_INDI_FILT_OMEGA2;
  udotdot_estimation[1] = -udot_estimation[1] * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (u_act_dyn_estimation[1] - u_estimation[1])*IDENTIFICATION_INDI_FILT_OMEGA2;
  udotdot_estimation[2] = -udot_estimation[2] * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (u_act_dyn_estimation[2] - u_estimation[2])*IDENTIFICATION_INDI_FILT_OMEGA2;
  udotdot_estimation[3] = -udot_estimation[3] * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (u_act_dyn_estimation[3] - u_estimation[3])*IDENTIFICATION_INDI_FILT_OMEGA2;

  //Sensor dynamics (same filter as on gyro measurements)
  RATES_INTEGRATE(rate_estimation,ratedot_estimation,512.0);

  RATES_INTEGRATE(ratedot_estimation,ratedotdot_estimation,512.0);

  ratedotdot_estimation.p = -ratedot_estimation.p * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (stateGetBodyRates_f()->p - rate_estimation.p)*IDENTIFICATION_INDI_FILT_OMEGA2;
  ratedotdot_estimation.q = -ratedot_estimation.q * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (stateGetBodyRates_f()->q - rate_estimation.q)*IDENTIFICATION_INDI_FILT_OMEGA2;
  ratedotdot_estimation.r = -ratedot_estimation.r * 2*IDENTIFICATION_FILT_ZETA*IDENTIFICATION_INDI_FILT_OMEGA + (stateGetBodyRates_f()->r - rate_estimation.r)*IDENTIFICATION_INDI_FILT_OMEGA2;
}

static void rpm_cb(uint8_t sender_id, uint16_t *rpm, uint8_t count)
{
  for(int i = 0; i < count; i++) {
    act_obs_rpm[i] = (rpm[i] - get_servo_min(i));
    act_obs_rpm[i] *= (MAX_PPRZ / (float)(get_servo_max(i)-get_servo_min(i)));
  }
}

void calc_g1_element(float du_norm, float dx_error, int8_t i, int8_t j, float mu_extra) {
  G1_new[i][j] = G1[i][j] - du_estimation[j]*mu1*dx_error*mu_extra;
}

void calc_g2_element(float dx_error, int8_t j, float mu_extra) {
  G2_new[j] = G2[j] - ddu_estimation[j]*mu2*dx_error*mu_extra;
}

void lms_estimation(void) {
  dx_estimation[0] = ratedotdot_estimation.p;
  dx_estimation[1] = ratedotdot_estimation.q;
  dx_estimation[2] = ratedotdot_estimation.r;
  du_estimation[0] = udot_estimation[0]/1000.0;
  du_estimation[1] = udot_estimation[1]/1000.0;
  du_estimation[2] = udot_estimation[2]/1000.0;
  du_estimation[3] = udot_estimation[3]/1000.0;
  ddu_estimation[0] = udotdot_estimation[0]/1000.0/512.0;
  ddu_estimation[1] = udotdot_estimation[1]/1000.0/512.0;
  ddu_estimation[2] = udotdot_estimation[2]/1000.0/512.0;
  ddu_estimation[3] = udotdot_estimation[3]/1000.0/512.0;

  //Estimation of G
  float du_norm = du_estimation[0]*du_estimation[0] + du_estimation[1]*du_estimation[1] +du_estimation[2]*du_estimation[2] + du_estimation[3]*du_estimation[3];
  float dx_norm = dx_estimation[0]*dx_estimation[0] + dx_estimation[1]*dx_estimation[1] + dx_estimation[2]*dx_estimation[2];
//   if((dx_norm > 40000.0)) {
    for(int8_t i=0; i<3; i++) {
      float mu_extra = 1.0;
      float dx_error = G1[i][0]*du_estimation[0] + G1[i][1]*du_estimation[1] + G1[i][2]*du_estimation[2] + G1[i][3]*du_estimation[3] - dx_estimation[i];
      dx_error_disp[i] = dx_error;
      //if the yaw axis, also use G2
      if(i==2) {
        mu_extra = 0.3;
        dx_error = dx_error + G2[0]*ddu_estimation[0] + G2[1]*ddu_estimation[1] + G2[2]*ddu_estimation[2] + G2[3]*ddu_estimation[3];
        for(int8_t j=0; j<4; j++) {
          calc_g2_element(dx_error,j, mu_extra);
        }
      }
      for(int8_t j=0; j<4; j++) {
        calc_g1_element(du_norm, dx_error, i, j, mu_extra);
      }
    }

    for(int8_t j=0; j<4; j++) {
      G2[j] = G2_new[j];
      for(int8_t i=0; i<3; i++) {
        G1[i][j] = G1_new[i][j];
      }
    }

    calc_g1g2_pseudo_inv();
}

void calc_g1g2_pseudo_inv(void) {

  //sum of G1 and G2
  for(int8_t i=0; i<3; i++) {
    for(int8_t j=0; j<4; j++) {
      if(i<2)
        G1G2[i][j] = G1[i][j]/1000.0;
      else
        G1G2[i][j] = G1[i][j]/1000.0 + G2[j]/1000.0;
    }
  }

  //G1G2*transpose(G1G2)
  //calculate matrix multiplication of its transpose 3x4 x 4x3
  float element = 0;
  for(int8_t row=0; row<3; row++) {
    for(int8_t col=0; col<3; col++) {
      element = 0;
      for(int8_t i=0; i<4; i++) {
        element = element + G1G2[row][i]*G1G2[col][i];
      }
      MAT33_ELMT(G1G2_trans_mult,row,col) = element;
    }
  }

  //there are numerical errors if the scaling is not right.
  MAT33_MULT_SCALAR(G1G2_trans_mult,100.0);

  //inverse of 3x3 matrix
  MAT33_INV(G1G2inv,G1G2_trans_mult);

  //scale back
  MAT33_MULT_SCALAR(G1G2inv,100.0);

  //G1G2'*G1G2inv
  //calculate matrix multiplication 4x3 x 3x3
  for(int8_t row=0; row<4; row++) {
    for(int8_t col=0; col<3; col++) {
      element = 0;
      for(int8_t i=0; i<3; i++) {
        element = element + G1G2[i][row]*MAT33_ELMT(G1G2inv,col,i);
      }
      G1G2_pseudo_inv[row][col] = element;
    }
  }
}

