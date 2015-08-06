/*
 * Copyright (C) 2012-2013 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.c
 *  Read an attitude setpoint from the RC.
 */

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "generated/airframe.h"

#include "subsystems/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "guidance/guidance_v.h"

#ifndef STABILIZATION_ATTITUDE_DEADBAND_A
#define STABILIZATION_ATTITUDE_DEADBAND_A 0
#endif

#ifndef STABILIZATION_ATTITUDE_DEADBAND_E
#define STABILIZATION_ATTITUDE_DEADBAND_E 0
#endif

#ifndef OUTER_LOOP_INDI
#define OUTER_LOOP_INDI TRUE
#endif

#define YAW_DEADBAND_EXCEEDED()                                         \
  (radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
   radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)

float care_free_heading = 0;

float pv_gain = 0.5;
float sum_gain = 0.4;
float pos_x_err =0;
float pos_y_err =0;
float rc_speed_roll =0.0;
float rc_speed_pitch =0.0;
// float pos_gain = 1.8;
float pos_gain = 0.9;
float rc_accel_roll = 0;
float rc_accel_pitch = 0;
float roll_in = 0;
float roll_filt = 0;
float roll_filtd = 0;
float roll_filtdd = 0;
float pitch_in = 0;
float pitch_filt = 0;
float pitch_filtd = 0;
float pitch_filtdd = 0;
float filt_accely = 0;
float filt_accelyd = 0;
float filt_accelydd = 0;
float filt_accelx = 0;
float filt_accelxd = 0;
float filt_accelxdd = 0;

float filt_accelyn = 0;
float filt_accelydn = 0;
float filt_accelyddn = 0;
float filt_accelxn = 0;
float filt_accelxdn = 0;
float filt_accelxddn = 0;
float filt_accelzn = 0;
float filt_accelzdn = 0;
float filt_accelzddn = 0;

float speedpitch = 0;
float speedroll = 0;
// float auto_speed_gain = 3.5;//1.2;
float auto_speed_gain = 1.8;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 inputs;
struct FloatEulers indicontrol;

void indi_filter_attitude(void);

static int32_t get_rc_roll(void)
{
  const int32_t max_rc_phi = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
  int32_t roll = radio_control.values[RADIO_ROLL];
#if STABILIZATION_ATTITUDE_DEADBAND_A
  DeadBand(roll, STABILIZATION_ATTITUDE_DEADBAND_A);
  return roll * max_rc_phi / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_A);
#else
  return roll * max_rc_phi / MAX_PPRZ;
#endif
}

static int32_t get_rc_pitch(void)
{
  const int32_t max_rc_theta = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
  int32_t pitch = radio_control.values[RADIO_PITCH];
#if STABILIZATION_ATTITUDE_DEADBAND_E
  DeadBand(pitch, STABILIZATION_ATTITUDE_DEADBAND_E);
  return pitch * max_rc_theta / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_E);
#else
  return pitch * max_rc_theta / MAX_PPRZ;
#endif
}

static int32_t get_rc_yaw(void)
{
  const int32_t max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);
  int32_t yaw = radio_control.values[RADIO_YAW];
  DeadBand(yaw, STABILIZATION_ATTITUDE_DEADBAND_R);
  return yaw * max_rc_r / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_R);
}

static float get_rc_roll_f(void)
{
  int32_t roll = radio_control.values[RADIO_ROLL];
#if STABILIZATION_ATTITUDE_DEADBAND_A
  DeadBand(roll, STABILIZATION_ATTITUDE_DEADBAND_A);
  return roll * STABILIZATION_ATTITUDE_SP_MAX_PHI / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_A);
#else
  return roll * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ;
#endif
}

static float get_rc_pitch_f(void)
{
  int32_t pitch = radio_control.values[RADIO_PITCH];
#if STABILIZATION_ATTITUDE_DEADBAND_E
  DeadBand(pitch, STABILIZATION_ATTITUDE_DEADBAND_E);
  return pitch * STABILIZATION_ATTITUDE_SP_MAX_THETA / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_E);
#else
  return pitch * STABILIZATION_ATTITUDE_SP_MAX_THETA / MAX_PPRZ;
#endif
}

static inline float get_rc_yaw_f(void)
{
  int32_t yaw = radio_control.values[RADIO_YAW];
  DeadBand(yaw, STABILIZATION_ATTITUDE_DEADBAND_R);
  return yaw * STABILIZATION_ATTITUDE_SP_MAX_R / (MAX_PPRZ - STABILIZATION_ATTITUDE_DEADBAND_R);
}

/// reset the heading for care-free mode to current heading
void stabilization_attitude_reset_care_free_heading(void)
{
  care_free_heading = stateGetNedToBodyEulers_f()->psi;
}

/*   This is a different way to obtain yaw. It will not switch when going beyond 90 degrees pitch.
     However, when rolling more then 90 degrees in combination with pitch it switches. For a
     transition vehicle this is better as 90 degrees pitch will occur, but more than 90 degrees roll probably not. */
int32_t stabilization_attitude_get_heading_i(void)
{
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();

  int32_t heading;

  if (abs(att->phi) < INT32_ANGLE_PI_2) {
    int32_t sin_theta;
    PPRZ_ITRIG_SIN(sin_theta, att->theta);
    heading = att->psi - INT_MULT_RSHIFT(sin_theta, att->phi, INT32_TRIG_FRAC);
  } else if (ANGLE_FLOAT_OF_BFP(att->theta) > 0) {
    heading = att->psi - att->phi;
  } else {
    heading = att->psi + att->phi;
  }

  return heading;
}

float stabilization_attitude_get_heading_f(void)
{
  struct FloatEulers *att = stateGetNedToBodyEulers_f();

  float heading;

  if (abs(att->phi) < M_PI / 2) {
    heading = att->psi - sinf(att->theta) * att->phi;
  } else if (att->theta > 0) {
    heading = att->psi - att->phi;
  } else {
    heading = att->psi + att->phi;
  }

  return heading;
}


/** Read attitude setpoint from RC as euler angles
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  in_flight         true if in flight
 * @param[out] sp                attitude setpoint as euler angles
 */
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight, bool_t in_carefree,
    bool_t coordinated_turn)
{
  /* last time this function was called, used to calculate yaw setpoint update */
  static float last_ts = 0.f;

  sp->phi = get_rc_roll();
  sp->theta = get_rc_pitch();

  if (in_flight) {
    /* calculate dt for yaw integration */
    float dt = get_sys_time_float() - last_ts;
    /* make sure nothing drastically weird happens, bound dt to 0.5sec */
    Bound(dt, 0, 0.5);

    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
      sp->psi += get_rc_yaw() * dt;
      INT32_ANGLE_NORMALIZE(sp->psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      int32_t omega;
      const int32_t max_phi = ANGLE_BFP_OF_REAL(RadOfDeg(85.0));
      if (abs(sp->phi) < max_phi) {
        omega = ANGLE_BFP_OF_REAL(1.3 * tanf(ANGLE_FLOAT_OF_BFP(sp->phi)));
      } else { //max 60 degrees roll, then take constant omega
        omega = ANGLE_BFP_OF_REAL(1.3 * 1.72305 * ((sp->phi > 0) - (sp->phi < 0)));
      }

      sp->psi += omega * dt;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    const int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);

    int32_t heading = stabilization_attitude_get_heading_i();

    int32_t delta_psi = sp->psi - heading;
    INT32_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > delta_limit) {
      sp->psi = heading + delta_limit;
    } else if (delta_psi < -delta_limit) {
      sp->psi = heading - delta_limit;
    }
    INT32_ANGLE_NORMALIZE(sp->psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      int32_t cos_psi;
      int32_t sin_psi;
      int32_t temp_theta;
      int32_t care_free_delta_psi_i;

      care_free_delta_psi_i = sp->psi - ANGLE_BFP_OF_REAL(care_free_heading);

      INT32_ANGLE_NORMALIZE(care_free_delta_psi_i);

      PPRZ_ITRIG_SIN(sin_psi, care_free_delta_psi_i);
      PPRZ_ITRIG_COS(cos_psi, care_free_delta_psi_i);

      temp_theta = INT_MULT_RSHIFT(cos_psi, sp->theta, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->phi,
                   INT32_ANGLE_FRAC);
      sp->phi = INT_MULT_RSHIFT(cos_psi, sp->phi, INT32_ANGLE_FRAC) - INT_MULT_RSHIFT(sin_psi, sp->theta, INT32_ANGLE_FRAC);

      sp->theta = temp_theta;
    }
  } else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_i()->psi;
  }

  /* update timestamp for dt calculation */
  last_ts = get_sys_time_float();
}


void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool_t in_flight, bool_t in_carefree,
    bool_t coordinated_turn)
{
  /* last time this function was called, used to calculate yaw setpoint update */
  static float last_ts = 0.f;

  sp->phi = get_rc_roll_f();
  sp->theta = get_rc_pitch_f();

  if (in_flight) {
    /* calculate dt for yaw integration */
    float dt = get_sys_time_float() - last_ts;
    /* make sure nothing drastically weird happens, bound dt to 0.5sec */
    Bound(dt, 0, 0.5);

    /* do not advance yaw setpoint if within a small deadband around stick center or if throttle is zero */
    if (YAW_DEADBAND_EXCEEDED() && !THROTTLE_STICK_DOWN()) {
      sp->psi += get_rc_yaw_f() * dt;
      FLOAT_ANGLE_NORMALIZE(sp->psi);
    }
    if (coordinated_turn) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      float omega;
      const float max_phi = RadOfDeg(85.0);
      if (abs(sp->phi) < max_phi) {
        omega = 1.3 * tanf(sp->phi);
      } else { //max 60 degrees roll, then take constant omega
        omega = 1.3 * 1.72305 * ((sp->phi > 0) - (sp->phi < 0));
      }

      sp->psi += omega * dt;
    }
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
    // Make sure the yaw setpoint does not differ too much from the real yaw
    // to prevent a sudden switch at 180 deg
    float heading = stabilization_attitude_get_heading_f();

    float delta_psi = sp->psi - heading;
    FLOAT_ANGLE_NORMALIZE(delta_psi);
    if (delta_psi > STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT) {
      sp->psi = heading + STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    } else if (delta_psi < -STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT) {
      sp->psi = heading - STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT;
    }
    FLOAT_ANGLE_NORMALIZE(sp->psi);
#endif
    //Care Free mode
    if (in_carefree) {
      //care_free_heading has been set to current psi when entering care free mode.
      float cos_psi;
      float sin_psi;
      float temp_theta;

      float care_free_delta_psi_f = sp->psi - care_free_heading;

      FLOAT_ANGLE_NORMALIZE(care_free_delta_psi_f);

      sin_psi = sinf(care_free_delta_psi_f);
      cos_psi = cosf(care_free_delta_psi_f);

      temp_theta = cos_psi * sp->theta - sin_psi * sp->phi;
      sp->phi = cos_psi * sp->phi - sin_psi * sp->theta;

      sp->theta = temp_theta;
    }
  } else { /* if not flying, use current yaw as setpoint */
    sp->psi = stateGetNedToBodyEulers_f()->psi;
  }

  /* update timestamp for dt calculation */
  last_ts = get_sys_time_float();
}


/** Read roll/pitch command from RC as quaternion.
 * Interprets the stick positions as axes.
 * @param[out] q quaternion representing the RC roll/pitch input
 */
void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat *q, bool_t in_flight)
{
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = get_rc_roll_f();
  ov.y = get_rc_pitch_f();
  ov.z = 0.0;

  indi_filter_attitude();
  indi_filter_accel_ned();

  if(radio_control.values[RADIO_MODE] < -4000) {

  if(guidance_h_mode == GUIDANCE_H_MODE_HOVER) {

    float cospsi = cosf(stateGetNedToBodyEulers_f()->psi);
    float sinpsi = sinf(stateGetNedToBodyEulers_f()->psi);

    if(false) { //speed control
      rc_speed_roll = ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*10.0;
      rc_speed_pitch = ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*10.0;
    }
    else { //position control
      pos_x_err = (POS_FLOAT_OF_BFP(guidance_h_pos_sp.x) - stateGetPositionNed_f()->x);
      pos_y_err = POS_FLOAT_OF_BFP(guidance_h_pos_sp.y) - stateGetPositionNed_f()->y;
      rc_speed_pitch = -pos_gain*(cospsi*pos_x_err + sinpsi*pos_y_err);
      rc_speed_roll = pos_gain*(-sinpsi*pos_x_err + cospsi*pos_y_err);
    }

    speedpitch = cospsi*stateGetSpeedNed_f()->x + sinpsi*stateGetSpeedNed_f()->y;
    speedroll = -sinpsi*stateGetSpeedNed_f()->x + cospsi*stateGetSpeedNed_f()->y;
  //       float accelx = cospsi*stateGetAccelNed_f()->x + sinpsi*stateGetAccelNed_f()->y;
  //   float accely =-sinpsi*stateGetAccelNed_f()->x + cospsi*stateGetAccelNed_f()->y;

    rc_accel_roll = (rc_speed_roll - speedroll)*auto_speed_gain;
    rc_accel_pitch = (rc_speed_pitch + speedpitch)*auto_speed_gain;
//     rc_accel_roll = ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*3.0;
//     rc_accel_pitch = ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*3.0;
  }
  else {
    rc_accel_roll = ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*8.0;
    rc_accel_pitch = ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*8.0;
  }
  float inv_accel_dyn = 1.0/9.81;
  roll_in = roll_filt + inv_accel_dyn*(rc_accel_roll - filt_accely);
  pitch_in = pitch_filt + inv_accel_dyn*(rc_accel_pitch + filt_accelx);
  Bound(roll_in, -0.4, 0.4);
  Bound(pitch_in, -0.4, 0.4);
  ov.x = roll_in;
  ov.y = pitch_in;

  /* quaternion from that orientation vector */
  float_quat_of_orientation_vect(q, &ov);

  }
  else {
    if(OUTER_LOOP_INDI) {
      pos_x_err = POS_FLOAT_OF_BFP(guidance_h_pos_sp.x) - stateGetPositionNed_f()->x +-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*3.0;
      pos_y_err = POS_FLOAT_OF_BFP(guidance_h_pos_sp.y) - stateGetPositionNed_f()->y + ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*3.0;

      float speed_sp_x = pos_x_err*pos_gain;
      float speed_sp_y = pos_y_err*pos_gain;

  //     float accel_x = (-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*8.0 - stateGetSpeedNed_f()->x)*auto_speed_gain;
  //     float accel_y = (ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*8.0 - stateGetSpeedNed_f()->y)*auto_speed_gain;
      float accel_x = (speed_sp_x - stateGetSpeedNed_f()->x)*auto_speed_gain;
      float accel_y = (speed_sp_y - stateGetSpeedNed_f()->y)*auto_speed_gain;

      //   struct FloatMat33 Ga;
      indi_calcG(&Ga);
      MAT33_INV(Ga_inv, Ga);

      float altitude_sp = POS_FLOAT_OF_BFP(guidance_v_z_sp);
      float vertical_velocity_sp = pv_gain*(altitude_sp - stateGetPositionNed_f()->z);
  //     float vertical_velocity_rc_euler = -(stabilization_cmd[COMMAND_THRUST]-4500.0)/4500.0*2.0;
      float vertical_velocity_err_euler = vertical_velocity_sp - stateGetSpeedNed_f()->z;
      float accel_ref_euler = vertical_velocity_err_euler*vv_gain;

      struct FloatVect3 a_diff = { accel_x - filt_accelxn, accel_y -filt_accelyn, accel_ref_euler - filt_accelzn };

      Bound(a_diff.x, -6.0, 6.0);
      Bound(a_diff.y, -6.0, 6.0);
      Bound(a_diff.z, -9.0, 9.0);

      MAT33_VECT3_MUL(inputs, Ga_inv, a_diff);

      indicontrol.phi = roll_filt + inputs.x;
      indicontrol.theta = pitch_filt + inputs.y;
      indicontrol.psi = 0;//stateGetNedToBodyEulers_f()->psi;

      //Bound euler angles to prevent flipping and keep upright
      Bound(indicontrol.phi, -0.7, 0.7);
      Bound(indicontrol.theta, -0.7, 0.7);

      float_quat_of_eulers(q, &indicontrol);
    }
    else {
      pos_x_err = POS_FLOAT_OF_BFP(guidance_h_pos_sp.x) - stateGetPositionNed_f()->x +-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*2.0;
      pos_y_err = POS_FLOAT_OF_BFP(guidance_h_pos_sp.y) - stateGetPositionNed_f()->y + ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*2.0;

      static float sum_errx, sum_erry;
      if(in_flight) {
        sum_errx += pos_x_err;
        sum_erry += pos_y_err;
      }
      else {
        sum_errx = 0;
        sum_erry = 0;
      }

      float speed_sp_x = pos_x_err*pos_gain + sum_errx/512*pos_gain*sum_gain;
      float speed_sp_y = pos_y_err*pos_gain + sum_erry/512*pos_gain*sum_gain;
  //     float accel_x = (-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*8.0 - stateGetSpeedNed_f()->x)*auto_speed_gain;
  //     float accel_y = (ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*8.0 - stateGetSpeedNed_f()->y)*auto_speed_gain;
      float accel_x = (speed_sp_x - stateGetSpeedNed_f()->x)*auto_speed_gain;
      float accel_y = (speed_sp_y - stateGetSpeedNed_f()->y)*auto_speed_gain;

      float altitude_sp = POS_FLOAT_OF_BFP(guidance_v_z_sp);
      float altitude_gain = 0.5;
      float vertical_velocity_sp = altitude_gain*(altitude_sp - stateGetPositionNed_f()->z);
  //     float vertical_velocity_rc_euler = -(stabilization_cmd[COMMAND_THRUST]-4500.0)/4500.0*2.0;
      float vertical_velocity_err_euler = vertical_velocity_sp - stateGetSpeedNed_f()->z;
      float accel_ref_euler = vertical_velocity_err_euler*6.0;

      float cospsi = cosf(stateGetNedToBodyEulers_f()->psi);
      float sinpsi = sinf(stateGetNedToBodyEulers_f()->psi);

      indicontrol.phi = (cospsi*accel_y -sinpsi*accel_x)/10.0;
      indicontrol.theta = (cospsi*-accel_x - sinpsi*accel_y)/10.0;
      indicontrol.psi = 0;//stateGetNedToBodyEulers_f()->psi;

      //Bound euler angles to prevent flipping and keep upright
      Bound(indicontrol.phi, -0.3, 0.3);
      Bound(indicontrol.theta, -0.3, 0.3);

      float_quat_of_eulers(q, &indicontrol);
    }
  }
}

/** Read roll/pitch command from RC as quaternion.
 * Both angles are are interpreted relative to to the horizontal plane (earth bound).
 * @param[out] q quaternion representing the RC roll/pitch input
 */
void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat *q)
{
  /* only non-zero entries for roll quaternion */
  float roll2 = get_rc_roll_f() / 2.0f;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);

  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = (ANGLE_FLOAT_OF_BFP(transition_theta_offset) + get_rc_pitch_f()) / 2.0f;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of float_quat_comp(q, &q_roll, &q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}

void indi_calcG(struct FloatMat33 *Gmat) {

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
//   float T = -9.81;
  float T = (filt_accelzn-9.81)/(cphi*ctheta); //calculate specific force in body z axis by using the accelerometer

  RMAT_ELMT(*Gmat, 0, 0) = (cphi*spsi - sphi*cpsi*stheta)*T;
  RMAT_ELMT(*Gmat, 1, 0) = (-sphi*spsi*stheta - cpsi*cphi)*T;
  RMAT_ELMT(*Gmat, 2, 0) = -ctheta*sphi*T;
  RMAT_ELMT(*Gmat, 0, 1) = (cphi*cpsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 1, 1) = (cphi*spsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 2, 1) = -stheta*cphi*T;
  RMAT_ELMT(*Gmat, 0, 2) = sphi*spsi + cphi*cpsi*stheta;
  RMAT_ELMT(*Gmat, 1, 2) = cphi*spsi*stheta - cpsi*sphi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

void indi_filter_accel_ned(void)
{
  filt_accelyn = filt_accelyn + filt_accelydn / 512.0;
  filt_accelxn = filt_accelxn + filt_accelxdn / 512.0;
  filt_accelzn = filt_accelzn + filt_accelzdn / 512.0;

  filt_accelydn = filt_accelydn + filt_accelyddn / 512.0;
  filt_accelxdn = filt_accelxdn + filt_accelxddn / 512.0;
  filt_accelzdn = filt_accelzdn + filt_accelzddn / 512.0;

  filt_accelyddn = -filt_accelydn * 2 * 0.55 * 50.0 + (stateGetAccelNed_f()->y - filt_accelyn) * 50.0*50.0;
  filt_accelxddn = -filt_accelxdn * 2 * 0.55 * 50.0 + (stateGetAccelNed_f()->x - filt_accelxn) * 50.0*50.0;
  filt_accelzddn = -filt_accelzdn * 2 * 0.55 * 50.0 + (stateGetAccelNed_f()->z - filt_accelzn) * 50.0*50.0;
}

void indi_filter_attitude(void)
{
  roll_filt = roll_filt + roll_filtd / 512.0;
  filt_accely = filt_accely + filt_accelyd / 512.0;
  pitch_filt = pitch_filt + pitch_filtd / 512.0;
  filt_accelx = filt_accelx + filt_accelxd / 512.0;

  roll_filtd = roll_filtd + roll_filtdd / 512.0;
  filt_accelyd = filt_accelyd + filt_accelydd / 512.0;
  pitch_filtd = pitch_filtd + pitch_filtdd / 512.0;
  filt_accelxd = filt_accelxd + filt_accelxdd / 512.0;

  float cospsi = cosf(stateGetNedToBodyEulers_f()->psi);
  float sinpsi = sinf(stateGetNedToBodyEulers_f()->psi);
  float accelx = cospsi*stateGetAccelNed_f()->x + sinpsi*stateGetAccelNed_f()->y;
  float accely =-sinpsi*stateGetAccelNed_f()->x + cospsi*stateGetAccelNed_f()->y;

//   roll_filtdd = -roll_filtd * 2 * 0.9 * 20.0 + (stateGetNedToBodyEulers_f()->phi - roll_filt) * 400.0;
//   filt_accelydd = -filt_accelyd * 2 * 0.9 * 20.0 + (accely - filt_accely) * 400.0;
//   pitch_filtdd = -pitch_filtd * 2 * 0.9 * 20.0 + (stateGetNedToBodyEulers_f()->theta - pitch_filt) * 400.0;
//   filt_accelxdd = -filt_accelxd * 2 * 0.9 * 20.0 + (accelx - filt_accelx) * 400.0;

  roll_filtdd = -roll_filtd * 2 * 0.55 * 50.0 + (stateGetNedToBodyEulers_f()->phi - roll_filt) * 50.0*50.0;
  filt_accelydd = -filt_accelyd * 2 * 0.55 * 50.0 + (accely - filt_accely) * 50.0*50.0;
  pitch_filtdd = -pitch_filtd * 2 * 0.55 * 50.0 + (stateGetNedToBodyEulers_f()->theta - pitch_filt) * 50.0*50.0;
  filt_accelxdd = -filt_accelxd * 2 * 0.55 * 50.0 + (accelx - filt_accelx) * 50.0*50.0;
}

/** Read attitude setpoint from RC as quaternion
 * Interprets the stick positions as axes.
 * @param[in]  coordinated_turn  true if in horizontal mode forward
 * @param[in]  in_carefree       true if in carefree mode
 * @param[in]  in_flight         true if in flight
 * @param[out] q_sp              attitude setpoint as quaternion
 */
void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat *q_sp, bool_t in_flight, bool_t in_carefree,
    bool_t coordinated_turn)
{

  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
#if defined STABILIZATION_ATTITUDE_TYPE_INT
  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#endif
}

void stabilization_attitude_calc_setpoint(struct FloatQuat *q_sp, bool_t in_flight, bool_t in_carefree) {
  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_quat_f(&q_rp_cmd, in_flight);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  //Care Free mode
  if (in_carefree) {
    //care_free_heading has been set to current psi when entering care free mode.
    float_quat_of_axis_angle(&q_yaw, &zaxis, care_free_heading);
  } else {
    float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);
  }

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
  float_quat_normalize(&q_rp_sp);

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
#if defined STABILIZATION_ATTITUDE_TYPE_INT
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
#else
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, stab_att_sp_euler.psi);
#endif

    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    float_quat_comp_norm_shortest(q_sp, &q_rp_sp, &q_yaw_diff);
  } else {
    QUAT_COPY(*q_sp, q_rp_sp);
  }
}

//Function that reads the rc setpoint in an earth bound frame
void stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(struct FloatQuat *q_sp, bool_t in_flight,
    bool_t in_carefree, bool_t coordinated_turn)
{
  // FIXME: remove me, do in quaternion directly
  // is currently still needed, since the yaw setpoint integration is done in eulers
#if defined STABILIZATION_ATTITUDE_TYPE_INT
  stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight, in_carefree, coordinated_turn);
#endif

  const struct FloatVect3 zaxis = {0., 0., 1.};

  struct FloatQuat q_rp_cmd;
  stabilization_attitude_read_rc_roll_pitch_earth_quat_f(&q_rp_cmd);

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;

#if defined STABILIZATION_ATTITUDE_TYPE_INT
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.psi));
#else
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, stab_att_sp_euler.psi);
#endif

    float_quat_comp(q_sp, &q_yaw_sp, &q_rp_cmd);
  } else {
    struct FloatQuat q_yaw;
    float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

    /* roll/pitch commands applied to to current heading */
    struct FloatQuat q_rp_sp;
    float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
    float_quat_normalize(&q_rp_sp);

    QUAT_COPY(*q_sp, q_rp_sp);
  }
}
