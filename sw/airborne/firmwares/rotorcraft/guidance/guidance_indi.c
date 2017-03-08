/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to IROS2016 to learn more!
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stdio.h"
#include "filters/low_pass_filter.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifdef GUIDANCE_INDI_POS_GAIN
float guidance_indi_pos_gain = GUIDANCE_INDI_POS_GAIN;
#else
float guidance_indi_pos_gain = 0.5;
#endif

#ifdef GUIDANCE_INDI_SPEED_GAIN
float guidance_indi_speed_gain = GUIDANCE_INDI_SPEED_GAIN;
#else
float guidance_indi_speed_gain = 1.8;
#endif

struct FloatVect3 sp_accel = {0.0,0.0,0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float thrust_in_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS
#ifndef STABILIZATION_INDI_ACT_DYN_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS STABILIZATION_INDI_ACT_DYN_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

float wiggle_magnitude = 0.00;
float inv_eff[4];

enum transition transition_state = HOVER;
enum transition_ctrl transition_control = TO_FORWARD;
struct FloatVect2 sp_accel_tr;

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
Butterworth2LowPass pitchtr_filt;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

float vspeed_sp_setting = 0.0;
float transition_accel = 2.0;

bool perform_transition = false;
bool transition_back = false;

static void guidance_indi_propagate_filters(void);
static void guidance_indi_calcG(struct FloatMat33 *Gmat);
static void transition_quat_from_angles(struct FloatQuat *q, struct FloatEulers *angles);

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void) {
  thrust_in = 0.0;
  thrust_act = 0;

  float tau = 1.0/(2.0*M_PI*filter_cutoff);
  float sample_time = 1.0/PERIODIC_FREQUENCY;
  for(int8_t i=0; i<3; i++) {
    init_butterworth_2_low_pass(&filt_accel_ned[i], tau, sample_time, 0.0);
  }
  init_butterworth_2_low_pass(&roll_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitchtr_filt, tau, sample_time, 0.0);

  perform_transition = false;
  transition_control = TO_FORWARD;
  FLOAT_VECT2_ZERO(sp_accel_tr);
}

/**
 * @param in_flight in flight boolean
 * @param heading the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(bool in_flight, int32_t heading) {
  float heading_f = ANGLE_FLOAT_OF_BFP(heading);

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters();

  //Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x) - stateGetPositionNed_f()->x;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y) - stateGetPositionNed_f()->y;
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  float speed_sp_x = pos_x_err * guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err * guidance_indi_pos_gain;
  float speed_sp_z = pos_z_err * guidance_indi_pos_gain;

  sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
  sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
  sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;

  if(perform_transition) {
    /*Calculate pitch angle that can go beyond 90 degrees*/
    float transition_pitch = get_transition_pitch();

    /*Calculate the transition percentage so that the ctrl_effecitveness scheduling works*/
    transition_percentage = BFP_OF_REAL(transition_pitch/RadOfDeg(-78.0)*100,INT32_PERCENTAGE_FRAC);
    Bound(transition_percentage,0,BFP_OF_REAL(100,INT32_PERCENTAGE_FRAC));

    /*The desired acceleration in 2D, depending on the transition state*/
    switch (transition_control) {
      case TO_FORWARD:
        sp_accel_tr.x = transition_accel;
        if(transition_pitch < RadOfDeg(-70.0)) {
          transition_control++;
        }
        break;
      case IN_FORWARD:
        sp_accel_tr.x = 0.0;
        if(transition_back) {
          transition_control++;
        }
        break;
      case TO_HOVER:
        sp_accel_tr.x = -transition_accel;
        if(transition_pitch > RadOfDeg(-10.0)) {
          perform_transition = false;
          transition_control = TO_FORWARD;
        }
        break;
      default:
        sp_accel_tr.x = 0.0;
        break;
    }
    sp_accel_tr.y = sp_accel.z; //Always keep controlling the altitude (y = z axis)

    // Calculate the inverse of the control effectiveness
    calc_inv_transition_effectiveness(transition_pitch, inv_eff);

    // Obtain the acceleration in a 2D 'navigation' plane
    struct FloatVect2 accel_tr;
    get_two_d_accel(&accel_tr, heading_f);

    // Calculate the acceleration error
    struct FloatVect2 a_diff_tr = {sp_accel_tr.x - accel_tr.x, sp_accel_tr.y - accel_tr.y};

    // Multiply with the inverse control effectiveness
    struct FloatVect2 cmds;
    float_mat2_mult(&cmds,inv_eff,a_diff_tr); // cmds = delta 1)pitch 2)thrust

    if(transition_control == IN_FORWARD) {
      stabilization_cmd[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE];
    } else {
      // Send the thrust increment to the inner loop
      AbiSendMsgTHRUST(THRUST_INCREMENT_ID, cmds.y);
    }

    // Incrment the pitch angle, using the special pitch representation
    guidance_euler_cmd.theta = pitchtr_filt.o[0] + cmds.x;

    // Set the roll and yaw angles. Yaw will be updated from the read rc coordinated turn function
    int32_t roll_rc = radio_control.values[RADIO_ROLL];
    float roll_f = roll_rc * STABILIZATION_ATTITUDE_SP_MAX_PHI / MAX_PPRZ;
    guidance_euler_cmd.phi = roll_f;
    guidance_euler_cmd.psi = heading_f;

    //Bound euler angles to prevent flipping
    Bound(guidance_euler_cmd.theta, RadOfDeg(-110), 0.0);

    // Set the quaternion setpoint, by subsequently rotating around 'navigation' axes
    struct FloatQuat sp_quat;
    transition_quat_from_angles(&sp_quat, &guidance_euler_cmd);
    float_quat_normalize(&sp_quat);
    QUAT_BFP_OF_REAL(stab_att_sp_quat,sp_quat);

  } else {
#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
    //for rc control horizontal, rotate from body axes to NED
    float psi = stateGetNedToBodyEulers_f()->psi;
    float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
    float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
    sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
    sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

#if CYCLONE_DESCEND_TEST
    speed_sp_z = vspeed_sp_setting;
    sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;
#else
    //for rc vertical control
    sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif
#endif

    //Calculate matrix of partial derivatives
    guidance_indi_calcG(&Ga);
    //Invert this matrix
    MAT33_INV(Ga_inv, Ga);

    struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y -filt_accel_ned[1].o[0], sp_accel.z -filt_accel_ned[2].o[0]};

    //Bound the acceleration error so that the linearization still holds
    Bound(a_diff.x, -6.0, 6.0);
    Bound(a_diff.y, -6.0, 6.0);
    Bound(a_diff.z, -9.0, 9.0);

    //If the thrust to specific force ratio has been defined, include vertical control
    //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
    a_diff.z = 0.0;
#endif
#endif

    //Calculate roll,pitch and thrust command
    MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

    AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);

    guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
    guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;
    //zero psi command, because a roll/pitch quat will be constructed later
    guidance_euler_cmd.psi = 0;

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
    guidance_indi_filter_thrust();

    //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
    thrust_in = thrust_filt.o[0] + euler_cmd.z*thrust_in_specific_force_gain;
    Bound(thrust_in, 0, 9600);

#if GUIDANCE_INDI_RC_DEBUG
    if(radio_control.values[RADIO_THROTTLE]<300) {
      thrust_in = 0;
    }
#endif

    //Overwrite the thrust command from guidance_v
    stabilization_cmd[COMMAND_THRUST] = thrust_in;
#endif

    //Bound euler angles to prevent flipping
    Bound(guidance_euler_cmd.phi, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);
    Bound(guidance_euler_cmd.theta, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);

    // Start the wiggle!
    static int32_t wiggle_counter = 0;
    if(wiggle_counter >= 1024) {
      wiggle_counter = 0;
    } else if(wiggle_counter < 512) {
      heading += ANGLE_BFP_OF_REAL(wiggle_magnitude);
    }
    wiggle_counter ++;

    //set the quat setpoint with the calculated roll and pitch
    stabilization_attitude_set_setpoint_rp_quat_f(&guidance_euler_cmd, in_flight, heading);
  }
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);

  // same filter as for the acceleration
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
}
#endif

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 */
void guidance_indi_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, stateGetNedToBodyEulers_f()->phi);
  update_butterworth_2_low_pass(&pitch_filt, stateGetNedToBodyEulers_f()->theta);

  update_butterworth_2_low_pass(&pitchtr_filt, get_transition_pitch());
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations
 */
void guidance_indi_calcG(struct FloatMat33 *Gmat) {

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81;

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

/**
 * @param indi_rp_cmd roll/pitch command from indi guidance [rad] (float)
 * @param in_flight in flight boolean
 * @param heading the desired heading [rad] in BFP with INT32_ANGLE_FRAC
 *
 * function that creates a quaternion from a roll, pitch and yaw setpoint
 */
void stabilization_attitude_set_setpoint_rp_quat_f(struct FloatEulers* indi_rp_cmd, bool in_flight, int32_t heading)
{
  struct FloatQuat q_rp_cmd;
  //this is a quaternion without yaw! add the desired yaw before you use it!
  float_quat_of_eulers(&q_rp_cmd, indi_rp_cmd);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
  float_quat_normalize(&q_rp_sp);

  struct FloatQuat q_sp;

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, ANGLE_FLOAT_OF_BFP(heading));


    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    float_quat_comp_norm_shortest(&q_sp, &q_rp_sp, &q_yaw_diff);
  } else {
    QUAT_COPY(q_sp, q_rp_sp);
  }

  QUAT_BFP_OF_REAL(stab_att_sp_quat,q_sp);
}

float get_transition_pitch(void) {
  float fwd_theta = get_fwd_pitch()-RadOfDeg(90.0);
  float hover_theta = stateGetNedToBodyEulers_f()->theta;

  if(transition_state == HOVER) {
    if(hover_theta > RadOfDeg(-45.0)) {
      return hover_theta;
    } else {
      transition_state = FORWARD;
      return fwd_theta;
    }
  } else {
    if(fwd_theta < RadOfDeg(-45.0)) {
      return fwd_theta;
    } else {
      transition_state = HOVER;
      return hover_theta;
    }
  }
}

float get_fwd_pitch(void) {
  struct FloatQuat ninety_quat = {0.7071, 0, 0.7071, 0};

  struct FloatQuat rotated_quat;
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  float_quat_comp(&rotated_quat, quat, &ninety_quat);
  float_quat_normalize(&rotated_quat);
  struct FloatEulers eulers;
  float_eulers_of_quat(&eulers, &rotated_quat);

  return eulers.theta;
}

void get_two_d_accel(struct FloatVect2 *accel, float heading) {
  accel->x = filt_accel_ned[0].o[0] * cosf(heading) + filt_accel_ned[1].o[0] * sinf(heading);
  accel->y = filt_accel_ned[2].o[0];
}

void calc_inv_transition_effectiveness(float theta, float inv_eff[4]) {
  float T = 9.81;

  float eff[4];
  eff[0] = -cosf(theta)*T;
  eff[1] =  sinf(theta);
  eff[2] =  sinf(theta)*T - 42.0*theta/RadOfDeg(-90.0);
  eff[3] =  cosf(theta);

  float_mat_inv_2d(inv_eff, eff);
}

/** First pitch, then roll, the rotate to the correct heading.
 * The roll angle is interpreted relative to to the horizontal plane (earth bound).
 * @param[in] roll roll angle
 * @param[in] pitch pitch angle
 * @param[in] yaw yaw angle
 * @param[out] q quaternion representing the RC roll/pitch input
 */
void transition_quat_from_angles(struct FloatQuat *q, struct FloatEulers *angles)
{
  /* only non-zero entries for roll quaternion */
  float roll2 = angles->phi / 2.0f;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);

  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = angles->theta / 2.0f;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);

  /* only multiply non-zero entries of float_quat_comp(q, &q_roll, &q_pitch) */
  struct FloatQuat q_rp;
  q_rp.qi = qi_roll * qi_pitch;
  q_rp.qx = qx_roll * qi_pitch;
  q_rp.qy = qi_roll * qy_pitch;
  q_rp.qz = qx_roll * qy_pitch;

  struct FloatQuat q_yaw_sp;
  const struct FloatVect3 zaxis = {0., 0., 1.};
  float_quat_of_axis_angle(&q_yaw_sp, &zaxis, angles->psi);
  float_quat_comp(q, &q_yaw_sp, &q_rp);
}

