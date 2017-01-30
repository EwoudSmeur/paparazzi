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
#include "subsystems/datalink/telemetry.h"

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

static float calcthrust(uint16_t *rpm);
static struct FloatVect3 calc_euler_cmd_nl(struct FloatVect3 input_accel);
static struct FloatVect3 calc_input_accel(struct FloatEulers *eulers);

bool back_forth_maneuver = false;
uint32_t maneuver_counter = 0;

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

float thrust_act = 0;
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

static void guidance_indi_propagate_filters(void);
static void guidance_indi_calcG(struct FloatMat33 *Gmat);

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
}

/**
 * @param in_flight in flight boolean
 * @param heading the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(bool in_flight, int32_t heading) {

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

  static int32_t counter = 0;
  if(back_forth_maneuver) {
  sp_accel.x = 0.0;
  sp_accel.z = 0.0;

    if(counter < 256) {
      sp_accel.y = 4.0;
    } else if(counter < 512) {
      sp_accel.y = -4.0;
    } else {
      back_forth_maneuver = false;
    }
    counter += 1;
  } else {
    counter = 0;
  }


#if GUIDANCE_INDI_RC_DEBUG
  //for rc control horizontal, rotate from body axes to NED
  float psi = stateGetNedToBodyEulers_f()->psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  //for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif

  //Calculate matrix of partial derivatives
  guidance_indi_calcG(&Ga);
  //Invert this matrix
  MAT33_INV(Ga_inv, Ga);

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y -filt_accel_ned[1].o[0], sp_accel.z -filt_accel_ned[2].o[0]};

  //Bound the acceleration error so that the linearization still holds
  BoundAbs(a_diff.x, 9.0);
  BoundAbs(a_diff.y, 9.0);
  BoundAbs(a_diff.z, 9.0);

  //If the thrust to specific force ratio has been defined, include vertical control
  //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

#if NONLINEAR_INDI_GUIDANCE
  struct FloatEulers filt_state_eulers;
  filt_state_eulers.phi = roll_filt.o[0];
  filt_state_eulers.theta = pitch_filt.o[0];
  filt_state_eulers.psi = stateGetNedToBodyEulers_f()->psi; //TODO this should be filtered as well i guess?

  // Calculate the current acceleration due to inputs
  struct FloatVect3 input_accel0 = calc_input_accel(&filt_state_eulers);

  // Calculate the new required acceleration due to input with the acceleration increment
  struct FloatVect3 input_accel;
  VECT3_SUM(input_accel, input_accel0, a_diff);

  // Calculate the new input to achieve the acceleration input_accel
  struct FloatVect3 output_cmd = calc_euler_cmd_nl(input_accel);

  guidance_euler_cmd.phi = output_cmd.x;
  guidance_euler_cmd.theta = output_cmd.y;
  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, output_cmd.z);
#else
  //Calculate roll,pitch and thrust command
  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);

  guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;
#endif

  //zero psi command, because a roll/pitch quat will be constructed later
  guidance_euler_cmd.psi = 0;

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();

  //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
  thrust_in = thrust_filt.o[0] + euler_cmd.z*thrust_in_specific_force_gain;
  Bound(thrust_in, 0, 9600);

#ifdef GUIDANCE_INDI_RC_DEBUG
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

  //set the quat setpoint with the calculated roll and pitch
  stabilization_attitude_set_setpoint_rp_quat_f(&guidance_euler_cmd, in_flight, heading);
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
}

/**
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * wrt the NED accelerations
 *
 * @param Gmat array to write the matrix to [3x3]
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

  uint16_t rpm_filt[4];
  int8_t i = 0;
  for(i=0; i<4; i++) {
    float scaling = get_servo_max(i)-get_servo_min(i);
    rpm_filt[i] =(uint16_t) (actuator_lowpass_filters[i].o[0]*scaling/MAX_PPRZ+get_servo_min(i));
  }
  float T = -calcthrust(rpm_filt)/0.395;

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


/**
 * @brief calculates the inputs for an acceleration vector
 *
 * @param input_accel Required acceleration
 *
 * @return Input vector
 */
struct FloatVect3 calc_euler_cmd_nl(struct FloatVect3 input_accel) {

  struct FloatVect3 output;

  // The required thrust is the norm of the acceleration vector
  // Make sure that the thrust is not (close to) zero
  float Tm;
  float input_accel_norm = float_vect3_norm(&input_accel);
  if(input_accel_norm < 0.5)
    Tm = -0.5;
  else
    Tm = -input_accel_norm;

  // Calculate the required change in thrust
  uint16_t rpm_filt[4];
  int8_t i = 0;
  for(i=0; i<4; i++) {
    float scaling = get_servo_max(i)-get_servo_min(i);
    rpm_filt[i] =(uint16_t) (actuator_lowpass_filters[i].o[0]*scaling/MAX_PPRZ+get_servo_min(i));
  }
  output.z = Tm-(-calcthrust(rpm_filt)/0.395);

  // pre-calculate sin(psi) and cos(psi)
  float psi = stateGetNedToBodyEulers_f()->psi;
  float spsi = sinf(psi);
  float cpsi = cosf(psi);

  // Calculate required roll
  float temp = (spsi*input_accel.x-cpsi*input_accel.y)/Tm;
  BoundAbs(temp, 1.0);
  output.x = asinf(temp);

  // Calculate required pitch
  temp = (spsi*input_accel.y + cpsi*input_accel.x)/Tm/cosf(output.x);
  BoundAbs(temp, 1.0);
  output.y = asinf(temp);

  return output;
}

/**
 * @brief Calculates acceleration vector for these inputs
 *
 * Using this acceleration vector, we can calculate how much we should increment
 *
 * @param eulers The attitude
 *
 * @return The resultant acceleration
 */
struct FloatVect3 calc_input_accel(struct FloatEulers *eulers) {

  // First calculate the thrust magnitude
  uint16_t rpm_filt[4];
  int8_t i = 0;
  for(i=0; i<4; i++) {
    float scaling = get_servo_max(i)-get_servo_min(i);
    rpm_filt[i] =(uint16_t) (actuator_lowpass_filters[i].o[0]*scaling/MAX_PPRZ+get_servo_min(i));
  }
  float Tm = -calcthrust(rpm_filt)/0.395;

  // Calculate the thrust vector
  struct FloatVect3 accel_input0;
  accel_input0.x = (sinf(eulers->phi)*sinf(eulers->psi) + cosf(eulers->phi)*cosf(eulers->psi)*sinf(eulers->theta))*Tm;
  accel_input0.y = (cosf(eulers->phi)*sinf(eulers->psi)*sinf(eulers->theta) - cosf(eulers->psi)*sinf(eulers->phi))*Tm;
  accel_input0.z = cosf(eulers->phi)*cosf(eulers->theta)*Tm;

  return accel_input0;
}

/**
 * @brief Calculates the thrust based on rpm for bebop1
 * Using a second order fit
 *
 * @param rpm array with rpm of each rotor
 *
 * @return Calculated thrust (positive)
 */
float calcthrust(uint16_t *rpm){
  float thrust = 0;
  float rps = 0;
  for(int i=0; i<4; i++) {
    // go to rounds per second instead of rpm
    rps = rpm[i]/60.0;
    // Second order function of rps
    thrust = thrust + 0.03166 -0.001013*rps + 6.7705e-05 * rps * rps;
  }
  return thrust;
}
