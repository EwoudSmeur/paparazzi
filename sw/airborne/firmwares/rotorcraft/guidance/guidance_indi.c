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
 * @file firmwares/rotorcraft/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to ICRA2016 to learn more!
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
#include "subsystems/datalink/downlink.h"

float guidance_indi_pos_gain = 0.7;
float guidance_indi_speed_gain = 1.5;
float guidance_indi_pos_gain_vertical = 0.5;
float guidance_indi_speed_gain_vertical = 3.0;
struct FloatVect3 sp_accel = {0.0,0.0,0.0};

struct FloatVect3 filt_accel_ned;
struct FloatVect3 filt_accel_ned_d;
struct FloatVect3 filt_accel_ned_dd;
float filt_accelzbody = 0;
float filt_accelzbodyd = 0;
float filt_accelzbodydd = 0;
float roll_filt = 0;
float roll_filtd = 0;
float roll_filtdd = 0;
float pitch_filt = 0;
float pitch_filtd = 0;
float pitch_filtdd = 0;
float T_in = 0;
float T_act = 0;
float T_filt = 0;
float T_filtd = 0;
float T_filtdd = 0;

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_omega = 50.0;
float filter_zeta = 0.55;

struct FloatEulers guidance_euler_cmd;

struct FloatVect3 calc_input_accel(struct FloatEulers *eulers);
struct FloatEulers calc_euler_cmd_nl(struct FloatVect3 input_accel);
void guidance_indi_filter_thrust(void);
float calcthrust(uint16_t *rpm);
void guidance_indi_accel_offset(bool_t in_flight);

void guidance_indi_enter(void) {
  filt_accelzbody = 0;
  filt_accelzbodyd = 0;
  filt_accelzbodydd = 0;
  roll_filt = 0;
  roll_filtd = 0;
  roll_filtdd = 0;
  pitch_filt = 0;
  pitch_filtd = 0;
  pitch_filtdd = 0;
  FLOAT_VECT3_ZERO(filt_accel_ned);
  FLOAT_VECT3_ZERO(filt_accel_ned_d);
  FLOAT_VECT3_ZERO(filt_accel_ned_dd);
  T_in = 0;
  T_act = 0;
  T_filt = 0;
  T_filtd = 0;
  T_filtdd = 0;
}

void guidance_indi_run(bool_t in_flight, int32_t heading) {

  guidance_indi_accel_offset(in_flight);

  //filter accel to get rid of noise
  //filter attitude to synchronize with accel
  guidance_indi_filter_attitude();
  guidance_indi_filter_accel();

  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.sp.pos.x) - stateGetPositionNed_f()->x; //+-ov.y/ (STABILIZATION_ATTITUDE_SP_MAX_THETA)*3.0;
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.sp.pos.y) - stateGetPositionNed_f()->y; //+ ov.x/ (STABILIZATION_ATTITUDE_SP_MAX_PHI)*3.0;

  float speed_sp_x = pos_x_err*guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err*guidance_indi_pos_gain;

  struct FloatEulers *state_eulers = stateGetNedToBodyEulers_f();

#if !OUTER_LOOP_INDI_USE_RC
  sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x)*guidance_indi_speed_gain;
  sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y)*guidance_indi_speed_gain;

  float altitude_sp = POS_FLOAT_OF_BFP(guidance_v_z_ref);
  float vertical_velocity_sp = guidance_indi_pos_gain_vertical*(altitude_sp - stateGetPositionNed_f()->z);
#else
  //rotate rc commands to ned axes
  float psi = state_eulers->psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  float vertical_velocity_sp = -(radio_control.values[RADIO_THROTTLE]-4500.0)/4500.0*2.0;
  //   sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500.0)/4500.0*2.0;
#endif

  float vertical_velocity_err = vertical_velocity_sp - stateGetSpeedNed_f()->z;
  sp_accel.z = vertical_velocity_err*guidance_indi_speed_gain_vertical;

  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned.x, sp_accel.y -filt_accel_ned.y, sp_accel.z -filt_accel_ned.z};

  guidance_indi_filter_thrust();

#if NONLINEAR_INDI
  struct FloatEulers filt_state_eulers;
  filt_state_eulers.phi = roll_filt;
  filt_state_eulers.theta = pitch_filt;
  filt_state_eulers.psi = state_eulers->psi; //TODO this should be filtered as well i guess?
  struct FloatVect3 input_accel0 = calc_input_accel(&filt_state_eulers);

  struct FloatVect3 input_accel;
  VECT3_SUM(input_accel, input_accel0, a_diff);

  struct FloatEulers output_euler = calc_euler_cmd_nl(input_accel);

  if(radio_control.values[RADIO_THROTTLE]<300) {
    T_in = 0;
  }

  guidance_euler_cmd.phi = output_euler.phi;
  guidance_euler_cmd.theta = output_euler.theta;
//   stabilization_cmd[COMMAND_THRUST] = T_in;

#warning "the command taken by inner loop is now euler_cmd. make sure NONLINEAR_INDI outputs an acceleration!"

//   RunOnceEvery(50, DOWNLINK_SEND_OUTER_INDI(DefaultChannel, DefaultDevice, &T_in, &Tm_meas, &input_accel.z));
#else
  //   struct FloatMat33 Ga;
  guidance_indi_calcG(&Ga);
  MAT33_INV(Ga_inv, Ga);

//   Bound(a_diff.x, -6.0, 6.0);
//   Bound(a_diff.y, -6.0, 6.0);
//   Bound(a_diff.z, -9.0, 9.0);

  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  T_in = T_filt -500.0*euler_cmd.z;
  Bound(T_in, 0.0, 9600.0);

  if(radio_control.values[RADIO_THROTTLE]<300) {
    T_in = 0;
  }

//   stabilization_cmd[COMMAND_THRUST] = T_in;

//   RunOnceEvery(50, DOWNLINK_SEND_OUTER_INDI(DefaultChannel, DefaultDevice, &T_in, &euler_cmd.z, &a_diff.z));

  guidance_euler_cmd.phi = roll_filt + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt + euler_cmd.y;
#endif

  guidance_euler_cmd.psi = 0;//stateGetNedToBodyEulers_f()->psi;

  //Bound euler angles to prevent flipping and keep upright
  Bound(guidance_euler_cmd.phi, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);
  Bound(guidance_euler_cmd.theta, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);

  stabilization_attitude_set_setpoint_rp_quat_f(in_flight, heading);
}

struct FloatEulers calc_euler_cmd_nl(struct FloatVect3 input_accel) {

  float Tm;
  if(input_accel.z > 0.0)
    Tm = -0.5;
  else
    Tm = -float_vect3_norm(&input_accel); //linear thrust/acceleration assumption

  uint16_t rpm_filt[4];
  rpm_filt[0] =(uint16_t) (u_actuators[0]/9000.0*9600.0+3000.0);
  rpm_filt[1] =(uint16_t) (u_actuators[1]/9000.0*9600.0+3000.0);
  rpm_filt[2] =(uint16_t) (u_actuators[2]/9000.0*9600.0+3000.0);
  rpm_filt[3] =(uint16_t) (u_actuators[3]/9000.0*9600.0+3000.0);
  euler_cmd.z = Tm-(-calcthrust(rpm_filt)/0.395);
//   T_in = Tm*-500.0;
//   Bound(T_in, 0.0, 9600.0);

  struct FloatEulers output;

  float psi = stateGetNedToBodyEulers_f()->psi;
  float spsi = sinf(psi);
  float cpsi = cosf(psi);

  float temp = (spsi*input_accel.x-cpsi*input_accel.y)/Tm;
  if(temp>1.0) {
    temp = 1.0;
  }
  else if(temp<-1.0) {
    temp = -1.0;
  }

  output.phi = asinf(temp);

  temp = (spsi*input_accel.y + cpsi*input_accel.x)/Tm/cosf(output.phi);
  if(temp>1.0) {
    temp = 1.0;
  }
  else if(temp<-1.0) {
    temp = -1.0;
  }

  output.theta = asinf(temp);
  output.psi = 0;

  return output;
}

struct FloatVect3 calc_input_accel(struct FloatEulers *eulers) {
//   float Tm = T_filt/-500.0; //linear thrust/acceleration assumption

  uint16_t rpm_filt[4];
  rpm_filt[0] =(uint16_t) (u_actuators[0]/9000.0*9600.0+3000.0);
  rpm_filt[1] =(uint16_t) (u_actuators[1]/9000.0*9600.0+3000.0);
  rpm_filt[2] =(uint16_t) (u_actuators[2]/9000.0*9600.0+3000.0);
  rpm_filt[3] =(uint16_t) (u_actuators[3]/9000.0*9600.0+3000.0);
  float Tm = -calcthrust(rpm_filt)/0.395;

  struct FloatVect3 accel_input0;
  accel_input0.x = (sinf(eulers->phi)*sinf(eulers->psi) + cosf(eulers->phi)*cosf(eulers->psi)*sinf(eulers->theta))*Tm;
  accel_input0.y = (cosf(eulers->phi)*sinf(eulers->psi)*sinf(eulers->theta) - cosf(eulers->psi)*sinf(eulers->phi))*Tm;
  accel_input0.z = cosf(eulers->phi)*cosf(eulers->theta)*Tm;

  return accel_input0;
}

//low pass the accelerometer measurements with a second order filter to remove noise from vibrations
void guidance_indi_filter_accel(void)
{
  VECT3_ADD_SCALED(filt_accel_ned, filt_accel_ned_d, 1.0/PERIODIC_FREQUENCY);
//   filt_accelzbody = filt_accelzbody + filt_accelzbodyd / PERIODIC_FREQUENCY; //also do body z accel

  VECT3_ADD_SCALED(filt_accel_ned_d, filt_accel_ned_dd, 1.0/PERIODIC_FREQUENCY);
//   filt_accelzbodyd = filt_accelzbodyd + filt_accelzbodydd / PERIODIC_FREQUENCY; //also do body z accel

  filt_accel_ned_dd.x = -filt_accel_ned_d.x * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->x - filt_accel_ned.x) * filter_omega*filter_omega;
  filt_accel_ned_dd.y = -filt_accel_ned_d.y * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->y - filt_accel_ned.y) * filter_omega*filter_omega;
  filt_accel_ned_dd.z = -filt_accel_ned_d.z * 2 * filter_zeta * filter_omega + (stateGetAccelNed_f()->z - filt_accel_ned.z) * filter_omega*filter_omega;
//   filt_accelzbodydd= -filt_accelzbodyd * 2 * filter_zeta * filter_omega + (accel_meas_body_f.z - filt_accelzbody) * filter_omega*filter_omega;
}

void guidance_indi_filter_attitude(void)
{
  roll_filt = roll_filt + roll_filtd / PERIODIC_FREQUENCY;
  pitch_filt = pitch_filt + pitch_filtd / PERIODIC_FREQUENCY;

  roll_filtd = roll_filtd + roll_filtdd / PERIODIC_FREQUENCY;
  pitch_filtd = pitch_filtd + pitch_filtdd / PERIODIC_FREQUENCY;
//   float cospsi = cosf(stateGetNedToBodyEulers_f()->psi);
//   float sinpsi = sinf(stateGetNedToBodyEulers_f()->psi);

  roll_filtdd = -roll_filtd * 2 * filter_zeta * filter_omega + (stateGetNedToBodyEulers_f()->phi - roll_filt) * filter_omega*filter_omega;
  pitch_filtdd = -pitch_filtd * 2 * filter_zeta * filter_omega + (stateGetNedToBodyEulers_f()->theta - pitch_filt) * filter_omega*filter_omega;
}

void guidance_indi_filter_thrust(void)
{
  T_act = T_act + STABILIZATION_INDI_ACT_DYN_P * (T_in - T_act);

  T_filt = T_filt + T_filtd / PERIODIC_FREQUENCY;
  T_filtd = T_filtd + T_filtdd / PERIODIC_FREQUENCY;
  T_filtdd = -T_filtd * 2 * filter_zeta * filter_omega + (T_act - T_filt) * filter_omega*filter_omega;
}

void guidance_indi_calcG(struct FloatMat33 *Gmat) {

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
  float Tm = -9.81; //minus gravity is a good guesstimate of the thrust force
//   float T = (filt_accelzn-9.81)/(cphi*ctheta); //calculate specific force in body z axis by using the accelerometer
//   float T = filt_accelzbody; //if body acceleration is available, use that!

  Tm = -calcthrust(actuators_bebop.rpm_obs)/0.395;
//   RunOnceEvery(50, DOWNLINK_SEND_OUTER_INDI(DefaultChannel, DefaultDevice, &T_in, &T, &T));

  RMAT_ELMT(*Gmat, 0, 0) = (cphi*spsi - sphi*cpsi*stheta)*Tm;
  RMAT_ELMT(*Gmat, 1, 0) = (-sphi*spsi*stheta - cpsi*cphi)*Tm;
  RMAT_ELMT(*Gmat, 2, 0) = -ctheta*sphi*Tm;
  RMAT_ELMT(*Gmat, 0, 1) = (cphi*cpsi*ctheta)*Tm;
  RMAT_ELMT(*Gmat, 1, 1) = (cphi*spsi*ctheta)*Tm;
  RMAT_ELMT(*Gmat, 2, 1) = -stheta*cphi*Tm;
  RMAT_ELMT(*Gmat, 0, 2) = sphi*spsi + cphi*cpsi*stheta;
  RMAT_ELMT(*Gmat, 1, 2) = cphi*spsi*stheta - cpsi*sphi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

void stabilization_attitude_set_setpoint_rp_quat_f(bool_t in_flight, int32_t heading)
{
  struct FloatQuat q_rp_cmd;
  float_quat_of_eulers(&q_rp_cmd, &guidance_euler_cmd); //TODO this is a quaternion without yaw! add the desired yaw before you use it!

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

float calcthrust(uint16_t *rpm){
  float thrust = 0;
  float rps = 0;
  for(int i=0; i<4; i++) {
    rps = rpm[i]/60.0;
    thrust = thrust + 0.08 -0.002283*rps + 7.088e-5 * rps * rps;
  }
  return thrust;
}


#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
static float filter2_omega = 0.25;
static float filter2_zeta = 0.55;
struct FloatVect3 speed_ned_prev = {0.0,0.0,0.0};
struct FloatVect3 accel_diff_body_filt = {0.0,0.0,0.0};
struct FloatVect3 accel_diff_body_filt_d = {0.0,0.0,0.0};
struct FloatVect3 accel_diff_body_filt_dd = {0.0,0.0,0.0};
struct FloatVect3 accel_body_gps;
//low pass the accelerometer measurements with a second order filter to remove noise from vibrations
void guidance_indi_accel_offset(bool_t in_flight)
{
  //Calculate the acceleration in the NED frame (no filtering in this stage)
  struct NedCoor_f *speed_ned = stateGetSpeedNed_f();
  struct FloatVect3 accel_ned;
  VECT3_DIFF(accel_ned, *speed_ned, speed_ned_prev);
  VECT3_COPY(speed_ned_prev, *speed_ned);
  VECT3_SMUL(accel_ned, accel_ned, 512.0);
  accel_ned.z -= 9.81; //add gravity to be compatible with the measurement of the accelerometer

  // Rotate the NED acceleration to the body axes
  struct FloatVect3 accel_gps_body;
  float_rmat_vmult(&accel_gps_body, stateGetNedToBodyRMat_f(), &accel_ned);

  struct Int32Vect3 accel_meas_body_i;
  struct FloatVect3 accel_meas_body_f;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&accel_meas_body_i, body_to_imu_rmat, &imu.accel);
  ACCELS_FLOAT_OF_BFP(accel_meas_body_f,accel_meas_body_i);

  //calculate acceleration difference in the body axes
  struct FloatVect3 accel_diff_body;
  VECT3_DIFF(accel_diff_body,accel_gps_body, accel_meas_body_f);

  if(in_flight) {
    VECT3_ADD_SCALED(accel_diff_body_filt, accel_diff_body_filt_d, 1.0/PERIODIC_FREQUENCY);

    VECT3_ADD_SCALED(accel_diff_body_filt_d, accel_diff_body_filt_dd, 1.0/PERIODIC_FREQUENCY);

    accel_diff_body_filt_dd.x = -accel_diff_body_filt_d.x * 2 * filter2_zeta * filter2_omega + (accel_diff_body.x - accel_diff_body_filt.x) * filter2_omega*filter2_omega;
    accel_diff_body_filt_dd.y = -accel_diff_body_filt_d.y * 2 * filter2_zeta * filter2_omega + (accel_diff_body.y - accel_diff_body_filt.y) * filter2_omega*filter2_omega;
    accel_diff_body_filt_dd.z = -accel_diff_body_filt_d.z * 2 * filter2_zeta * filter2_omega + (accel_diff_body.z - accel_diff_body_filt.z) * filter2_omega*filter2_omega;
  }
//   RunOnceEvery(50, DOWNLINK_SEND_OUTER_INDI(DefaultChannel, DefaultDevice, &accel_diff_body_filt.x, &filt_accel_ned.x, &accel_diff_body_filt.z));
}
