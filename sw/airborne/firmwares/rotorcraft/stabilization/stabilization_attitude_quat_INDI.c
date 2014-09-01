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

/** @file stabilization_attitude_quat_int.c
 * Rotorcraft quaternion attitude stabilization INDI control
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "subsystems/actuators/motor_mixing.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/radio_control.h"

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
};

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_PHI_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_THETA_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_PGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_DGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_IGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_IGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_IGAIN  < 0)
#warning "ALL control gains are now positive!!!"
#endif

void stabililzation_attitude_change_motor_mixing(int32_t (*coef)[MOTOR_MIXING_NB_MOTOR], int32_t (*orig_coef)[MOTOR_MIXING_NB_MOTOR], int32_t denominator);

struct Int32Quat stabilization_att_sum_err_quat;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

struct FloatRates filtered_rate = {0., 0., 0.};
struct FloatRates filtered_rate_deriv = {0., 0., 0.};
struct FloatRates filtered_rate_2deriv = {0., 0., 0.};
struct FloatRates angular_accel_ref = {0., 0., 0.};
struct FloatRates indi_u = {0., 0., 0.};
struct FloatRates indi_du = {0., 0., 0.};
float kp_p = 95;
float kd_p = 10;
float m_c_p = 40;
float kp_q = 95;
float kd_q = 9;
float m_c_q = 9;
float kp_r = 20;
float kd_r = 5;
float m_c_r = 170;
float att_err_x = 0;
struct FloatRates u_act_dyn = {0., 0., 0.};
struct FloatRates u_in = {0., 0., 0.};
struct FloatRates udot = {0., 0., 0.};
struct FloatRates udotdot = {0., 0., 0.};
struct FloatRates filt_rate = {0., 0., 0.};

float m_c_p_library[5] = {40,40,40,40,40};
float m_c_q_library[5] = {9,9,9,9,9};
float m_c_r_library[5] = {170,170,130,100,80};

int32_t pitch_coef_orig[MOTOR_MIXING_NB_MOTOR] = MOTOR_MIXING_PITCH_COEF;
int32_t yaw_coef_orig[MOTOR_MIXING_NB_MOTOR] = MOTOR_MIXING_YAW_COEF;

int32_t elevator_gain = 0;
int32_t aileron_gain = 1;
int32_t elevator_gain_goal = 0;
int32_t aileron_gain_goal = 1;

float filtered_rate_deriv_prev = 0;
float u_prev = 0;
float drate = 0;
float filt_du = 0;
float d_eff = 0;
float invact_eff = 1.0/15.0;

#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48
#define GAIN_PRESCALER_I 48

#define ZETA 0.37
#define OMEGA 50
#define OMEGA2 2500

#define ZETA_R 0.5
#define OMEGA_R 20
#define OMEGA2_R 400

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(void) { //FIXME really use this message here ?
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();
  DOWNLINK_SEND_STAB_ATTITUDE_INT(DefaultChannel, DefaultDevice,
      &yaw_coef[0], &pitch_coef[1], &pitch_coef[2],
      &pitch_coef[3], &(att->theta), &(att->psi),
      &stab_att_sp_euler.phi,
      &stab_att_sp_euler.theta,
      &stab_att_sp_euler.psi,
      &att_err_x,
      &stabilization_att_sum_err_quat.qx,
      &stabilization_att_sum_err_quat.qy,
      &yaw_coef[0],
      &yaw_coef[1],
      &yaw_coef[2],
      &yaw_coef[3],
      &stabilization_att_ff_cmd[COMMAND_PITCH],
      &stabilization_att_ff_cmd[COMMAND_YAW],
      &stabilization_cmd[COMMAND_ROLL],
      &stabilization_cmd[COMMAND_PITCH],
      &stabilization_cmd[COMMAND_YAW]);
}

static void send_att_ref(void) {
  DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(DefaultChannel, DefaultDevice,
                                      &stab_att_sp_euler.phi,
                                      &stab_att_sp_euler.theta,
                                      &stab_att_sp_euler.psi,
                                      &stab_att_ref_euler.phi,
                                      &stab_att_ref_euler.theta,
                                      &stab_att_ref_euler.psi,
                                      &stab_att_ref_rate.p,
                                      &stab_att_ref_rate.q,
                                      &stab_att_ref_rate.r,
                                      &stab_att_ref_accel.p,
                                      &stab_att_ref_accel.q,
                                      &stab_att_ref_accel.r);
}

static void send_ahrs_ref_quat(void) {
  struct Int32Quat* quat = stateGetNedToBodyQuat_i();
  DOWNLINK_SEND_AHRS_REF_QUAT(DefaultChannel, DefaultDevice,
      &stab_att_ref_quat.qi,
      &stab_att_ref_quat.qx,
      &stab_att_ref_quat.qy,
      &stab_att_ref_quat.qz,
      &(quat->qi),
      &(quat->qx),
      &(quat->qy),
      &(quat->qz));
}

static void send_att_indi(void) {
  DOWNLINK_SEND_STAB_ATTITUDE_INDI(DefaultChannel, DefaultDevice,
                                   &filtered_rate.r,
                                   &filtered_rate_deriv.r,
                                   &filtered_rate_2deriv.r,
                                   &u_in.p,
                                   &u_in.q,
                                   &u_in.r);
}
#endif

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_INDI", send_att_indi);
#endif
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  INT32_QUAT_ZERO(stabilization_att_sum_err_quat);

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

  yaw_coef[0]  = yaw_coef_orig[0];
  yaw_coef[1]  = yaw_coef_orig[1];
  yaw_coef[2]  = yaw_coef_orig[2];
  yaw_coef[3]  = yaw_coef_orig[3];
  
  pitch_coef[0]  = pitch_coef_orig[0];
  pitch_coef[1]  = pitch_coef_orig[1];
  pitch_coef[2]  = pitch_coef_orig[2];
  pitch_coef[3]  = pitch_coef_orig[3];
  elevator_gain = 0;
  aileron_gain = 3;
  m_c_r = 170;

}

void stabilization_attitude_set_failsafe_setpoint(void) {
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy) {
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading) {
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

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

#define BOUND_CONTROLS(_v, _min, _max) {         \
_v = _v < _min ? _min : _v > _max ? _max : _v;  \
}

static void attitude_run_fb(int32_t fb_commands[], struct Int32AttitudeGains *gains, struct Int32Quat *att_err,
    struct Int32Rates *rate_err, struct Int32Quat *sum_err)
{
  angular_accel_ref.p = kp_p * QUAT1_FLOAT_OF_BFP(att_err->qx) - kd_p * filtered_rate.p;
  angular_accel_ref.q = kp_q * QUAT1_FLOAT_OF_BFP(att_err->qy) - kd_q * filtered_rate.q;
  angular_accel_ref.r = kp_r * QUAT1_FLOAT_OF_BFP(att_err->qz) - kd_r * filtered_rate.r;

  indi_du.p = m_c_p * (angular_accel_ref.p - filtered_rate_deriv.p);
  indi_du.q = m_c_q * (angular_accel_ref.q - filtered_rate_deriv.q);
  indi_du.r = m_c_r * (angular_accel_ref.r - filtered_rate_deriv.r);

  u_in.p = indi_u.p + indi_du.p;
  u_in.q = indi_u.q + indi_du.q;
  u_in.r = indi_u.r + indi_du.r;

  BOUND_CONTROLS(u_in.p, -4500, 4500);
  BOUND_CONTROLS(u_in.q, -4500, 4500);
  float half_thrust = ((float) stabilization_cmd[COMMAND_THRUST]/2);

  //Save error for displaying purposes
  att_err_x = QUAT1_FLOAT_OF_BFP(att_err->qx);

  if(radio_control.values[5] > 0) {
    /*  INDI feedback */
    fb_commands[COMMAND_ROLL] = u_in.p;
    fb_commands[COMMAND_PITCH] = u_in.q;
    fb_commands[COMMAND_YAW] = u_in.r;
    if( (norm_ref_airspeed > (13<<8)) || (transition_percentage > (90 << INT32_PERCENTAGE_FRAC)) ) {
      //if in forward flight use ailerons for roll control and adjust gains
      stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);
      stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,512); //don't use motors for yaw (fixedwing roll)
      aileron_gain = 1;
      elevator_gain = 0;
      m_c_r = 30;
      BOUND_CONTROLS(u_in.r, -9600, 9600);
    }
    else if(norm_ref_airspeed > (10<<8)) {
      //if in forward flight use ailerons for roll control and adjust gains
      stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);
      stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,512); //don't use motors for yaw (fixedwing roll)
      aileron_gain = 1;
      elevator_gain = 0;
      m_c_r = 30;
      BOUND_CONTROLS(u_in.r, -9600, 9600);
    }
    else if(norm_ref_airspeed > (7<<8)) {
      //if in forward flight use ailerons for roll control and adjust gains
      stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);
      stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,512); //don't use motors for yaw (fixedwing roll)
      aileron_gain = 1;
      elevator_gain = 0;
      m_c_r = 30;
      BOUND_CONTROLS(u_in.r, -9600, 9600);
    }
    else if(norm_ref_airspeed > (4<<8)) {
      //if in forward flight use ailerons for roll control and adjust gains
      stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);
      stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,2); //don't use motors for yaw (fixedwing roll)
      aileron_gain = 1;
      elevator_gain = 0;
      m_c_r = 30;
      BOUND_CONTROLS(u_in.r, -half_thrust, half_thrust);
    }
    else {
      // if not in forward flight use hover settings
      stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,1);
      stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);

      elevator_gain = 0;
      aileron_gain = 3;
      m_c_r = 170;
      BOUND_CONTROLS(u_in.r, -half_thrust, half_thrust);
    }
  }
  else {
    //if using PID, set INDI to default values and run PID
    stabililzation_attitude_change_motor_mixing(&pitch_coef,&pitch_coef_orig,1);
    stabililzation_attitude_change_motor_mixing(&yaw_coef,&yaw_coef_orig,1);
    elevator_gain = 4;
    aileron_gain = 6;
    m_c_r = 170;
    /*  PID feedback */
    fb_commands[COMMAND_ROLL] =
        GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) / 4 +
        GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) / 16 +
        GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx) / 2;

    fb_commands[COMMAND_PITCH] =
        GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) / 4 +
        GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q)  / 16 +
        GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy) / 2;

    fb_commands[COMMAND_YAW] =
        GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) / 4 +
        GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r)  / 16 +
        GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz) / 2;

    FLOAT_RATES_ZERO(indi_u);
    FLOAT_RATES_ZERO(indi_du);
    FLOAT_RATES_ZERO(u_act_dyn);
    FLOAT_RATES_ZERO(u_in);
    FLOAT_RATES_ZERO(udot);
    FLOAT_RATES_ZERO(udotdot);
  }

  //Propagate input filters
  stabilization_indi_filter_inputs();

  //Don't increment if thrust is off, to avoid windup on ground
  if(stabilization_cmd[COMMAND_THRUST]<300) {
    FLOAT_RATES_ZERO(indi_u);
    FLOAT_RATES_ZERO(indi_du);
    FLOAT_RATES_ZERO(u_act_dyn);
    FLOAT_RATES_ZERO(u_in);
    FLOAT_RATES_ZERO(udot);
    FLOAT_RATES_ZERO(udotdot);
  }

//   stabilization_indi_adaptive_gains();
//   m_c_q = 1.0/invact_eff;
}

void stabililzation_attitude_change_motor_mixing(int32_t (*coef)[MOTOR_MIXING_NB_MOTOR], int32_t (*orig_coef)[MOTOR_MIXING_NB_MOTOR], int32_t denominator) {
  for(uint8_t i = 0; i<MOTOR_MIXING_NB_MOTOR; i++){
    (*coef)[i] = (*orig_coef)[i]/denominator;
  }
}

void stabilization_attitude_run(bool_t enable_integrator) {

  /*
   * Update reference
   */
  stabilization_attitude_ref_update();
  stabilization_indi_filter_gyro();

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat* att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_sp_quat);
  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /*  rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) };
  struct Int32Rates rate_err;
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

  if(radio_control.values[5] < 0) {
    /* integrated error */
    if (enable_integrator) {
      stabilization_att_sum_err_quat.qx += att_err.qx /IERROR_SCALE;
      stabilization_att_sum_err_quat.qy += att_err.qy /IERROR_SCALE;
      stabilization_att_sum_err_quat.qz += att_err.qz /IERROR_SCALE;
      Bound(stabilization_att_sum_err_quat.qx,-100000,100000);
      Bound(stabilization_att_sum_err_quat.qy,-100000,100000);
      Bound(stabilization_att_sum_err_quat.qz,-100000,100000);
    } else {
      /* reset accumulator */
      INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
    }
  }

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat);

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}

void stabilization_indi_filter_gyro(void) {
  filtered_rate.p = filtered_rate.p + filtered_rate_deriv.p/512.0;
  filtered_rate.q = filtered_rate.q + filtered_rate_deriv.q/512.0;
  filtered_rate.r = filtered_rate.r + filtered_rate_deriv.r/512.0;
  
  filtered_rate_deriv.p = filtered_rate_deriv.p + filtered_rate_2deriv.p/512.0;
  filtered_rate_deriv.q = filtered_rate_deriv.q + filtered_rate_2deriv.q/512.0;
  filtered_rate_deriv.r = filtered_rate_deriv.r + filtered_rate_2deriv.r/512.0;
  
  filtered_rate_2deriv.p = -filtered_rate_deriv.p * 2*ZETA*OMEGA + ( stateGetBodyRates_f()->p - filtered_rate.p)*OMEGA2;
  filtered_rate_2deriv.q = -filtered_rate_deriv.q * 2*ZETA*OMEGA + ( stateGetBodyRates_f()->q - filtered_rate.q)*OMEGA2;
  filtered_rate_2deriv.r = -filtered_rate_deriv.r * 2*ZETA_R*OMEGA_R + ( stateGetBodyRates_f()->r - filtered_rate.r)*OMEGA2_R;
}

void stabilization_indi_filter_inputs(void) {

  //actuator dynamics
  u_act_dyn.p = u_act_dyn.p + 0.03*( u_in.p - u_act_dyn.p);
  u_act_dyn.q = u_act_dyn.q + 0.03*( u_in.q - u_act_dyn.q);
  u_act_dyn.r = u_act_dyn.r + 0.03*( u_in.r - u_act_dyn.r);

  //Sensor dynamics (same filter as on gyro measurements)
  indi_u.p = indi_u.p + udot.p/512.0;
  indi_u.q = indi_u.q + udot.q/512.0;
  indi_u.r = indi_u.r + udot.r/512.0;
  
  udot.p = udot.p + udotdot.p/512.0;
  udot.q = udot.q + udotdot.q/512.0;
  udot.r = udot.r + udotdot.r/512.0;
  
  udotdot.p = -udot.p * 2*ZETA*OMEGA + (u_act_dyn.p - indi_u.p)*OMEGA2;
  udotdot.q = -udot.q * 2*ZETA*OMEGA + (u_act_dyn.q - indi_u.q)*OMEGA2;
  udotdot.r = -udot.r * 2*ZETA_R*OMEGA_R + (u_act_dyn.r - indi_u.r)*OMEGA2_R;
}

void stabilization_indi_adaptive_gains(void) {
  filt_du = indi_u.q - u_prev;
  drate = filtered_rate_deriv.q - filtered_rate_deriv_prev;

  u_prev = indi_u.q;
  filtered_rate_deriv_prev = filtered_rate_deriv.q;
  
  if( (filt_du != 0) && (drate != 0) && (fabs(filt_du) > 1)) {
    d_eff = 0.0001*((drate/filt_du) - invact_eff);
    if(d_eff > 0.001)
      d_eff = 0.001;
    else if(d_eff < -0.001)
      d_eff = -0.001;
    invact_eff = invact_eff + d_eff;
    }
  else
    invact_eff = invact_eff;
}