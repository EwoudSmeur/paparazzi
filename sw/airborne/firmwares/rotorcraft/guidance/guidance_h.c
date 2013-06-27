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

/** @file firmwares/rotorcraft/guidance/guidance_h.c
 *  Horizontal guidance for rotorcrafts.
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"

#include "state.h"

#include "generated/airframe.h"

/* error if some gains are negative */
#if (GUIDANCE_H_PGAIN < 0) ||                   \
  (GUIDANCE_H_DGAIN < 0)   ||                   \
  (GUIDANCE_H_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#else
#if (GUIDANCE_H_AGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif
#endif

#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

#ifndef GUIDANCE_H_FORWARD_MAX_BANK
#define GUIDANCE_H_FORWARD_MAX_BANK RadOfDeg(30)
#endif

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)


uint8_t guidance_h_mode;
bool_t guidance_h_use_ref;

struct Int32Vect2 guidance_h_pos_sp;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;

struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_pos_err_sum;
struct Int32Vect2 guidance_h_nav_err;

struct Int32Eulers guidance_h_rc_sp;
struct Int32Vect2  guidance_h_command_earth;
struct Int32Vect2  guidance_h_stick_earth_sp;
struct Int32Eulers guidance_h_command_body;

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_again;

int32_t transition_percentage;
int32_t transition_percentage_goal;
int32_t transition_theta_offset;
int32_t carrot_heading;
int32_t heading_error;


static void guidance_h_update_reference(void);
static void guidance_h_traj_run(bool_t in_flight);
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void guidance_transitioned(bool_t in_flight);
static void guidance_roll_pitch_earth_quat_f(struct FloatQuat* q, int32_t pitch, int32_t roll);


void guidance_h_init(void) {

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  guidance_h_use_ref = GUIDANCE_H_USE_REF;

  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_pos_err_sum);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  INT_EULERS_ZERO(guidance_h_command_body);
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;
  transition_percentage = 0;
  transition_theta_offset = 0;
  transition_percentage_goal = 100;
  carrot_heading = 0;
  heading_error = 0;
}


static inline void reset_guidance_reference_from_current_position(void) {
  VECT2_COPY(guidance_h_pos_ref, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h_speed_ref, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h_accel_ref);
  gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

  INT_VECT2_ZERO(guidance_h_pos_err_sum);
}

void guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == guidance_h_mode)
    return;

  if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE) {
     transition_percentage = 0;
     transition_theta_offset = 0;
   }

  switch (new_mode) {
    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_enter();
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_enter();
      break;

    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_reset_care_free_heading();
    case GUIDANCE_H_MODE_FORWARD:
      transition_percentage_goal = 100;
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE) {
        stabilization_attitude_enter();
      }
      break;

    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE) {
        stabilization_attitude_enter();
      }
      break;

    default:
      break;
  }

  guidance_h_mode = new_mode;

}


void guidance_h_read_rc(bool_t  in_flight) {

  switch ( guidance_h_mode ) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_read_rc();
      break;

    case GUIDANCE_H_MODE_RATE:
#if SWITCH_STICKS_FOR_RATE_CONTROL
      stabilization_rate_read_rc_switched_sticks();
#else
      stabilization_rate_read_rc();
#endif
      break;

    case GUIDANCE_H_MODE_FORWARD:
    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_read_rc(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight);
      }
      else {
        INT_EULERS_ZERO(guidance_h_rc_sp);
      }
      break;
    default:
      break;
  }

}


void guidance_h_run(bool_t  in_flight) {
  switch ( guidance_h_mode ) {

    case GUIDANCE_H_MODE_RC_DIRECT:
      stabilization_none_run(in_flight);
      break;

    case GUIDANCE_H_MODE_RATE:
      stabilization_rate_run(in_flight);
      break;

    case GUIDANCE_H_MODE_FORWARD:
      if(transition_percentage < (transition_percentage_goal<<INT32_PERCENTAGE_FRAC)) {
        transition_run();
      }
    case GUIDANCE_H_MODE_CARE_FREE:
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_HOVER:
      if (!in_flight)
        guidance_h_hover_enter();

      guidance_h_update_reference();

#ifndef ALIGN_WITH_WIND
      /* set psi command */
      guidance_h_command_body.psi = guidance_h_rc_sp.psi;
#endif
      /* compute roll and pitch commands and set final attitude setpoint */
      guidance_h_traj_run(in_flight);

      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight)
        guidance_h_nav_enter();

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        struct Int32Eulers sp_euler_i;
        sp_euler_i.phi = nav_roll;
        sp_euler_i.theta = nav_pitch;
        /* FIXME: heading can't be set via attitude block yet, use current heading for now */
        sp_euler_i.psi = stateGetNedToBodyEulers_i()->psi;
        stabilization_attitude_set_from_eulers_i(&sp_euler_i);
      }
      else {
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

#ifdef USE_NAV_TRANSITION
        struct Int32Vect2 path_to_waypoint;
        VECT2_DIFF(path_to_waypoint, navigation_target, *stateGetPositionEnu_i());
        
        INT32_VECT2_NORM(dist_to_waypoint, path_to_waypoint);
        
        if(dist_to_waypoint > (15 << 8)) {

          if(transition_percentage < (transition_percentage_goal<<INT32_PERCENTAGE_FRAC)) {
            transition_run();
            guidance_h_command_body.theta = transition_theta_offset;
            guidance_h_command_body.phi = 0;
            stabilization_attitude_set_from_eulers_i(&guidance_h_command_body);
          }
          else {
            guidance_h_command_body.theta = transition_theta_offset;
            guidance_transitioned(in_flight);
//             call a function in aircraft_guidance.c?
            //           adjust heading with roll, only if transitioned
            //           get airspeed, use ff heading change with roll angle

            //           constant pitch, adjust altitude with throttle?

            //           (adjust speed with throttle)
            //           (adjust altitude with pitch)
          }
        }
        else {
          transition_percentage = 0;
          transition_theta_offset = 0;
          
          guidance_h_update_reference();
          
          #ifndef ALIGN_WITH_WIND
          /* set psi command */
          guidance_h_command_body.psi = nav_heading;
          #endif
          
          /* compute roll and pitch commands and set final attitude setpoint */
          guidance_h_traj_run(in_flight);
        }
#else

        guidance_h_update_reference();

#ifndef ALIGN_WITH_WIND
        /* set psi command */
        guidance_h_command_body.psi = nav_heading;
#endif

        /* compute roll and pitch commands and set final attitude setpoint */
        guidance_h_traj_run(in_flight);
#endif
      }
      stabilization_attitude_run(in_flight);
      break;

    default:
      break;
  }
}


static void guidance_h_update_reference(void) {
  /* compute reference even if usage temporarily disabled via guidance_h_use_ref */
#if GUIDANCE_H_USE_REF
  gh_update_ref_from_pos_sp(guidance_h_pos_sp);
#endif

  /* either use the reference or simply copy the pos setpoint */
  if (guidance_h_use_ref) {
    /* convert our reference to generic representation */
    INT32_VECT2_RSHIFT(guidance_h_pos_ref,   gh_pos_ref,   (GH_POS_REF_FRAC - INT32_POS_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_speed_ref, gh_speed_ref, (INT32_SPEED_FRAC - GH_SPEED_REF_FRAC));
    INT32_VECT2_LSHIFT(guidance_h_accel_ref, gh_accel_ref, (INT32_ACCEL_FRAC - GH_ACCEL_REF_FRAC));
  } else {
    VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
    INT_VECT2_ZERO(guidance_h_speed_ref);
    INT_VECT2_ZERO(guidance_h_accel_ref);
  }
}


#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#define MAX_POS_ERR_SUM ((int32_t)(MAX_POS_ERR)<< 12)

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

/** maximum bank angle: default 20 deg */
#define TRAJ_MAX_BANK BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC)

static void guidance_h_traj_run(bool_t in_flight) {

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* update pos error integral, zero it if not in_flight */
   if (in_flight) {
    VECT2_ADD(guidance_h_pos_err_sum, guidance_h_pos_err);
    /* saturate it               */
    VECT2_STRIM(guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);
   } else {
     INT_VECT2_ZERO(guidance_h_pos_err_sum);
   }

#ifdef ALIGN_WITH_WIND
  //Align the vehicle with the wind (forward or backward)
//   int32_t norm_distance_error_sum;
  //To avoid overflow, go back to 12 bitshifts total
//   INT32_VECT2_NORM(norm_distance_error_sum, (guidance_h_pos_err_sum >> 12));
  //If the integrator is higher than 1/4 of its maximum, start utilising wind alignment. This is to avoid rapid heading changes.
  if( (abs(guidance_h_pos_err_sum.x) > (MAX_POS_ERR_SUM>>2)) || (abs(guidance_h_pos_err_sum.y) > (MAX_POS_ERR_SUM>>2)) ) {
    //Calculate the angle of the integrated position error. guidance_h_pos_err_sum does not have a unit/scale, but because we are calculating the angle it does not matter.
    float guidance_h_pos_err_sum_angle_f = atan2f( (float) POS_FLOAT_OF_BFP(guidance_h_pos_err_sum.y), (float) POS_FLOAT_OF_BFP(guidance_h_pos_err_sum.x));

    //The difference of the current heading with the required heading. This determines forward or backward alignment.
    float guidance_heading_diff = guidance_h_pos_err_sum_angle_f - stateGetNedToBodyEulers_f()->psi;
    FLOAT_ANGLE_NORMALIZE(guidance_heading_diff);

    //Give the heading command.
    if(guidance_heading_diff > RadOfDeg(-90.0) && guidance_heading_diff < RadOfDeg(90.0)) {
      guidance_h_command_body.psi = ANGLE_BFP_OF_REAL(guidance_h_pos_err_sum_angle_f);
    }
    else {
      float backward_angle = guidance_h_pos_err_sum_angle_f + RadOfDeg(180.0);
      FLOAT_ANGLE_NORMALIZE(backward_angle);
      guidance_h_command_body.psi = ANGLE_BFP_OF_REAL(backward_angle);
    }
  }
#endif

  /* run PID */
  guidance_h_command_earth.x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2)) +
    ((guidance_h_igain * (guidance_h_pos_err_sum.x >> 12)) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8); /* feedforward gain */
  guidance_h_command_earth.y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2)) +
    ((guidance_h_igain * (guidance_h_pos_err_sum.y >> 12)) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8); /* feedforward gain */

  VECT2_STRIM(guidance_h_command_earth, -TRAJ_MAX_BANK, TRAJ_MAX_BANK);

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  // Restore angle ref resolution after rotation
  guidance_h_command_body.phi =
    ( - s_psi * guidance_h_command_earth.x + c_psi * guidance_h_command_earth.y) >> INT32_TRIG_FRAC;
  guidance_h_command_body.theta =
    - ( c_psi * guidance_h_command_earth.x + s_psi * guidance_h_command_earth.y) >> INT32_TRIG_FRAC;


  /* Add RC roll and pitch setpoints for emergency corrections */
  guidance_h_command_body.phi += guidance_h_rc_sp.phi;
  guidance_h_command_body.theta += guidance_h_rc_sp.theta;

  /* Set attitude setpoint from pseudo-eulers */
  stabilization_attitude_set_from_eulers_i(&guidance_h_command_body);
}

static void guidance_h_hover_enter(void) {

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();

  guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i()->psi;
  guidance_h_command_body.psi = stateGetNedToBodyEulers_i()->psi;
}

static void guidance_h_nav_enter(void) {

  /* horizontal position setpoint from navigation/flightplan */
  INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

  reset_guidance_reference_from_current_position();

  nav_heading = stateGetNedToBodyEulers_i()->psi;
}

static inline void transition_run(void) {
  //Add 0.00625%
  transition_percentage += 1<<(INT32_PERCENTAGE_FRAC-4);

#ifdef TRANSITION_MAX_OFFSET
  const int32_t max_offset = ANGLE_BFP_OF_REAL(TRANSITION_MAX_OFFSET);
  transition_theta_offset = INT_MULT_RSHIFT((transition_percentage<<(INT32_ANGLE_FRAC-INT32_PERCENTAGE_FRAC))/100, max_offset, INT32_ANGLE_FRAC);
#endif
}

/** maximum bank angle: default 20 deg */
#define FORWARD_MAX_BANK BFP_OF_REAL(GUIDANCE_H_FORWARD_MAX_BANK, INT32_ANGLE_FRAC)

static void guidance_transitioned(bool_t in_flight) {
  //Update the reference in case it is used for hovering
  VECT2_COPY(guidance_h_pos_ref, guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_speed_ref);
  INT_VECT2_ZERO(guidance_h_accel_ref);

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_sp, *stateGetPositionNed_i());

  carrot_heading = ANGLE_BFP_OF_REAL(atan2f( (float) POS_FLOAT_OF_BFP(guidance_h_pos_err.y), (float) POS_FLOAT_OF_BFP(guidance_h_pos_err.x)));

  int32_t heading_state = ANGLE_BFP_OF_REAL(stabilization_attitude_get_heading_f());

  heading_error = carrot_heading - heading_state;
  INT32_ANGLE_NORMALIZE(heading_error);

  //proportional roll angle
  guidance_h_command_body.phi = heading_error/3;
  //saturate it
  if(guidance_h_command_body.phi > FORWARD_MAX_BANK)
    guidance_h_command_body.phi = FORWARD_MAX_BANK;
  else if(guidance_h_command_body.phi < -FORWARD_MAX_BANK)
    guidance_h_command_body.phi = -FORWARD_MAX_BANK;

  //Coordinated turn
  //feedforward estimate angular rotation omega = g*tan(phi)/v
  //Take for now a constant v = 9.81/1.3 m/s
  int32_t omega;
  
  omega = ANGLE_BFP_OF_REAL(1.3*tanf(ANGLE_FLOAT_OF_BFP(guidance_h_command_body.phi)));
  
  guidance_h_command_body.psi += omega/512;

  INT32_ANGLE_NORMALIZE(guidance_h_command_body.psi);

  struct FloatQuat q_rp_cmd;
  guidance_roll_pitch_earth_quat_f(&q_rp_cmd, guidance_h_command_body.theta, guidance_h_command_body.phi);

  struct Int32Quat q_rp_cmd_i;
  QUAT_BFP_OF_REAL(q_rp_cmd_i, q_rp_cmd);
  
  const struct Int32Vect3 zaxis = {0, 0, 1};
  
  if (in_flight) {
    /* get current heading setpoint */
    struct Int32Quat q_yaw_sp;
    
    INT32_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, guidance_h_command_body.psi);
    
    INT32_QUAT_COMP(stab_att_sp_quat, q_yaw_sp, q_rp_cmd_i);
  }
  else {
    struct Int32Quat q_yaw;
    INT32_QUAT_OF_AXIS_ANGLE(q_yaw, zaxis, stateGetNedToBodyEulers_i()->psi);
    
    /* roll/pitch commands applied to to current heading */
    struct Int32Quat q_rp_sp;
    INT32_QUAT_COMP(q_rp_sp, q_yaw, q_rp_cmd_i);
    INT32_QUAT_NORMALIZE(q_rp_sp);
    
    QUAT_COPY(stab_att_sp_quat, q_rp_sp);
  }

//   heading error -> roll angle --> heading sp change
}

void guidance_roll_pitch_earth_quat_f(struct FloatQuat* q, int32_t pitch, int32_t roll) {
  /* only non-zero entries for roll quaternion */
  float roll2 = ANGLE_FLOAT_OF_BFP(roll)/2;
  float qx_roll = sinf(roll2);
  float qi_roll = cosf(roll2);
  
  //An offset is added if in forward mode
  /* only non-zero entries for pitch quaternion */
  float pitch2 = ANGLE_FLOAT_OF_BFP(pitch) / 2;
  float qy_pitch = sinf(pitch2);
  float qi_pitch = cosf(pitch2);
  
  /* only multiply non-zero entries of FLOAT_QUAT_COMP(*q, q_roll, q_pitch) */
  q->qi = qi_roll * qi_pitch;
  q->qx = qx_roll * qi_pitch;
  q->qy = qi_roll * qy_pitch;
  q->qz = qx_roll * qy_pitch;
}