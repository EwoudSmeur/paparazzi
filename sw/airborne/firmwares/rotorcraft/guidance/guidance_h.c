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

#include "generated/airframe.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/navigation.h"
#include "subsystems/radio_control.h"

/* for guidance_v_thrust_coeff */
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "state.h"

#ifndef GUIDANCE_H_AGAIN
#define GUIDANCE_H_AGAIN 0
#endif

#ifndef GUIDANCE_H_VGAIN
#define GUIDANCE_H_VGAIN 0
#endif

/* error if some gains are negative */
#if (GUIDANCE_H_PGAIN < 0) ||                   \
  (GUIDANCE_H_DGAIN < 0)   ||                   \
  (GUIDANCE_H_IGAIN < 0)   ||                   \
  (GUIDANCE_H_AGAIN < 0)   ||                   \
  (GUIDANCE_H_VGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

#ifndef GUIDANCE_H_MAX_BANK
#define GUIDANCE_H_MAX_BANK RadOfDeg(20)
#endif

PRINT_CONFIG_VAR(GUIDANCE_H_USE_REF)
PRINT_CONFIG_VAR(GUIDANCE_H_USE_SPEED_REF)

#ifndef GUIDANCE_H_APPROX_FORCE_BY_THRUST
#define GUIDANCE_H_APPROX_FORCE_BY_THRUST FALSE
#endif

// max airspeed for quadshot guidance
#define MAX_AIRSPEED 15

#define INT32_ANGLE_HIGH_RES_FRAC 18


uint8_t guidance_h_mode;
bool_t guidance_h_use_ref;
bool_t guidance_h_approx_force_by_thrust;

struct Int32Vect2 guidance_h_pos_sp;
struct Int32Vect2 guidance_h_pos_ref;
struct Int32Vect2 guidance_h_speed_ref;
struct Int32Vect2 guidance_h_accel_ref;
#if GUIDANCE_H_USE_SPEED_REF
struct Int32Vect2 guidance_h_speed_sp;
#endif
struct Int32Vect2 guidance_h_pos_err;
struct Int32Vect2 guidance_h_speed_err;
struct Int32Vect2 guidance_h_trim_att_integrator;

struct Int32Vect2  guidance_h_cmd_earth;
struct Int32Eulers guidance_h_rc_sp;
int32_t guidance_h_heading_sp;

struct Int32Eulers guidance_h_ypr_sp;
struct Int32Vect2 guidance_h_airspeed_sp;
struct Int32Vect2 guidance_h_airspeed_ref;
struct Int32Vect2 wind_estimate;
struct Int32Vect2 wind_estimate_high_res;
struct Int32Vect2 guidance_h_ref_airspeed;

int32_t guidance_h_pgain;
int32_t guidance_h_dgain;
int32_t guidance_h_igain;
int32_t guidance_h_again;
int32_t guidance_h_vgain;

int32_t transition_percentage;
int32_t transition_theta_offset;

int32_t norm_sp_airspeed_disp;
int32_t heading_diff_disp;
int32_t omega_disp;
int32_t high_res_psi;
int32_t airspeed_sp_heading_disp;
bool_t guidance_hovering;
int32_t horizontal_speed_gain;
int32_t norm_ref_airspeed;
int32_t max_airspeed = MAX_AIRSPEED;
float max_turn_bank;
float turn_bank_gain;
int32_t wind_gain;

static void guidance_h_update_reference(void);
static void guidance_h_traj_run(bool_t in_flight);
static void guidance_h_hover_enter(void);
static void guidance_h_nav_enter(void);
static inline void transition_run(void);
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight);
void stabilization_attitude_set_cmd_i(struct Int32Eulers *sp_cmd);
void guidance_h_airspeed_to_attitude(struct Int32Eulers *ypr_sp);
void guidance_h_position_to_airspeed(void);
void guidance_h_determine_wind_estimate(void);

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_gh(void) {
  struct NedCoor_i* pos = stateGetPositionNed_i();
  DOWNLINK_SEND_GUIDANCE_H_INT(DefaultChannel, DefaultDevice,
      &guidance_h_pos_sp.x, &guidance_h_pos_sp.y,
      &guidance_h_pos_ref.x, &guidance_h_pos_ref.y,
      &(pos->x), &(pos->y));
}

static void send_hover_loop(void) {
  struct NedCoor_i* pos = stateGetPositionNed_i();
  struct NedCoor_i* speed = stateGetSpeedNed_i();
  struct NedCoor_i* accel = stateGetAccelNed_i();
  DOWNLINK_SEND_HOVER_LOOP(DefaultChannel, DefaultDevice,
                           &guidance_h_airspeed_sp.x,
                           &guidance_h_airspeed_sp.y,
                           &(pos->x), &(pos->y),
                           &(speed->x), &(speed->y),
                           &wind_estimate.x, &wind_estimate.y,
                           &guidance_h_pos_err.x,
                           &guidance_h_pos_err.y,
                           &airspeed_sp_heading_disp,
                           &omega_disp,
                           &norm_ref_airspeed,
                           &heading_diff_disp,
                           &guidance_h_ypr_sp.theta,
                           &guidance_h_ypr_sp.phi,
                           &guidance_h_ypr_sp.psi);
}

static void send_href(void) {
  DOWNLINK_SEND_GUIDANCE_H_REF_INT(DefaultChannel, DefaultDevice,
      &guidance_h_pos_sp.x, &guidance_h_pos_ref.x,
      &guidance_h_speed_ref.x, &guidance_h_accel_ref.x,
      &guidance_h_pos_sp.y, &guidance_h_pos_ref.y,
      &guidance_h_speed_ref.y, &guidance_h_accel_ref.y);
}

static void send_tune_hover(void) {
  DOWNLINK_SEND_ROTORCRAFT_TUNE_HOVER(DefaultChannel, DefaultDevice,
      &radio_control.values[RADIO_ROLL],
      &radio_control.values[RADIO_PITCH],
      &radio_control.values[RADIO_YAW],
      &stabilization_cmd[COMMAND_ROLL],
      &stabilization_cmd[COMMAND_PITCH],
      &stabilization_cmd[COMMAND_YAW],
      &stabilization_cmd[COMMAND_THRUST],
      &(stateGetNedToBodyEulers_i()->phi),
      &(stateGetNedToBodyEulers_i()->theta),
      &(stateGetNedToBodyEulers_i()->psi));
}

static void send_vel_guidance(void) {
  struct NedCoor_i* pos = stateGetPositionNed_i();
  struct NedCoor_i* speed = stateGetSpeedNed_i();
  DOWNLINK_SEND_VEL_GUIDANCE(DefaultChannel, DefaultDevice,
                           &(pos->x), &(pos->y),
                           &(speed->x), &(speed->y),
                           &wind_estimate.x, &wind_estimate.y,
                           &guidance_h_pos_err.x,
                           &guidance_h_pos_err.y,
                           &guidance_h_airspeed_sp.x,
                           &guidance_h_airspeed_sp.y,
                           &norm_ref_airspeed,
                           &heading_diff_disp,
                           &guidance_h_ypr_sp.phi,
                           &guidance_h_ypr_sp.theta,
                           &guidance_h_ypr_sp.psi);
}

#endif

void guidance_h_init(void) {

  guidance_h_mode = GUIDANCE_H_MODE_KILL;
  guidance_h_use_ref = GUIDANCE_H_USE_REF;
  guidance_h_approx_force_by_thrust = GUIDANCE_H_APPROX_FORCE_BY_THRUST;

  INT_VECT2_ZERO(guidance_h_pos_sp);
  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  INT_EULERS_ZERO(guidance_h_rc_sp);
  INT_EULERS_ZERO(guidance_h_ypr_sp);
  INT_VECT2_ZERO(guidance_h_airspeed_sp);
  INT_VECT2_ZERO(guidance_h_airspeed_ref);
  guidance_h_heading_sp = 0;
  guidance_h_pgain = GUIDANCE_H_PGAIN;
  guidance_h_igain = GUIDANCE_H_IGAIN;
  guidance_h_dgain = GUIDANCE_H_DGAIN;
  guidance_h_again = GUIDANCE_H_AGAIN;
  guidance_h_vgain = GUIDANCE_H_VGAIN;
  transition_percentage = 0;
  transition_theta_offset = 0;
  high_res_psi = 0;
  guidance_hovering = true;
  horizontal_speed_gain = 6;
  norm_ref_airspeed = 0;
  max_turn_bank = 40.0;
  turn_bank_gain = 0.8;
  wind_gain = 64;
  INT_VECT2_ZERO(wind_estimate);
  INT_VECT2_ZERO(guidance_h_ref_airspeed);
  INT_VECT2_ZERO(wind_estimate_high_res);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_INT", send_gh);
  register_periodic_telemetry(DefaultPeriodic, "HOVER_LOOP", send_hover_loop);
  register_periodic_telemetry(DefaultPeriodic, "GUIDANCE_H_REF", send_href);
  register_periodic_telemetry(DefaultPeriodic, "ROTORCRAFT_TUNE_HOVER", send_tune_hover);
  register_periodic_telemetry(DefaultPeriodic, "VEL_GUIDANCE", send_vel_guidance);
#endif
}


static inline void reset_guidance_reference_from_current_position(void) {
  VECT2_COPY(guidance_h_pos_ref, *stateGetPositionNed_i());
  VECT2_COPY(guidance_h_speed_ref, *stateGetSpeedNed_i());
  INT_VECT2_ZERO(guidance_h_accel_ref);
  gh_set_ref(guidance_h_pos_ref, guidance_h_speed_ref, guidance_h_accel_ref);

  INT_VECT2_ZERO(guidance_h_trim_att_integrator);
}

void guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == guidance_h_mode)
    return;

  if (new_mode != GUIDANCE_H_MODE_FORWARD && new_mode != GUIDANCE_H_MODE_RATE) {
     transition_percentage = 0;
     transition_theta_offset = 0;
   }

   norm_ref_airspeed = 0;

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
    case GUIDANCE_H_MODE_ATTITUDE:
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_HOVER:
      guidance_h_hover_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
      break;

    case GUIDANCE_H_MODE_NAV:
      guidance_h_nav_enter();
#if NO_ATTITUDE_RESET_ON_MODE_CHANGE
      /* reset attitude stabilization if previous mode was not using it */
      if (guidance_h_mode == GUIDANCE_H_MODE_KILL ||
          guidance_h_mode == GUIDANCE_H_MODE_RATE ||
          guidance_h_mode == GUIDANCE_H_MODE_RC_DIRECT)
#endif
        stabilization_attitude_enter();
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
    case GUIDANCE_H_MODE_CARE_FREE:
      stabilization_attitude_read_rc(in_flight, TRUE, FALSE);
      break;
    case GUIDANCE_H_MODE_FORWARD:
      stabilization_attitude_read_rc(in_flight, FALSE, TRUE);
      break;
    case GUIDANCE_H_MODE_ATTITUDE:
      stabilization_attitude_read_rc(in_flight, FALSE, FALSE);
      break;
    case GUIDANCE_H_MODE_HOVER:
      stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
#if GUIDANCE_H_USE_SPEED_REF
      read_rc_setpoint_speed_i(&guidance_h_speed_sp, in_flight);
#endif
      break;

    case GUIDANCE_H_MODE_NAV:
      if (radio_control.status == RC_OK) {
        stabilization_attitude_read_rc_setpoint_eulers(&guidance_h_rc_sp, in_flight, FALSE, FALSE);
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
      if(transition_percentage < (100<<INT32_PERCENTAGE_FRAC)) {
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

      /* set psi command */
      guidance_h_heading_sp = guidance_h_rc_sp.psi;
      /* compute x,y earth commands */
      guidance_h_traj_run(in_flight);
      /* set final attitude setpoint */
      stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                             guidance_h_heading_sp);
      stabilization_attitude_run(in_flight);
      break;

    case GUIDANCE_H_MODE_NAV:
      if (!in_flight)
        guidance_h_nav_enter();

      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        struct Int32Eulers sp_cmd_i;
        sp_cmd_i.phi = nav_roll;
        sp_cmd_i.theta = nav_pitch;
        /** @todo: heading can't be set via attitude block yet.
         * use current euler psi for now, should be real heading
         */
        sp_cmd_i.psi = stateGetNedToBodyEulers_i()->psi;
        //make sure the heading is right before leaving horizontal_mode attiude
        guidance_h_ypr_sp.psi = sp_cmd_i.psi;
        high_res_psi = sp_cmd_i.psi << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
        stabilization_attitude_set_rpy_setpoint_i(&sp_cmd_i);
      }
      else {

#if QUADSHOT_NAVIGATION
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_target);
        guidance_h_determine_wind_estimate();
        guidance_h_position_to_airspeed();

        guidance_h_airspeed_to_attitude(&guidance_h_ypr_sp);

        stabilization_attitude_set_cmd_i(&guidance_h_ypr_sp);
#else
        INT32_VECT2_NED_OF_ENU(guidance_h_pos_sp, navigation_carrot);

        guidance_h_update_reference();

        /* set psi command */
        guidance_h_heading_sp = nav_heading;
        INT32_ANGLE_NORMALIZE(guidance_h_heading_sp);
        /* compute x,y earth commands */
        guidance_h_traj_run(in_flight);
        /* set final attitude setpoint */
        stabilization_attitude_set_earth_cmd_i(&guidance_h_cmd_earth,
                                               guidance_h_heading_sp);
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
#if GUIDANCE_H_USE_SPEED_REF
  if(guidance_h_mode == GUIDANCE_H_MODE_HOVER)
    gh_update_ref_from_speed_sp(guidance_h_speed_sp);
  else
#endif
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

#if GUIDANCE_H_USE_SPEED_REF
  if (guidance_h_mode == GUIDANCE_H_MODE_HOVER) {
    VECT2_COPY(guidance_h_pos_sp, guidance_h_pos_ref); // for display only
  }
#endif
}


#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

#ifndef GUIDANCE_H_THRUST_CMD_FILTER
#define GUIDANCE_H_THRUST_CMD_FILTER 10
#endif

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

static void guidance_h_traj_run(bool_t in_flight) {
  /* maximum bank angle: default 20 deg, max 40 deg*/
  static const int32_t traj_max_bank = Max(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
                                           BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
  static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_ref, *stateGetPositionNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  VECT2_DIFF(guidance_h_speed_err, guidance_h_speed_ref, *stateGetSpeedNed_i());
  /* saturate it               */
  VECT2_STRIM(guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

  /* run PID */
  int32_t pd_x =
    ((guidance_h_pgain * guidance_h_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  int32_t pd_y =
    ((guidance_h_pgain * guidance_h_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
    ((guidance_h_dgain * (guidance_h_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
  guidance_h_cmd_earth.x =
    pd_x +
    ((guidance_h_vgain * guidance_h_speed_ref.x) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.x) >> 8);   /* acceleration feedforward gain */
  guidance_h_cmd_earth.y =
    pd_y +
    ((guidance_h_vgain * guidance_h_speed_ref.y) >> 17) + /* speed feedforward gain */
    ((guidance_h_again * guidance_h_accel_ref.y) >> 8);   /* acceleration feedforward gain */

  /* trim max bank angle from PD */
  VECT2_STRIM(guidance_h_cmd_earth, -traj_max_bank, traj_max_bank);

  /* Update pos & speed error integral, zero it if not in_flight.
   * Integrate twice as fast when not only POS but also SPEED are wrong,
   * but do not integrate POS errors when the SPEED is already catching up.
   */
  if (in_flight) {
    /* ANGLE_FRAC (12) * GAIN (8) * LOOP_FREQ (9) -> INTEGRATOR HIGH RES ANGLE_FRAX (28) */
    guidance_h_trim_att_integrator.x += (guidance_h_igain * pd_x);
    guidance_h_trim_att_integrator.y += (guidance_h_igain * pd_y);
    /* saturate it  */
    VECT2_STRIM(guidance_h_trim_att_integrator, -(traj_max_bank << 16), (traj_max_bank << 16));
    /* add it to the command */
    guidance_h_cmd_earth.x += (guidance_h_trim_att_integrator.x >> 16);
    guidance_h_cmd_earth.y += (guidance_h_trim_att_integrator.y >> 16);
  } else {
    INT_VECT2_ZERO(guidance_h_trim_att_integrator);
  }

  /* compute a better approximation of force commands by taking thrust into account */
  if (guidance_h_approx_force_by_thrust && in_flight) {
    static int32_t thrust_cmd_filt;
    int32_t vertical_thrust = (stabilization_cmd[COMMAND_THRUST] * guidance_v_thrust_coeff) >> INT32_TRIG_FRAC;
    thrust_cmd_filt = (thrust_cmd_filt * GUIDANCE_H_THRUST_CMD_FILTER + vertical_thrust) / (GUIDANCE_H_THRUST_CMD_FILTER + 1);
    guidance_h_cmd_earth.x = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.x * MAX_PPRZ / INT32_ANGLE_PI_2), thrust_cmd_filt));
    guidance_h_cmd_earth.y = ANGLE_BFP_OF_REAL(atan2f((guidance_h_cmd_earth.y * MAX_PPRZ / INT32_ANGLE_PI_2), thrust_cmd_filt));
  }

  VECT2_STRIM(guidance_h_cmd_earth, -total_max_bank, total_max_bank);
}

static void guidance_h_hover_enter(void) {

  /* set horizontal setpoint to current position */
  VECT2_COPY(guidance_h_pos_sp, *stateGetPositionNed_i());

  reset_guidance_reference_from_current_position();

  guidance_h_rc_sp.psi = stateGetNedToBodyEulers_i()->psi;
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

/// read speed setpoint from RC
static void read_rc_setpoint_speed_i(struct Int32Vect2 *speed_sp, bool_t in_flight) {
  if (in_flight) {
    // negative pitch is forward
    int64_t rc_x = -radio_control.values[RADIO_PITCH];
    int64_t rc_y = radio_control.values[RADIO_ROLL];
    DeadBand(rc_x, MAX_PPRZ/20);
    DeadBand(rc_y, MAX_PPRZ/20);

    // convert input from MAX_PPRZ range to SPEED_BFP
    int32_t max_speed = SPEED_BFP_OF_REAL(GUIDANCE_H_REF_MAX_SPEED);
    /// @todo calc proper scale while making sure a division by zero can't occur
    //int32_t rc_norm = sqrtf(rc_x * rc_x + rc_y * rc_y);
    //int32_t max_pprz = rc_norm * MAX_PPRZ / Max(abs(rc_x), abs(rc_y);
    rc_x = rc_x * max_speed / MAX_PPRZ;
    rc_y = rc_y * max_speed / MAX_PPRZ;

    /* Rotate from body to NED frame by negative psi angle */
    int32_t psi = -stateGetNedToBodyEulers_i()->psi;
    int32_t s_psi, c_psi;
    PPRZ_ITRIG_SIN(s_psi, psi);
    PPRZ_ITRIG_COS(c_psi, psi);
    speed_sp->x = (int32_t)(( (int64_t)c_psi * rc_x + (int64_t)s_psi * rc_y) >> INT32_TRIG_FRAC);
    speed_sp->y = (int32_t)((-(int64_t)s_psi * rc_x + (int64_t)c_psi * rc_y) >> INT32_TRIG_FRAC);
  }
  else {
    speed_sp->x = 0;
    speed_sp->y = 0;
  }
}

#define INT32_ANGLE_HIGH_RES_NORMALIZE(_a) {             \
  while ((_a) > (INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC)))  (_a) -= (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
  while ((_a) < (-INT32_ANGLE_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC))) (_a) += (INT32_ANGLE_2_PI << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC));    \
}

/// Convert a required airspeed to a certain attitude for the Quadshot
void guidance_h_airspeed_to_attitude(struct Int32Eulers *ypr_sp) {

  //notes:
  //in forward flight, it is preferred to first get to min(airspeed_sp, airspeed_ref) and then change heading and then get to airspeed_sp
  //in hover, just a gradual change is needed, or maybe not even needed
  //changes between flight regimes should be handled

  //determine the heading of the airspeed_sp vector
  int32_t omega = 0;
  float airspeed_sp_heading = atan2f( (float) POS_FLOAT_OF_BFP(guidance_h_airspeed_sp.y), (float) POS_FLOAT_OF_BFP(guidance_h_airspeed_sp.x));
  //only for debugging
  airspeed_sp_heading_disp = (int32_t) (DegOfRad(airspeed_sp_heading));

  //The difference of the current heading with the required heading.
  float heading_diff = airspeed_sp_heading - ANGLE_FLOAT_OF_BFP(ypr_sp->psi);
  FLOAT_ANGLE_NORMALIZE(heading_diff);

  //only for debugging
  heading_diff_disp = (int32_t) (heading_diff/3.14*180.0);

  //calculate the norm of the airspeed setpoint
  int32_t norm_sp_airspeed;
  INT32_VECT2_NORM(norm_sp_airspeed,guidance_h_airspeed_sp);

  //reference goes with a steady pace towards the setpoint airspeed
  //hold ref norm below 4 m/s until heading is aligned
  if( !((norm_sp_airspeed > (4<<8)) && (norm_ref_airspeed < (4<<8)) && (norm_ref_airspeed > ((4<<8)-10)) && (fabs(heading_diff) > (5.0/180.0*3.14))) )
    norm_ref_airspeed = norm_ref_airspeed +  ( (int32_t) (norm_sp_airspeed > norm_ref_airspeed) * 2 - 1)*3/2;

  norm_sp_airspeed_disp = norm_sp_airspeed;

  int32_t psi = ypr_sp->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);

  guidance_h_ref_airspeed.x = (norm_ref_airspeed * c_psi) >> INT32_TRIG_FRAC;
  guidance_h_ref_airspeed.y = (norm_ref_airspeed * s_psi) >> INT32_TRIG_FRAC;

  if(norm_ref_airspeed < (4<<8)) {
    /// if required speed is lower than 4 m/s act like a rotorcraft
    // translate speed_sp into bank angle and heading

    // change heading to direction of airspeed, faster if the airspeed is higher
    if(heading_diff > 0.0)
      omega = (norm_sp_airspeed << (INT32_ANGLE_FRAC - INT32_POS_FRAC))/6;
    else if(heading_diff < 0.0)
      omega = (norm_sp_airspeed << (INT32_ANGLE_FRAC - INT32_POS_FRAC))/-6;

    if(omega > ANGLE_BFP_OF_REAL(0.8)) omega = ANGLE_BFP_OF_REAL(0.8);
    if(omega < ANGLE_BFP_OF_REAL(-0.8)) omega = ANGLE_BFP_OF_REAL(-0.8);

    // 2) calculate roll/pitch commands
    struct Int32Vect2 hover_sp;
    if(norm_sp_airspeed > (4<<8)) { //if the setpoint is beyond 4m/s but the ref is not, the norm of the hover sp will stay at 4m/s
      hover_sp.x = (guidance_h_airspeed_sp.x << 8)/norm_sp_airspeed * 4;
      hover_sp.y = (guidance_h_airspeed_sp.y << 8)/norm_sp_airspeed * 4;
    }
    else {
      hover_sp.x = guidance_h_airspeed_sp.x;
      hover_sp.y = guidance_h_airspeed_sp.y;
    }

    // gain of 10 means that for 4 m/s an angle of 40 degrees is needed
    ypr_sp->theta = (((- ( c_psi * hover_sp.x + s_psi * hover_sp.y)) >> INT32_TRIG_FRAC) * 10*INT32_ANGLE_PI/180) >> 8;
    ypr_sp->phi = (((( - s_psi * hover_sp.x + c_psi * hover_sp.y)) >> INT32_TRIG_FRAC) * 10*INT32_ANGLE_PI/180) >>  8;
  }
  else {
    /// if required speed is higher than 4 m/s act like a fixedwing
    // translate speed_sp into theta + thrust
    // coordinated turns to change heading

    // calculate required pitch angle from airspeed_sp magnitude
    if(norm_ref_airspeed > (15<<8))
      ypr_sp->theta = -ANGLE_BFP_OF_REAL(RadOfDeg(78.0));
    else if(norm_ref_airspeed > (8<<8))
      ypr_sp->theta = -(( (norm_ref_airspeed - (8<<8)) * 2*INT32_ANGLE_PI/180) >> 8) - ANGLE_BFP_OF_REAL(RadOfDeg(68.0));
    else 
      ypr_sp->theta = -(( (norm_ref_airspeed - (4<<8)) * 7*INT32_ANGLE_PI/180) >> 8) - ANGLE_BFP_OF_REAL(RadOfDeg(40.0));

    // if the sp_airspeed is within hovering range, don't start a coordinated turn
    if(norm_sp_airspeed < (4<<8)) {
      omega = 0;
      ypr_sp->phi = 0;
    }
    else { // coordinated turn
      ypr_sp->phi = ANGLE_BFP_OF_REAL(heading_diff*turn_bank_gain);
      if(ypr_sp->phi > ANGLE_BFP_OF_REAL(max_turn_bank/180.0*M_PI)) ypr_sp->phi = ANGLE_BFP_OF_REAL(max_turn_bank/180.0*M_PI);
      if(ypr_sp->phi < ANGLE_BFP_OF_REAL(-max_turn_bank/180.0*M_PI)) ypr_sp->phi = ANGLE_BFP_OF_REAL(-max_turn_bank/180.0*M_PI);

      //feedforward estimate angular rotation omega = g*tan(phi)/v
      omega = ANGLE_BFP_OF_REAL(9.81/POS_FLOAT_OF_BFP(norm_ref_airspeed)*tanf(ANGLE_FLOAT_OF_BFP(ypr_sp->phi)));

      if(omega > ANGLE_BFP_OF_REAL(0.7)) omega = ANGLE_BFP_OF_REAL(0.7);
      if(omega < ANGLE_BFP_OF_REAL(-0.7)) omega = ANGLE_BFP_OF_REAL(-0.7);
    }
  }

  //only for debugging purposes
  omega_disp = omega;

  //go to higher resolution because else the increment is too small to be added
  high_res_psi += (omega << (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC))/512;

  INT32_ANGLE_HIGH_RES_NORMALIZE(high_res_psi);

  // go back to angle_frac
  ypr_sp->psi = high_res_psi >> (INT32_ANGLE_HIGH_RES_FRAC - INT32_ANGLE_FRAC);
  ypr_sp->theta = ypr_sp->theta + v_control_pitch;
}

void guidance_h_position_to_airspeed(void) {
  /* compute position error    */
  VECT2_DIFF(guidance_h_pos_err, guidance_h_pos_sp, *stateGetPositionNed_i());

  // Compute ground speed setpoint
  struct Int32Vect2 guidance_h_groundspeed_sp;
  VECT2_SDIV(guidance_h_groundspeed_sp, guidance_h_pos_err, horizontal_speed_gain);

  struct Int32Vect2 airspeed_sp;
  INT_VECT2_ZERO(airspeed_sp);
  VECT2_ADD(airspeed_sp, guidance_h_groundspeed_sp);
  VECT2_ADD(airspeed_sp, wind_estimate);

  int32_t norm_groundspeed_sp;
  INT32_VECT2_NORM(norm_groundspeed_sp, guidance_h_groundspeed_sp);

  int32_t norm_airspeed_sp;
  INT32_VECT2_NORM(norm_airspeed_sp, airspeed_sp);

  if( norm_airspeed_sp > (max_airspeed<<8) && norm_groundspeed_sp > 0) {
    int32_t av = INT_MULT_RSHIFT(guidance_h_groundspeed_sp.x, guidance_h_groundspeed_sp.x,8) + INT_MULT_RSHIFT(guidance_h_groundspeed_sp.y, guidance_h_groundspeed_sp.y, 8);
    int32_t bv = 2*( INT_MULT_RSHIFT(wind_estimate.x, guidance_h_groundspeed_sp.x, 8) + INT_MULT_RSHIFT(wind_estimate.y, guidance_h_groundspeed_sp.y, 8));
    int32_t cv = INT_MULT_RSHIFT(wind_estimate.x, wind_estimate.x, 8) + INT_MULT_RSHIFT(wind_estimate.y, wind_estimate.y, 8) - (max_airspeed<<8)*max_airspeed;

    float dv = POS_FLOAT_OF_BFP(bv) * POS_FLOAT_OF_BFP(bv) - 4.0* POS_FLOAT_OF_BFP(av) * POS_FLOAT_OF_BFP(cv);
    float d_sqrt_f = sqrtf(dv);
    int32_t d_sqrt = POS_BFP_OF_REAL(d_sqrt_f);

    int32_t result = ((-bv + d_sqrt)<<8)/(2*av);

    guidance_h_airspeed_sp.x = wind_estimate.x + INT_MULT_RSHIFT(guidance_h_groundspeed_sp.x, result, 8);
    guidance_h_airspeed_sp.y = wind_estimate.y + INT_MULT_RSHIFT(guidance_h_groundspeed_sp.y, result, 8);
  }
  else {
    // Add the wind to get the airspeed setpoint
    guidance_h_airspeed_sp = guidance_h_groundspeed_sp;
    VECT2_ADD(guidance_h_airspeed_sp, wind_estimate);
  }

//   limit the airspeed setpoint to 15 m/s, because else saturation+windup will occur
  INT32_VECT2_NORM(norm_airspeed_sp, guidance_h_airspeed_sp);
  if(norm_airspeed_sp > (max_airspeed<<8)) {
    guidance_h_airspeed_sp.x = guidance_h_airspeed_sp.x*(max_airspeed<<8)/norm_airspeed_sp;
    guidance_h_airspeed_sp.y = guidance_h_airspeed_sp.y*(max_airspeed<<8)/norm_airspeed_sp;
  }
}

void guidance_h_determine_wind_estimate(void) {

  /* compute speed error    */
  struct Int32Vect2 wind_estimate_measured;
  struct Int32Vect2 measured_ground_speed;
  INT32_VECT2_RSHIFT(measured_ground_speed, *stateGetSpeedNed_i(), 11);
  VECT2_DIFF(wind_estimate_measured, guidance_h_ref_airspeed, measured_ground_speed );

  //Low pass wind_estimate, because we know the wind usually only changes slowly
  //But not too slow, because the wind_estimate is also an adaptive element for the airspeed model inaccuracies
  wind_estimate_high_res.x += (( (wind_estimate_measured.x - wind_estimate.x) > 0)*2-1) * wind_gain;
  wind_estimate_high_res.y += (( (wind_estimate_measured.y - wind_estimate.y) > 0)*2-1) * wind_gain;

  wind_estimate.x = ((wind_estimate_high_res.x) >> 8);
  wind_estimate.y = ((wind_estimate_high_res.y) >> 8);
}

void stabilization_attitude_set_cmd_i(struct Int32Eulers *sp_cmd) {
  /// @todo calc sp_quat in fixed-point
  
  /* orientation vector describing simultaneous rotation of roll/pitch */
  struct FloatVect3 ov;
  ov.x = ANGLE_FLOAT_OF_BFP(sp_cmd->phi);
  ov.y = ANGLE_FLOAT_OF_BFP(sp_cmd->theta);
  ov.z = 0.0;
  /* quaternion from that orientation vector */
  struct FloatQuat q_rp;
  FLOAT_QUAT_OF_ORIENTATION_VECT(q_rp, ov);
  struct Int32Quat q_rp_i;
  QUAT_BFP_OF_REAL(q_rp_i, q_rp);

  //   get the vertical vector to rotate the roll pitch setpoint around
  const struct Int32Vect3 zaxis = {0, 0, 1};

  /* get current heading setpoint */
  struct Int32Quat q_yaw_sp;
  INT32_QUAT_OF_AXIS_ANGLE(q_yaw_sp, zaxis, sp_cmd->psi);

  //   first apply the roll/pitch setpoint and then the yaw
  INT32_QUAT_COMP(stab_att_sp_quat, q_yaw_sp, q_rp_i);
}
