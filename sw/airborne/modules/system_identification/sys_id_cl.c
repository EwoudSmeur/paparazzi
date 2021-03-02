/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/system_identification/sys_id_cl.c
 * @brief module that executes sys_id maneuvers in closed-loop
 *
 */

#include "modules/system_identification/sys_id_cl.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/actuators.h"
#include "filters/low_pass_filter.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_random.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

struct sys_id_cl_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;

} sys_id_cl;

float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
float ctrl_module_demo_pr_d_gain;
float ctrl_module_demo_y_ff_gain;   // Yaw
float ctrl_module_demo_y_d_gain;

float time_roll_0 = 0;
uint8_t actuators_combination = 0;

uint8_t signal_type_pitch = 0;
uint8_t increase_first_pitch = 1;
pprz_t amplitude_pitch = 0;
float timestep_pitch = 0.1;
float fstart_chirp_pitch = 0.05;
float fend_chirp_pitch = 0.1;
float length_chirp_pitch = 0.1;
float chirp_noise_ratio_pitch = 0;

uint8_t signal_type_thrust = 0;
uint8_t increase_first_thrust = 1;
pprz_t amplitude_thrust = 0;
float timestep_thrust = 0.1;
float fstart_chirp_thrust = 0.05;
float fend_chirp_thrust = 0.1;
float length_chirp_thrust = 0.1;
float chirp_noise_ratio_thrust = 0;

float time_shift = 0.1;

struct signal_config_t {
  uint8_t signal_type;
  uint8_t increase_first;
  float amplitude;
  float timestep;
  float fstart_chirp;
  float fend_chirp;
  float length_chirp;
  float chirp_noise_ratio;
  float time_start;
  struct FirstOrderLowPass filter_noise;
};

struct signal_config_t signal_pitch;
struct signal_config_t signal_thrust;

int32_t ref_thrust;
struct FloatEulers ref_eulers;

float delta_input_pitch;
float delta_input_thrust;

#define CHIRP_C1 4.0f
#define CHIRP_C2 1.0f / (expf(CHIRP_C1) - 1)

void sys_id_cl_init(void);
void sys_id_cl_run(bool in_flight);
void sys_id_cl_enter(void);
float sid_signal_generator(struct signal_config_t *signal, float current_time);

void sys_id_cl_init(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averageFirstOrderLowPassd_command

  sys_id_cl.rc_x = 0;
  sys_id_cl.rc_y = 0;
  sys_id_cl.rc_z = 0;
  sys_id_cl.rc_t = 0;

  ref_eulers.phi = 0;
  ref_eulers.theta = 0;
  ref_eulers.psi = 0;
  ref_thrust = 0;

  signal_pitch.signal_type = signal_type_pitch;
  signal_pitch.increase_first = increase_first_pitch;
  signal_pitch.amplitude = amplitude_pitch * M_PI / 180;
  signal_pitch.timestep = timestep_pitch;
  signal_pitch.fstart_chirp = fstart_chirp_pitch;
  signal_pitch.fend_chirp = fend_chirp_pitch;
  signal_pitch.length_chirp = length_chirp_pitch;
  signal_pitch.chirp_noise_ratio = chirp_noise_ratio_pitch;
  signal_pitch.time_start = 0.f;

  signal_thrust.signal_type = signal_type_thrust;
  signal_thrust.increase_first = increase_first_thrust;
  signal_thrust.amplitude = amplitude_thrust;
  signal_thrust.timestep = timestep_thrust;
  signal_thrust.fstart_chirp = fstart_chirp_thrust;
  signal_thrust.fend_chirp = fend_chirp_thrust;
  signal_thrust.length_chirp = length_chirp_thrust;
  signal_thrust.chirp_noise_ratio = chirp_noise_ratio_thrust;
  signal_thrust.time_start = 0.f;

  float sample_time = 1 / PERIODIC_FREQUENCY;

  float tau_filter_noise_pitch = 1 / (signal_pitch.fend_chirp * 2 * M_PI);
  init_first_order_low_pass(&signal_pitch.filter_noise, tau_filter_noise_pitch, sample_time, 0);

  float tau_filter_noise_thrust = 1 / (signal_thrust.fend_chirp * 2 * M_PI);
  init_first_order_low_pass(&signal_thrust.filter_noise, tau_filter_noise_thrust, sample_time, 0);

  init_random();

  delta_input_pitch = 0.f;
  delta_input_thrust = 0.f;
}

void sys_id_cl_enter(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averaged_command

  sys_id_cl_init();

  float_eulers_of_quat_zxy(&ref_eulers, stateGetNedToBodyQuat_f());
  ref_thrust = stabilization_cmd[COMMAND_THRUST];

  float time_start = get_sys_time_float();
  signal_pitch.time_start = time_start + time_roll_0;
  signal_thrust.time_start = time_start + time_roll_0;

  // float amplitude_max_pitch = (90 - ABS(final_trim[0]));
  // signal_pitch.amplitude = Min(amplitude_pitch,amplitude_max_pitch);

  float amplitude_max_thrust = MAX_PPRZ - ref_thrust;
  signal_thrust.amplitude = Min(amplitude_thrust,amplitude_max_thrust);

  if (actuators_combination == 2)
  {
    if (signal_pitch.signal_type == 0)
    {
      signal_thrust.time_start += ((2 * signal_pitch.timestep) + time_shift);
    }
    else if (signal_pitch.signal_type == 1)
    {
      signal_thrust.time_start += ((7 * signal_pitch.timestep) + time_shift);
    }
    else if (signal_pitch.signal_type == 2)
    {
      signal_thrust.time_start += (signal_pitch.length_chirp + time_shift);
    }
    else
    {
      signal_thrust.time_start += ((4 * signal_pitch.timestep) + time_shift);
    }
  }

  if (actuators_combination == 3)
  {
    if (signal_thrust.signal_type == 0)
    {
      signal_pitch.time_start += ((2 * signal_thrust.timestep) + time_shift);
    }
    else if (signal_thrust.signal_type == 1)
    {
      signal_pitch.time_start += ((7 * signal_thrust.timestep) + time_shift);
    }
    else if (signal_thrust.signal_type == 2)
    {
      signal_pitch.time_start += (signal_thrust.length_chirp + time_shift);
    }
    else
    {
      signal_pitch.time_start += ((4 * signal_thrust.timestep) + time_shift);
    }
  }
}

float sid_signal_generator(struct signal_config_t *signal, float current_time)
{
  float delta_input_normalized = 0.f;
  float delta_input = 0.f;
  float elapsed_time = (current_time - signal->time_start);

  if (elapsed_time >= 0)
  {
    if (signal->signal_type == 0)
    {

      if (elapsed_time < (2 * signal->timestep))
      {
        if (elapsed_time < signal->timestep)
        {
          delta_input_normalized = -1.f;
        }
        else
        {
          delta_input_normalized = 1.f;
        }

        if (signal->increase_first == 1)
        {
          delta_input_normalized = -(delta_input_normalized);
        }
      }

    }
    else if (signal->signal_type == 1)
    {

      if (elapsed_time < (7 * signal->timestep))
      {
        if (elapsed_time < signal->timestep)
        {
          delta_input_normalized = -1.f;
        }
        else if (elapsed_time < (2 * signal->timestep))
        {
          delta_input_normalized = 1.f;
        }
        else if (elapsed_time < (4 * signal->timestep))
        {
          delta_input_normalized = -1.f;
        }
        else
        {
          delta_input_normalized = 1.f;
        }

        if (signal->increase_first == 1)
        {
          delta_input_normalized = -(delta_input_normalized);
        }
      }
    }
    else if (signal->signal_type == 2)
    {
      if (elapsed_time <= signal->length_chirp)
      {

      float exponential = expf(CHIRP_C1 * elapsed_time / signal->length_chirp);
      float K = CHIRP_C2 * (exponential - 1);
      float theta = 2 * M_PI * (signal->fstart_chirp * elapsed_time
            + (signal->fend_chirp - signal->fstart_chirp) * (signal->length_chirp / CHIRP_C1 * K - CHIRP_C2 * elapsed_time));
      float delta_sweep_normalized = sinf(theta);

      if (signal->increase_first == 0)
      {
        delta_sweep_normalized = -(delta_sweep_normalized);
      }


      float delta_noise_normalized = signal->chirp_noise_ratio * (update_first_order_low_pass(&signal->filter_noise, rand_gaussian()));
      delta_input_normalized = delta_sweep_normalized + delta_noise_normalized;
      }
    }
    else
    {

      if (elapsed_time < (4 * signal->timestep))
      {
        if (elapsed_time < signal->timestep)
        {
          delta_input_normalized = -1.f;
        }
        else if (elapsed_time < (2 * signal->timestep))
        {
          delta_input_normalized = 1.f;
        }
        else if (elapsed_time < (3 * signal->timestep))
        {
          delta_input_normalized = -1.f;
        }
        else
        {
          delta_input_normalized = 1.f;
        }

        if (signal->increase_first == 1)
        {
          delta_input_normalized = -(delta_input_normalized);
        }
      }

    }

  delta_input = (signal->amplitude) * delta_input_normalized;
  }

  return delta_input;
}

// simple rate control without reference model nor attitude
void sys_id_cl_run(bool in_flight __attribute__((unused)))
{
  // Signal generation for the different actuators cases

  float time = get_sys_time_float();

  if (actuators_combination == 0)
  {
    delta_input_pitch = sid_signal_generator(&signal_pitch,time);
  }
  else if (actuators_combination == 1)
  {
    delta_input_thrust = sid_signal_generator(&signal_thrust,time);
  }
  else
  {
    delta_input_pitch = sid_signal_generator(&signal_pitch,time);
    delta_input_thrust = sid_signal_generator(&signal_thrust,time);
  }

  struct FloatEulers sp_eulers;
  sp_eulers.phi = 0;
  sp_eulers.theta = ref_eulers.theta + delta_input_pitch;
  sp_eulers.psi = ref_eulers.psi;
  float total_input_thrust = ref_thrust + delta_input_thrust;

  BoundAbs(total_input_thrust, MAX_PPRZ);

  // Execute commands with bounds on the actuators
  stabilization_indi_set_rpy_setpoint_i_zxy(&sp_eulers);
  stabilization_cmd[COMMAND_THRUST] = total_input_thrust;

}

// Only use for things that need to run when the module is not active!
// For instance keeping track of the trim condition

void sys_id_cl_periodic_init(void)
{

}

void sys_id_cl_periodic(void)
{
  //TODO average out commands (actuators_pprz)
  //

}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  sys_id_cl_init();
}

void guidance_h_module_enter(void)
{
  sys_id_cl_enter();
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  sys_id_cl.rc_t = radio_control.values[RADIO_THROTTLE];
  sys_id_cl.rc_x = radio_control.values[RADIO_ROLL];
  sys_id_cl.rc_y = radio_control.values[RADIO_PITCH];
  sys_id_cl.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  sys_id_cl_run(in_flight);
  stabilization_attitude_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
  // not needed
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
  // not needed
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
  // not needed
}
