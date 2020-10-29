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
 * @file modules/system_identification/sys_id_base.c
 * @brief module that executes sys_id maneuvers
 *
 */

#include "modules/system_identification/sys_id_base.h"
#include "state.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/actuators.h"
#include "filters/low_pass_filter.h"
#include "mcu_periph/sys_time.h"
#include "math/pprz_random.h"

struct sys_id_base_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;

} sys_id_base;

float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
float ctrl_module_demo_pr_d_gain;
float ctrl_module_demo_y_ff_gain;   // Yaw
float ctrl_module_demo_y_d_gain;

uint8_t actuators_combination = 0;

uint8_t signal_type_elevons = 0;
uint8_t increase_first_elevons = 1;
pprz_t amplitude_elevons = 0;
float timestep_elevons = 0.1;
float fstart_chirp_elevons = 0.05;
float fend_chirp_elevons = 0.1;
float length_chirp_elevons = 0.1;
float chirp_noise_ratio_elevons = 0;

uint8_t signal_type_motors = 0;
uint8_t increase_first_motors = 1;
pprz_t amplitude_motors = 0;
float timestep_motors = 0.1;
float fstart_chirp_motors = 0.05;
float fend_chirp_motors = 0.1;
float length_chirp_motors = 0.1;
float chirp_noise_ratio_motors = 0;

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

struct signal_config_t signal_elevons;
struct signal_config_t signal_motors;

float trim_state[4];
float final_trim[4];

float delta_input_elevons;
float delta_input_motors;

struct FirstOrderLowPass filter_trim_le;
struct FirstOrderLowPass filter_trim_re;
struct FirstOrderLowPass filter_trim_rm;
struct FirstOrderLowPass filter_trim_lm;

#define CHIRP_C1 4.0f
#define CHIRP_C2 1.0f / (expf(CHIRP_C1) - 1)

void sys_id_base_init(void);
void sys_id_base_run(bool in_flight);
void sys_id_base_enter(void);
float sid_signal_generator(struct signal_config_t *signal, float current_time);

void sys_id_base_init(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averageFirstOrderLowPassd_command

  sys_id_base.rc_x = 0;
  sys_id_base.rc_y = 0;
  sys_id_base.rc_z = 0;
  sys_id_base.rc_t = 0;

  final_trim[0] = trim_state[0];
  final_trim[1] = trim_state[1];
  final_trim[2] = trim_state[2];
  final_trim[3] = trim_state[3];

  signal_elevons.signal_type = signal_type_elevons;
  signal_elevons.increase_first = increase_first_elevons;
  signal_elevons.amplitude = amplitude_elevons;
  signal_elevons.timestep = timestep_elevons;
  signal_elevons.fstart_chirp = fstart_chirp_elevons;
  signal_elevons.fend_chirp = fend_chirp_elevons;
  signal_elevons.length_chirp = length_chirp_elevons;
  signal_elevons.chirp_noise_ratio = chirp_noise_ratio_elevons;
  signal_elevons.time_start = 0.f;

  signal_motors.signal_type = signal_type_motors;
  signal_motors.increase_first = increase_first_motors;
  signal_motors.amplitude = amplitude_motors;
  signal_motors.timestep = timestep_motors;
  signal_motors.fstart_chirp = fstart_chirp_motors;
  signal_motors.fend_chirp = fend_chirp_motors;
  signal_motors.length_chirp = length_chirp_motors;
  signal_motors.chirp_noise_ratio = chirp_noise_ratio_motors;
  signal_motors.time_start = 0.f;

  float sample_time = 0.002f;

  float tau_filter_noise_elevons = 1 / (signal_elevons.fend_chirp * 2 * M_PI);
  init_first_order_low_pass(&signal_elevons.filter_noise, tau_filter_noise_elevons, sample_time, 0);

  float tau_filter_noise_motors = 1 / (signal_motors.fend_chirp * 2 * M_PI);
  init_first_order_low_pass(&signal_motors.filter_noise, tau_filter_noise_motors, sample_time, 0);

  init_random();

  delta_input_elevons = 0.f;
  delta_input_motors = 0.f;
}

void sys_id_base_enter(void)
{
  // TODO: save the actuator values to the trim state
  // trim_state = last_averaged_command

  sys_id_base_init();

  float time_start = get_sys_time_float();
  signal_elevons.time_start = time_start;
  signal_motors.time_start = time_start;

  float amplitude_max_le = (MAX_PPRZ - ABS(final_trim[0]));
  float amplitude_max_re = (MAX_PPRZ - ABS(final_trim[1]));

  float amplitude_max_elevons = Min(amplitude_max_le,amplitude_max_re);
  signal_elevons.amplitude = Min(amplitude_elevons,amplitude_max_elevons);

  float amplitude_max_rm = (MAX_PPRZ - ABS(final_trim[2]));
  float amplitude_max_lm = (MAX_PPRZ - ABS(final_trim[3]));

  float amplitude_max_motors = Min(amplitude_max_rm,amplitude_max_lm);
  signal_motors.amplitude = Min(amplitude_motors,amplitude_max_motors);

  if (actuators_combination == 2)
  {
    if (signal_elevons.signal_type == 0)
    {
      signal_motors.time_start += ((2 * signal_elevons.timestep) + time_shift);
    }
    else if (signal_elevons.signal_type == 1)
    {
      signal_motors.time_start += ((7 * signal_elevons.timestep) + time_shift);
    }
    else
    {
      signal_motors.time_start += (signal_elevons.length_chirp + time_shift);
    }
  }

  if (actuators_combination == 3)
  {
    if (signal_motors.signal_type == 0)
    {
      signal_elevons.time_start += ((2 * signal_motors.timestep) + time_shift);
    }
    else if (signal_motors.signal_type == 1)
    {
      signal_elevons.time_start += ((7 * signal_motors.timestep) + time_shift);
    }
    else
    {
      signal_elevons.time_start += (signal_motors.length_chirp + time_shift);
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
    else
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
    
  delta_input = (signal->amplitude) * delta_input_normalized;
  }
  
  return delta_input;
}

// simple rate control without reference model nor attitude
void sys_id_base_run(bool in_flight __attribute__((unused)))
{
  // Signal generation for the different actuators cases

  float time = get_sys_time_float();

  if (actuators_combination == 0)
  {
    delta_input_elevons = sid_signal_generator(&signal_elevons,time);
  }
  else if (actuators_combination == 1)
  {
    delta_input_motors = sid_signal_generator(&signal_motors,time);
  }
  else if (actuators_combination == 2)
  {
    delta_input_elevons = sid_signal_generator(&signal_elevons,time);
    delta_input_motors = sid_signal_generator(&signal_motors,time);
  }
  else
  {
    delta_input_motors = sid_signal_generator(&signal_motors,time);
    delta_input_elevons = sid_signal_generator(&signal_elevons,time);
  }

  float total_input_le = final_trim[0] - delta_input_elevons;
  float total_input_re = final_trim[1] + delta_input_elevons;
  float total_input_rm = final_trim[2] + delta_input_motors;
  float total_input_lm = final_trim[3] + delta_input_motors;

  BoundAbs(total_input_le, MAX_PPRZ);
  BoundAbs(total_input_re, MAX_PPRZ);
  BoundAbs(total_input_rm, MAX_PPRZ);
  BoundAbs(total_input_lm, MAX_PPRZ);

  // Execute commands with bounds on the actuators
  actuators_pprz[0] = total_input_le;
  actuators_pprz[1] = total_input_re;
  actuators_pprz[2] = total_input_rm;
  actuators_pprz[3] = total_input_lm;
}

// Only use for things that need to run when the module is not active!
// For instance keeping track of the trim condition

void sys_id_base_periodic_init(void)
{
  trim_state[0] = actuators_pprz[0];
  trim_state[1] = actuators_pprz[1];
  trim_state[2] = actuators_pprz[2];
  trim_state[3] = actuators_pprz[3];

  float sample_time = 0.002f;

  // Filter flaps
  // float co_freq_elevons = 41.69f
  float tau_filter_trim_elevons = 1 / (10);
  init_first_order_low_pass(&filter_trim_le, tau_filter_trim_elevons, sample_time, trim_state[0]);
  init_first_order_low_pass(&filter_trim_re, tau_filter_trim_elevons, sample_time, trim_state[1]);

  // Filter motors
  // float co_freq_motors = 20.0f
  float tau_filter_trim_motors = 1 / (10);
  init_first_order_low_pass(&filter_trim_rm, tau_filter_trim_motors, sample_time, trim_state[2]);
  init_first_order_low_pass(&filter_trim_lm, tau_filter_trim_motors, sample_time, trim_state[3]);

}

void sys_id_base_periodic(void)
{
  //TODO average out commands (actuators_pprz)
  //
  trim_state[0] =  update_first_order_low_pass(&filter_trim_le, actuators_pprz[0]);
  trim_state[1] =  update_first_order_low_pass(&filter_trim_re, actuators_pprz[1]);
  trim_state[2] =  update_first_order_low_pass(&filter_trim_rm, actuators_pprz[2]);
  trim_state[3] =  update_first_order_low_pass(&filter_trim_lm, actuators_pprz[3]);
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  sys_id_base_init();
}

void guidance_h_module_enter(void)
{
  sys_id_base_enter();
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  sys_id_base.rc_t = radio_control.values[RADIO_THROTTLE];
  sys_id_base.rc_x = radio_control.values[RADIO_ROLL];
  sys_id_base.rc_y = radio_control.values[RADIO_PITCH];
  sys_id_base.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  sys_id_base_run(in_flight);
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
