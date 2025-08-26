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
 * @file modules/ctrl/ctrl_module_outerloop_demo.h
 * @brief example empty controller
 *
 */

#include "modules/ctrl/ctrl_module_outerloop_demo.h"
#include "state.h"
#include "modules/radio_control/radio_control.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"

#include "modules/imu/imu.h"
#include "modules/core/abi_sender_ids.h"

#include "modules/datalink/downlink.h"

#include "modules/datalink/downlink.h"
#include "mcu_periph/sys_time.h"
#include <math.h>


#include <stdio.h>

// Access estimated thrust from stabilization_indi.c file. This is estimated thrust in the z direction
extern float thrust_estimate;


// Gains and limits
static float vel_limit = 15.0;
static float acc_limit = 2.5;
static float thrust_limit = 0.1;
static float vel_gain = 0.6;
static float acc_gain = 1.2;
static float roll_rate_gain = 35.0;
static float pitch_rate_gain = 35.0;


struct ctrl_module_demo_struct {
// RC Inputs
  struct AttitudeRCInput rc_sp;

// Output command
  // struct Int32Eulers cmd;
  struct FloatRates cmd;

} ctrl;


// Settings
float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
void ctrl_module_init(void)
{
  stabilization_attitude_rc_setpoint_init(&ctrl.rc_sp);
}

void guidance_module_enter(void)
{
  // Store current heading
  ctrl.cmd.r = stateGetBodyRates_f()->r;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // vertical mode in hover
  guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
}


float* guidance_function(float d_accel_ref[3]);


void guidance_module_run(bool in_flight)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // DESIRED TRAJECTORY
  static int counter = 0;
  counter += 1;

  // Desired position
  static float pos_ref[3];
  pos_ref[0] = 3 * sinf(counter/420.0);
  // pos_ref[0] = 0.0;
  pos_ref[1] = 0.0;
  pos_ref[2] = -4.0;
  // pos_ref[2] = -15.0 + sinf(counter/420.0);

  // Current positions
  struct NedCoor_f *pos_actual = stateGetPositionNed_f();
  float pos_a[3];
  pos_a[0] = pos_actual->x;
  pos_a[1] = pos_actual->y;
  pos_a[2] = pos_actual->z;

  // Difference in positions
  float pos_error[3];
  pos_error[0] = pos_ref[0] - pos_a[0];
  pos_error[1] = pos_ref[1] - pos_a[1];
  pos_error[2] = pos_ref[2] - pos_a[2];

  // Trying to compute velocity as a gain times the position error. This is done in the MatLAB file
  float vel_ref[3];
  for (int i = 0; i < 3; i++) {
      vel_ref[i] = pos_error[i] * vel_gain;   // Gain to get velocity  0.95
      if (vel_ref[i] >= vel_limit) {
        vel_ref[i] = vel_limit;
      }
      if (vel_ref[i] <= -vel_limit) {
        vel_ref[i] = -vel_limit;
      }
  } 

  // Current speeds - plots give negative values, so I'm guessing that it is velocity and not speed
  struct NedCoor_f *vel_actual = stateGetSpeedNed_f();
  float vel_a[3];
  vel_a[0] = vel_actual->x;
  vel_a[1] = vel_actual->y;
  vel_a[2] = vel_actual->z;

  // Difference in speeds
  float vel_error[3];
  vel_error[0] = vel_ref[0] - vel_a[0];
  vel_error[1] = vel_ref[1] - vel_a[1];
  vel_error[2] = vel_ref[2] - vel_a[2];


  // Trying to compute acceleration as a gain times the velocity error. This is done in the MatLAB file
  float accel_ref[3];
  for (int i = 0; i < 3; i++) {
      accel_ref[i] = vel_error[i] * acc_gain;   // Gain to get acceleration    2  
      if (accel_ref[i] >= acc_limit) {
        accel_ref[i] = acc_limit;
      } 
      if (accel_ref[i] <= -acc_limit) {
        accel_ref[i] = -acc_limit;
      }
  } 

  // Current accelerations
  struct NedCoor_f *accel_actual = stateGetAccelNed_f();

  float accel_a[3];
  accel_a[0] = accel_actual->x;
  accel_a[1] = accel_actual->y;
  accel_a[2] = accel_actual->z;

  // d_accel_ref
  static float d_accel_ref[3];
  d_accel_ref[0] = accel_ref[0] - accel_a[0];
  d_accel_ref[1] = accel_ref[1] - accel_a[1];
  d_accel_ref[2] = accel_ref[2] - accel_a[2]; 

  // CONTROL LAW
  // Get results of guidance function
  float* rates_guidance = guidance_function(d_accel_ref);
  
  // Send control to the drone (angular rates)
  ctrl.cmd.p = rates_guidance[0];
  ctrl.cmd.q = rates_guidance[1];
  ctrl.cmd.r = 0.0;


  struct StabilizationSetpoint sp = stab_sp_from_rates_f(&(ctrl.cmd));
  struct ThrustSetpoint th = th_sp_from_incr_f(rates_guidance[2], THRUST_AXIS_Z);

  // execute attitude stabilization:
  stabilization_rate_run(in_flight, &sp, &th, stabilization.cmd);

  //---------------------SELF-DONE LOGGING-------------------
  const char *path = "/home/t/paparazzi/output.txt";
  // Try to open the file in "read" mode to check if it already exists
  FILE *check = fopen(path, "r");
  bool file_exists = (check != NULL);
  if (check) fclose(check);

  // Open the file in "append" mode so we don't overwrite existing data
  FILE *file = fopen(path, "a");
  if (file == NULL) {
      perror("Error opening file");
  }

  // Write header only if the file did not exist before
  if (!file_exists) {
      fprintf(file, "Time, pos_a, pos_ref, vel_a, vel_ref, accel_a, accel_ref\n");
      // fprintf(file, "Time, pos_a, pos_ref, vel_a, vel_ref, accel_a, accel_ref, incr_thrust_cmd, total thrust\n");
      // fprintf(file, "Time, ctrl.cmd.p, ctrl.cmd.r\n");
      // fprintf(file, "Time,accel_ref_x,accel_a_x\n");
      // fprintf(file, "Time, thrust_commanded, total_thrust\n");
  }

  // Write the current data values to the file
  fprintf(file, "%d,%f,%f,%f,%f,%f,%f\n", counter, pos_a[0], pos_ref[0], vel_a[0], vel_ref[0], accel_a[0], accel_ref[0]);
  // fprintf(file, "%d,%f,%f,%f,%f,%f,%f,%f,%f\n", counter, pos_a[0], pos_ref[0], vel_a[0], vel_ref[0], accel_a[0], accel_ref[0], rates_guidance[2], rates_guidance[2] + thrust_estimate);
  // fprintf(file, "%d,%f,%f\n", counter, ctrl.cmd.p, ctrl.cmd.r);
  // fprintf(file, "%d,%f,%f\n", counter, accel_ref[0], accel_a[0]);
  // fprintf(file, "%d,%f,%f\n", counter, rates_guidance[2] + thrust_estimate, thrust_estimate);

  // Close the file
  fclose(file);
  //-----------------END SELF-MADE LOGGING---------------

}

float* guidance_function(float d_accel_ref[3])
{
  // Setting fixed values for mass. Not sure if this is accurate.
  float mass = 0.73;    

  // Get thrust
  // float T = mass*9.81; //Hard-coding as a constant needed for a hover to counteract gravity for now, probably have to change.
  float T = -thrust_estimate;  
  if (T < thrust_limit) {
    T = thrust_limit;
  }

  // Rotation matrix, replacing eul2rotm(eulerzyx,"ZYX"). This gets the desired acceleration in the body frame
  struct FloatRMat *rot = stateGetNedToBodyRMat_f(); 

  // PPRZ ALGEBRA MATRICES
  // Calculate d_accel_ref_b via "matrix" calculation: rot * d_accel_ref_b 
  float d_accel_ref_b[3];
  for (int i = 0; i < 3; i++) {
    d_accel_ref_b[i] = 0;
    for (int j = 0; j < 3; j++) {
      d_accel_ref_b[i] += rot->m[i * 3 + j] * d_accel_ref[j];  
    }
  }


  // Inverse of the control effectiveness matrix. The inverse is directly computed here.
  float B_inverse[3][3] = {{0, 1/T, 0}, {1/T, 0, 0}, {0, 0, 1}};

  // Calculate dcmd via "matrix" calculation: dcmd = B_inverse * d_accel_ref_b * mass;
  float dcmd[3]; //MYB PUT LIMIT (45DEG)

  for (int i = 0; i < 3; i++) {
    dcmd[i] = 0;
    for (int j = 0; j < 3; j++) {
      dcmd[i] += B_inverse[i][j] * d_accel_ref_b[j];
    }
    dcmd[i] *= mass;
  }

  // Quaternion
  struct FloatQuat q; //quat output
  struct FloatEulers e;
  e.psi = 0.0;        
  e.theta = dcmd[1]; 
  e.phi = dcmd[0]; 
  float_quat_of_eulers(&q, &e); //This function employs ZYX, as in MATLab

  // Make array to return
  static float array[3];
  array[0] = roll_rate_gain*2*q.qx;
  array[1] = -pitch_rate_gain*2*q.qy;
  array[2] = dcmd[2]/mass;

  return array;
}


// DOWNLINK_SEND_PLOP(DefaultChannel, DefaultDevice,  &roll_v_ref, &pitch_v_ref, &yaw_v_ref, &T_cmd, &T_cmd, &T_cmd);
// RunOnceEvery(100,printf("%f, %f, %f\n", array[0], array[1], array[2]);)
  