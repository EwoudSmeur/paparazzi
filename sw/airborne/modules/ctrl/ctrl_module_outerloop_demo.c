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

#include "modules/datalink/downlink.h"

#include "modules/datalink/downlink.h"
#include "mcu_periph/sys_time.h"
#include <math.h>


#include <stdio.h>
// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct AttitudeRCInput rc_sp;

// Output command
  struct Int32Eulers cmd;
  // struct Int32Rates cmd;

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
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;
  // ctrl.cmd.r = stateGetBodyRates_f()->r;

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
  counter +=1;

  // Put in desired acceleration. Can change to position later 
  static float accel_d[3];
  // accel_d[0] = sinf(counter/500.0);
  accel_d[0] = 0.0;
  accel_d[1] = 0.0;
  // accel_d[1] = - 5.0 * cosf(counter/500.0);
  // accel_d[1] = sinf(counter/500.0 - M_PI/2);
  accel_d[2] = 0.0;

  // Current accelerations
  struct NedCoor_f *accel_actual = stateGetAccelNed_f();
  //struct EcefCoor_f *accel_actual = stateGetAccelEcef_f();
  float accel_a[3];
  accel_a[0] = accel_actual->x;
  accel_a[1] = accel_actual->y;
  accel_a[2] = accel_actual->z;

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
      fprintf(file, "Time,accel_a[0],accel_a[1],accel_a[2]\n");
  }

  // Write the current data values to the file
  // fprintf(file, "%f,%f,%f,%f\n", get_sys_time_float(), roll_v_ref, pitch_v_ref, yaw_v_ref);
  fprintf(file, "%d,%f,%f,%f\n", counter, accel_a[0],accel_a[1],accel_a[2]);

  // Close the file
  fclose(file);
  //-----------------END SELF-MADE LOGGING---------------

  // Setting fixed values for mass. Not sure if this is accurate.
  float mass = 0.4;

  // d_accel_ref
  static float d_accel_ref[3];
  d_accel_ref[0] = accel_d[0] - accel_a[0];
  d_accel_ref[1] = accel_d[1] - accel_a[1];
  d_accel_ref[2] = accel_d[2] - accel_a[2] + 9.81*mass; //Compensating for downwards gravity, working in NED frame


  // CONTROL LAW
  // Get results of guidance function
  float* rates_guidance = guidance_function(d_accel_ref);

   // Get current angular rates
  struct FloatRates *rates_actual = stateGetBodyRates_f();
  float rates_a[3];
  rates_a[0] = rates_actual->p;
  rates_a[1] = rates_actual->q; 
  rates_a[2] = rates_actual->r; 
  
  // Reference rates (difference between guidance calculated rates and actual rates)
  float roll_v_ref = 9*(rates_guidance[0] - rates_a[0]);
  float pitch_v_ref = 9*(rates_guidance[1] - rates_a[1]);
  float yaw_v_ref = 9*(0.0 - rates_a[2]); //Keep input yaw rate at zero, at least for now.

  float T_cmd = rates_guidance[2];
  
  // Make vector u, holding the roll rates and T_cmd
  float u[4] = {roll_v_ref, pitch_v_ref, yaw_v_ref, T_cmd};

  // B_pseudo_inverse is defined using the values seen in Matlab
  float B_pseudo_inverse[4][4] = {{-0.25, -0.25, -2.5, 25.0}, {0.25, -0.25, 2.5, 25.0}, {0.25, 0.25, -2.5, 25.0}, {-0.25, 0.25, 2.5, 25.0}};

  // Calculate delta_u. Doing this manually.
  float delta_u[4];

  // Matrix multiplication with B_pseudo_inverse to calculate delta_u
  for (int i = 0; i < 4; i++) {
    delta_u[i] = 0;  // Initialize the result element
    for (int j = 0; j < 4; j++) {
      delta_u[i] += B_pseudo_inverse[i][j] * u[j];
    }
  }

  // Send control to the drone (angles)
  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(delta_u[0]);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(delta_u[1]);
  // ctrl.cmd.psi = ANGLE_BFP_OF_REAL(delta_u[2]);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(0.0);

  // ctrl.cmd.phi = ANGLE_BFP_OF_REAL(0.0);
  // ctrl.cmd.theta = ANGLE_BFP_OF_REAL(0.0);
  // ctrl.cmd.psi = ANGLE_BFP_OF_REAL(0.0);

  // Send control to the drone (angular rates)
  // Get current angles
  // struct FloatEulers *att = stateGetNedToBodyEulers_f();
  // float yaw_c = att->psi; 

  // ctrl.cmd.p = RATE_BFP_OF_REAL(rates_guidance[0] - rates_a[0]);
  // ctrl.cmd.q = RATE_BFP_OF_REAL(rates_guidance[1] - rates_a[1]);
  // // ctrl.cmd.r = RATE_BFP_OF_REAL(0.0 - rates_a[2]);
  // ctrl.cmd.r = RATE_BFP_OF_REAL(-3*yaw_c - rates_a[2]);

  // ctrl.cmd.p = RATE_BFP_OF_REAL(0.0);
  // ctrl.cmd.q = RATE_BFP_OF_REAL(0.0);
  // // ctrl.cmd.r = RATE_BFP_OF_REAL(0.0 - rates_a[2]);
  // ctrl.cmd.r = RATE_BFP_OF_REAL(0.0);

  struct StabilizationSetpoint sp = stab_sp_from_eulers_i(&(ctrl.cmd));
  // struct StabilizationSetpoint sp = stab_sp_from_rates_i(&(ctrl.cmd));
  struct ThrustSetpoint th = guidance_v_run(in_flight);

  // execute attitude stabilization:
  stabilization_attitude_run(in_flight, &sp, &th, stabilization.cmd);
}

float* guidance_function(float d_accel_ref[3])
{
  // Setting fixed values for mass. Not sure if this is accurate.
  float mass = 0.4;

  // Get thrust. Hard-coding as a constant needed for a hover to counteract gravity for now, probably have to change.
  float T = mass*9.81; //IS THERE A WAY TO GET THRUST IN PPRZ

  // Rotation matrix, replacing eul2rotm(eulerzyx,"ZYX"). This gets the desired acceleration in the body frame
  struct FloatRMat *rot = stateGetNedToBodyRMat_f(); //I'm guessing this might be zyx, which matches the "unlabelled" function float_quat_of_eulers 

  // Calculate d_accel_ref_b via "matrix" calculation: rot * d_accel_ref_b 
  float d_accel_ref_b[3];
  for (int i = 0; i < 3; i++) {
    d_accel_ref_b[i] = 0;
    for (int j = 0; j < 3; j++) {
      d_accel_ref_b[i] += rot->m[i * 3 + j] * d_accel_ref[j];  // Hopefully no problems with how the rotation matrix is accessed.
    }
  }

  // Inverse of the control effectiveness matrix. The inverse is directly computed here.
  float B_inverse[3][3] = { {0, 1/T, 0}, {1/T, 0, 0}, {0, 0, -1}};

  // Calculate dcmd via "matrix" calculation: dcmd = B_inverse * d_accel_ref_b * mass;
  float dcmd[3];
  for (int i = 0; i < 3; i++) {
    dcmd[i] = 0;
    for (int j = 0; j < 3; j++) {
      dcmd[i] += B_inverse[i][j] * d_accel_ref_b[j];
    }
    dcmd[i] *= mass;
  }

  // Quaternion
  struct FloatQuat q; //quat output
  // struct FloatEulers e = {0.0, dcmd[1], dcmd[0]}; //euler input
  struct FloatEulers e;
  e.psi = 0.0;        
  e.theta = dcmd[1]; 
  e.phi = dcmd[0]; 
  // struct FloatEulers e = {dcmd[0], dcmd[1], dcmd[2]}; //euler input
  //ASK EWOUD ABOUT ZYX VS ZXY 
  float_quat_of_eulers(&q, &e); //This function employs ZYX, as in MATLab
  // float_quat_of_eulers_zxy(&q, &e);

  // Make array to return
  static float array[3];
  array[0] = 5*2*q.qx;
  array[1] = -5*2*q.qy;
  array[2] = T + dcmd[2];

  return array;
}





//   //---------------------SELF-DONE LOGGING-------------------
//   const char *path = "/home/t/output.csv";
//  // Try to open the file in "read" mode to check if it already exists
//   FILE *check = fopen(path, "r");
//   bool file_exists = (check != NULL);
//   if (check) fclose(check);

//   // Open the file in "append" mode so we don't overwrite existing data
//   FILE *file = fopen(path, "a");
//   if (file == NULL) {
//       perror("Error opening file");
//       return 1;
//   }

//   // If file is new, write the header row
//   if (!file_exists) {
//       fprintf(file, "roll_c,pitch_c,yaw_c,roll_rate_c,pitch_rate_c,yaw_rate_c\n");
//   }

//   // Write the current data values to the file
//   fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", get_sys_time_float(), roll_c, pitch_c, yaw_c, roll_rate_c, pitch_rate_c, yaw_rate_c);

//   // Close the file
//   fclose(file);
//   //-----------------END SELF-MADE LOGGING---------------

// DOWNLINK_SEND_PLOP(DefaultChannel, DefaultDevice,  &roll_v_ref, &pitch_v_ref, &yaw_v_ref, &T_cmd, &T_cmd, &T_cmd);
