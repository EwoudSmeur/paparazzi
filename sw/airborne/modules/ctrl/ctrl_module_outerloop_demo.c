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

// Own Variables

struct ctrl_module_demo_struct {
// RC Inputs
  struct AttitudeRCInput rc_sp;

// Output command
  struct Int32Eulers cmd;

} ctrl;


// Settings
float comode_time = 0;


////////////////////////////////////////////////////////////////////
// Call our controller
void ctrl_module_init(void)
{
  stabilization_attitude_rc_setpoint_init(&ctrl.rc_sp);
  printf("WORKING");
}

void guidance_module_enter(void)
{
  // Store current heading
  ctrl.cmd.psi = stateGetNedToBodyEulers_i()->psi;

  // Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // vertical mode in hover
  guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
}


// accel_d - desired acceleration
float* guidance_function(float d_accel_ref[3])
{
  // Get current angles
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float roll_c = att->phi;
  float pitch_c = att->theta; 
  float yaw_c = att->psi; 

  // Get current angular rates
  struct FloatRates *rates = stateGetBodyRates_f();
  float roll_rate_c = rates->p;
  float pitch_rate_c = rates->q; 
  float yaw_rate_c = rates->r; 

  // DOWNLINK_SEND_PLOP(DefaultChannel, DefaultDevice,  &roll_c, &pitch_c, &yaw_c, &roll_rate_c, &pitch_rate_c, &yaw_rate_c);


  // Setting fixed values for motor force constant and mass. Not sure if these are accurate.
  float mot_force_constant = 0.01;
  float mass = 0.4;

  // GET THRUST. Hard-coding as a constant for now, probably have to change.
  float T = 3.9;

  // Stop thrust from being too small
  // if (T < 0.1f) {
  //   T = 0.1f;
  // }

  // Rotation matrix, replacing eul2rotm(eulerzyx,"ZYX"). This gets the desired acceleration in the body frame
  struct FloatRMat *rot = stateGetNedToBodyRMat_f();

  // Calculate d_accel_ref_b via "matrix" calculation with for loops: rot * d_accel_ref_b 
  float d_accel_ref_b[3];
  for (int i = 0; i < 3; i++) {
    d_accel_ref_b[i] = 0;
    for (int j = 0; j < 3; j++) {
      //d_accel_ref_b[i] += rot[j][i] * d_accel_ref[j]; 
      d_accel_ref_b[i] += rot->m[i * 3 + j] * d_accel_ref[j];  // Hopefully no problems with how the rotation matrix is accessed.
    }
  }

  // Inverse of the control effectiveness matrix. The inverse is directly computed here.
  float B_inverse[3][3] = { {0, 1/T, 0}, {1/T, 0, 0}, {0, 0, -1}};


  // Calculate dcmd via "matrix" calculation with for loops: dcmd = B_inverse * d_accel_ref_b * mass;
  float dcmd[3];
  for (int i = 0; i < 3; i++) {
    dcmd[i] = 0;
    for (int j = 0; j < 3; j++) {
      dcmd[i] += B_inverse[i][j] * d_accel_ref_b[j];
    }
    dcmd[i] *= mass;
  }

  // Quaternion
  struct FloatQuat q[4];
  struct FloatEulers e = {0.0, dcmd[2], dcmd[1]};
  float_quat_of_eulers_zxy(&q, &e); //ASK EWOUD ABOUT ZYX VS ZXY 

  // Make array to return
  static float array[3];
  array[0] = 5*2*q->qx;
  array[1] = -5*2*q->qy;
  array[2] = T + dcmd[2];

  return array;
}

void guidance_module_run(bool in_flight)
{
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot_in_flight(), false, false, &radio_control);

  // YOUR NEW HORIZONTAL OUTERLOOP CONTROLLER GOES HERE
  // ctrl.cmd = CallMyNewHorizontalOuterloopControl(ctrl);

  // DESIRED TRAJECTORY
  static int counter = 0;
  counter +=1;

  // Put in desired acceleration. Can change to position later 
  static float accel_d[3];
  accel_d[0] = cosf(counter/500.0);
  accel_d[1] = sinf(counter/500.0);
  //accel_d[1] = sinf(counter/500.0);
  accel_d[2] = 0.0;

  // Current accelerations
  struct NedCoor_i *accel_actual = stateGetAccelNed_i();
  float accel_a[3];
  accel_a[0] = accel_actual->x;
  accel_a[1] = accel_actual->y;
  accel_a[2] = accel_actual->z;

  // d_accel_ref
  static float d_accel_ref[3];
  d_accel_ref[0] = accel_d[0] - accel_a[0];
  d_accel_ref[1] = accel_d[1] - accel_a[1];
  d_accel_ref[2] = accel_d[2] - accel_a[2];


  // CONTROL LAW
  // Get results of guidance function
  float* rates_ref = guidance_function(d_accel_ref);
  


  // Reference rates
  float roll_v_ref = rates_ref[0];
  float pitch_v_ref = rates_ref[1];
  // float roll_v_ref = 0.0;
  // float pitch_v_ref = 0.0;
  float yaw_v_ref = 0.0; //Keep at zero, at least for now.

  //float T_cmd = rates_ref[2];
  float T_cmd = 3.9; //Hard-coding as a constant for now. probably will have to change

  DOWNLINK_SEND_PLOP(DefaultChannel, DefaultDevice,  &roll_v_ref, &pitch_v_ref, &yaw_v_ref, &T_cmd, &T_cmd, &T_cmd);


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

  // Send control to the drone
  ctrl.cmd.phi = ANGLE_BFP_OF_REAL(delta_u[0]);
  ctrl.cmd.theta = ANGLE_BFP_OF_REAL(delta_u[1]);
  ctrl.cmd.psi = ANGLE_BFP_OF_REAL(delta_u[2]);

  struct StabilizationSetpoint sp = stab_sp_from_eulers_i(&(ctrl.cmd));
  struct ThrustSetpoint th = guidance_v_run(in_flight);

  // execute attitude stabilization:
  stabilization_attitude_run(in_flight, &sp, &th, stabilization.cmd);


  //  struct FloatEulers* att = stateGetNedToBodyEulers_f();
  //  struct FloatRates* rates = stateGetBodyRates_f();

  // static int counter = 0;
  // counter +=1;

  // float roll = sinf(counter/500.0);
  // float pitch = sinf(counter/500.0);
  //OR
  // Desired rates. To be replaced by desired trajectory, with "guidance" function to determine this from position description
  // float roll_v_d = (0.5/512.0)*cosf(counter/512.0);
  // float pitch_v_d = (-0.5/512.0)*sinf(counter/512.0);
  // float yaw_v_d = 0.0;

  // ctrl.cmd.phi = ANGLE_BFP_OF_REAL(roll);
  // ctrl.cmd.theta = ANGLE_BFP_OF_REAL(pitch);

  // struct StabilizationSetpoint sp = stab_sp_from_eulers_i(&(ctrl.cmd));
  // struct ThrustSetpoint th = guidance_v_run(in_flight);

  // // execute attitude stabilization:
  // stabilization_attitude_run(in_flight, &sp, &th, stabilization.cmd);
}

