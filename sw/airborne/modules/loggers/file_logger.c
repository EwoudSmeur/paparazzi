/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.h"
#include "state.h"
#include "stabilization/stabilization_attitude_rc_setpoint.h"
#include "guidance/guidance_v.h"
#include "subsystems/gps.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s%05d.csv", FILE_LOGGER_PATH, counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;

  struct FloatRates float_rates = *stateGetBodyRates_f();
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  struct Int32Quat *quatsp = &stab_att_sp_quat;
  struct NedCoor_f *accel_ned = stateGetAccelNed_f();
  struct NedCoor_f *speed_ned = stateGetSpeedNed_f();
  struct NedCoor_f *pos_ned = stateGetPositionNed_f();
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_sp) - stateGetPositionNed_f()->z;

  fprintf(file_logger, "%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d\n",
          counter,
          float_rates.p,
          float_rates.q,
          float_rates.r,
          indi_u_in_actuators[0],
          indi_u_in_actuators[1],
          indi_u_in_actuators[2],
          indi_u_in_actuators[3],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz,
          act_obs_rpm[0],
          act_obs_rpm[1],
          act_obs_rpm[2],
          act_obs_rpm[3],
          angular_accel_ref.p,
          angular_accel_ref.q,
          angular_accel_ref.r,
          quatsp->qi,
          quatsp->qx,
          quatsp->qy,
          quatsp->qz,
          accel_ned->x,
          accel_ned->y,
          accel_ned->z,
          speed_ned->x,
          speed_ned->y,
          speed_ned->z,
          pos_ned->x,
          pos_ned->y,
          pos_ned->z,
          indicontrol.phi,
          indicontrol.theta,
          pos_x_err,
          pos_y_err,
          pos_z_err,
          gps.ecef_vel.x,
          gps.ecef_vel.y,
          gps.ecef_vel.z
         );
  counter++;
}
