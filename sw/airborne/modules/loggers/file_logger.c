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
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.h"
#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH "/data/video/usb/"
#endif

/** The file pointer */
static FILE *file_logger;

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
  fclose(file_logger);
  file_logger = NULL;
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

  fprintf(file_logger, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          float_rates.p,
          float_rates.q,
          float_rates.r,
          G1[0][0],
          G1[0][1],
          G1[0][2],
          G1[0][3],
          G1[1][0],
          G1[1][1],
          G1[1][2],
          G1[1][3],
          G1[2][0],
          G1[2][1],
          G1[2][2],
          G1[2][3],
          G2[0],
          G2[1],
          G2[2],
          G2[3],
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
          imu.accel.x,
          imu.accel.y,
          imu.accel.z
         );
  counter++;
}
