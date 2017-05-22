/*
 * Copyright (C) 2016 Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file modules/loggers/log_indi.c
 *  @brief log INDI data for parameter estimation
 */

#include "log_indi.h"
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "subsystems/actuators.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"

#include "modules/loggers/sdlog_chibios.h"


void logger_start(void) {
}

void logger_stop(void) {
}

/** Log the values to a csv file */
void logger_periodic(void)
{
#define LOG_LENGTH_INT 10
#define LOG_LENGTH_FLOAT 7

  int32_t sd_buffer_i[LOG_LENGTH_INT] = {0};
  float sd_buffer_f[LOG_LENGTH_FLOAT] = {0};

  static uint32_t log_counter = 0;
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatRates *body_rates_f = stateGetBodyRates_f();
  struct Int32Vect3 *accelbody = stateGetAccelBody_i();

  sd_buffer_i[0] = log_counter;
  sd_buffer_i[1] = stabilization_cmd[COMMAND_ROLL];
  sd_buffer_i[2] = stabilization_cmd[COMMAND_PITCH];
  sd_buffer_i[3] = stabilization_cmd[COMMAND_YAW];
  sd_buffer_i[4] = stabilization_cmd[COMMAND_THRUST];
  sd_buffer_i[5] = stab_att_sp_quat.qi;
  sd_buffer_i[6] = stab_att_sp_quat.qx;
  sd_buffer_i[7] = stab_att_sp_quat.qy;
  sd_buffer_i[8] = stab_att_sp_quat.qz;
  sd_buffer_i[9] = accelbody->z;

  sd_buffer_f[0] = body_rates_f->p;
  sd_buffer_f[1] = body_rates_f->q;
  sd_buffer_f[2] = body_rates_f->r;
  sd_buffer_f[3] = quat->qi;
  sd_buffer_f[4] = quat->qx;
  sd_buffer_f[5] = quat->qy;
  sd_buffer_f[6] = quat->qz;

  sdLogWriteRaw(pprzLogFile, (uint8_t*) sd_buffer_i, LOG_LENGTH_INT*4);
  sdLogWriteRaw(pprzLogFile, (uint8_t*) sd_buffer_f, LOG_LENGTH_FLOAT*4);
  log_counter += 1;
}
